"""Hand guiding in the bridge (docs/HandGuide.md 3.7, 3.8, 4 and 6), on the REAL BridgeController, FeetechMotorsBus,
scservo_sdk and GuideBus over FakeServoSerial (only the fake port FAKE-COM4 can open), on a simulated clock: the guide
loop is stepped at 100 Hz and the control frame at 30 Hz by the test, never on threads. A test physics keeps every joint
where the "hand" holds it and reports the duty each servo's goal asks for as Present_Load (sign bit 10). Offline: no
serial port, no depth camera, no live bridge; Home, the joint limits and the calibration file are copies in a temp folder.

    python TestHandGuideBridge.py
"""
from __future__ import annotations

import json
import math
import shutil
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

import bridge_server as Bridge
import HandGuide
from FakeServoSerial import FakePort, FakeServoDevice, Installed
from GuideBus import GuideBus

Names = Bridge.MOTOR_NAMES
Ids = {Name: Index + 1 for Index, Name in enumerate(Names)}
Positions = {1: 3472, 2: 3171, 3: 2688, 4: 856, 5: 1521, 6: 703}
RealMonotonic = time.monotonic
HardwareCalls = []
Owner = "page-1"
# An open pose for the calibration: lift -10, elbow -40, wrist -20 deg (URDF), the rest as Positions
WorkPose = {"shoulder_lift": 142, "elbow_flex": 1216, "wrist_flex": 377}


def GuardedOpener():
    HardwareCalls.append("depth camera opened")
    raise RuntimeError("an offline test reached the real depth camera")


def setUpModule():
    for Patch in (patch.object(Bridge.depth_camera, "Opener", GuardedOpener),
                  patch.object(Bridge, "RaiseProcessPriority", lambda: HardwareCalls.append("priority raised") or "normal")):
        Patch.start()
        unittest.addModuleCleanup(Patch.stop)


def Signed(Value: int) -> int:
    return -(Value & 0x7FFF) if Value & 0x8000 else Value


def Corrupt(Device: FakeServoDevice, Address: int, Changes: dict, Times: int = 1) -> None:
    """The next Times SYNC_READs from Address answer with Changes ({id: {register: bytes}}) in place of the servos'
    tables, then the tables as they are: a corrupted reply whose checksum is good (2026-10-10 in plain Hold: torque off,
    0 V, 90 C and load 0xF501 from a servo that was fine)."""
    Original = Device.Answer
    Left = [Times]

    def Answer(Packet):
        if Left[0] and Packet[2] == 0xFE and Packet[4] == 0x82 and Packet[5] == Address:
            Left[0] -= 1
            Saved = {Id: bytes(Device.Tables[Id]) for Id in Changes}
            for Id, Registers in Changes.items():
                for Register, Value in Registers.items():
                    Device.Tables[Id][Register:Register + len(Value)] = Value
            try:
                return Original(Packet)
            finally:
                for Id, Table in Saved.items():
                    Device.Tables[Id][:] = Table
        return Original(Packet)
    Device.Answer = Answer


class HandPhysics:
    """Every joint stays where the hand holds it (Held, steps); Present_Load is the duty the servo's goal asks for, as
    its position loop would apply it (P 32 -> 8 units per step, dead zone 1, punch 16, clamped at Torque_Limit)."""

    def __init__(Self, Device: FakeServoDevice):
        Self.Device = Device
        Self.Held = dict(Positions)

    def __call__(Self, Device: FakeServoDevice, Seconds: float) -> None:
        for Id, Raw in Self.Held.items():
            Device.SetPosition(Id, Raw)
            if not Device.Tables[Id][40]:
                Device.SetLoad(Id, 0)
                continue
            Error = Signed(Device.Register(Id, 42, 2)) - Raw
            Effective = 0 if abs(Error) <= 1 else Error - (1 if Error > 0 else -1)
            Duty = 8 * Effective + (16 if Effective > 0 else -16 if Effective < 0 else 0)
            Limit = Device.Register(Id, 48, 2)
            Device.SetLoad(Id, max(-Limit, min(Limit, Duty)))


class SweepPhysics:
    """For the calibration's sweeps: every energised joint sits where its servo's position loop balances its load, and
    reports that load as Present_Load: gravity (the URDF model, Heavier times), friction against the motion and a
    velocity slope. A joint let go (torque off) runs on as a floating motor does, decelerating at Decel, and stops."""
    Heavier, Friction, FreeFriction, Slope, Decel = 1.05, 60.0, 30.0, 250.0, 700.0

    def __init__(Self, Device: FakeServoDevice):
        Self.Device = Device
        Self.LastGoal: dict[int, list[float]] = {}
        Self.Pos = {Id: float(Device.Position(Id)) for Id in Positions}
        Self.Spin: dict[int, float] = {}
        Self.Model = HandGuide.GravityModel(p=tuple(V * Self.Heavier for V in HandGuide.URDF_P0))
        Self.Law = HandGuide.HandGuideLaw({J: HandGuide.JointConfig(J) for J in HandGuide.COMP_JOINTS}, HandGuide.GravityModel())
        Self.Blocked: dict[int, float] = {}

    def __call__(Self, Device: FakeServoDevice, Seconds: float) -> None:
        Raw = {Name: Self.Pos[Ids[Name]] for Name in Names}
        Gravity = Self.Law.effort(Self.Model, Raw, 9.0, 1.0)
        for Name in Names:
            Id = Ids[Name]
            if not Device.Tables[Id][40]:
                Speed = Self.Spin.get(Id, 0.0)  # steps/s
                if Speed:
                    Slower = max(0.0, abs(Speed) - Self.Decel * 4096 / 360 * Seconds)
                    Self.Pos[Id] += (abs(Speed) + Slower) / 2 * Seconds * (1 if Speed > 0 else -1)
                    Self.Spin[Id] = math.copysign(Slower, Speed) if Slower else 0.0
                Device.SetPosition(Id, round(Self.Pos[Id]))
                Device.SetLoad(Id, 0)
                Self.LastGoal.pop(Id, None)
                continue
            if Device.Tables[Id][33] == 1:  # velocity mode (the bridge's hold corrections): the fake's own step moved it
                Self.Pos[Id] = float(Device.Position(Id))
                Self.LastGoal.pop(Id, None)
                continue
            Goal = float(Signed(Device.Register(Id, 42, 2)))
            # the goal is whole steps: its speed over the last 0.1 s, as the joint follows it
            History = Self.LastGoal.setdefault(Id, [])
            History.append(Goal)
            del History[:-11]
            Velocity = (History[-1] - History[0]) / (Seconds * max(1, len(History) - 1))  # steps/s
            Self.Spin[Id] = Velocity
            Moving = 1 if Velocity > 1 else -1 if Velocity < -1 else 0
            Load = (Gravity.get(Name, 0.0) + Moving * (Self.Friction if Name in HandGuide.COMP_JOINTS else Self.FreeFriction)
                    + Self.Slope * Velocity * HandGuide.STEP_RAD)
            Sign = 1 if Load > 0 else -1
            Self.Pos[Id] = Self.Blocked.get(Id, Goal - Sign * (1 + max(0.0, abs(Load) - 16) / 8))
            Device.SetPosition(Id, round(Self.Pos[Id]))
            Device.SetLoad(Id, round(Load))


def Calibration(Controller, Device, **Over) -> dict:
    """A calibration file that fits the fake arm (its servos' settings and this repo's zero pose): the URDF model."""
    Settings = GuideBus(Controller.bus, Names).ReadSettings()
    Fingerprint = Controller.HandGuideFingerprint(Settings, Controller.HandGuideReferenceRaw())
    Prior = HandGuide.kv_prior(8.9)
    Data = {"Version": 1, "HandGuideVersion": "guide-v1", "CalibratedAt": "2026-10-10T12:00:00", "VCal": 8.9,
            "Model": list(HandGuide.URDF_P0), "Alpha": 0.95, "CoastBrakes": False, "Fingerprint": Fingerprint,
            "Joints": {**{Name: {"k_v": 0.95 * Prior, "f_kin0": 40.0, "f_kin_load": 0.3, "c_lead": 0.6, "k_f": 0.5}
                          for Name in HandGuide.COMP_JOINTS},
                       **{Name: {"k_v": 0.95 * Prior, "f_kin": 30.0, "c_lead": 0.9, "k_f": 0.5} for Name in HandGuide.FREE_JOINTS}}}
    Data.update(Over)
    return Data


class HandGuideRig(unittest.TestCase):
    def setUp(Self):
        Self.Temp = Path(tempfile.mkdtemp(prefix="HandGuideBridge"))
        Self.addCleanup(shutil.rmtree, Self.Temp, True)
        for Name in ("home_preset.json", "gui_limits.json"):
            shutil.copy(Bridge.BACKEND_ROOT / Name, Self.Temp / Name)
        Self.Device = Self.enterContext(Installed(FakeServoDevice(Positions)))
        Self.Physics = HandPhysics(Self.Device)
        Self.Device.OnStep = Self.Physics
        Self.Clock = [1000.0]  # simulated; None switches to the real clock
        Self.enterContext(patch.object(Bridge.time, "monotonic",
                                       lambda: RealMonotonic() if Self.Clock[0] is None else Self.Clock[0]))
        Self.Controller = Bridge.BridgeController(home_path=Self.Temp / "home_preset.json",
                                                  limits_path=Self.Temp / "gui_limits.json")
        Self.Controller.HandGuideThreaded = False
        Self.Controller.HandGuideNow = lambda: time.perf_counter() if Self.Clock[0] is None else Self.Clock[0]
        Self.Controller.HandGuideCalibrationPath = Self.Temp / "hand_guide_calibration.json"
        Self.Events = []
        Self.Controller.EventSink = Self.Events.append
        Self.Beat = True

    def tearDown(Self):
        Self.assertEqual(HardwareCalls, [], "an offline test reached the depth camera or process priority")

    # -- helpers ------------------------------------------------------------------------------------------------------
    def Hold(Self, Calibrated=True):
        Controller = Self.Controller
        Ok, Text = Controller.connect(FakePort)
        Self.assertTrue(Ok, Text)
        if Controller.torque_enabled:
            Self.assertTrue(Controller.toggle_torque()[0], Controller.status)
        Self.assertTrue(Controller.toggle_torque()[0], Controller.status)
        Self.Frames(0.7)
        Self.assertIsNone(Controller.MotionFault, Controller.status)
        if Calibrated:
            Path(Controller.HandGuideCalibrationPath).write_text(json.dumps(Calibration(Controller, Self.Device)), encoding="utf-8")
        Self.Events.clear()

    def Frames(Self, Seconds):
        """The 30 Hz control loop alone (no session)."""
        for _ in range(round(Seconds * 30)):
            if Self.Clock[0] is None:
                time.sleep(1 / 30)
            else:
                Self.Clock[0] += 1 / 30
            Self.Device.Step(1 / 30)
            Self.Controller.refresh()

    def Guide(Self, Access="local", Connection=Owner, **Message):
        Self.Controller.HandGuideHeartbeat(Connection) if Connection else None
        return Self.Controller.handle_command({"type": "hand_guide", "enable": True, "explicit": True, **Message}, Access,
                                              Connection)

    def Run(Self, Seconds, Each=None):
        """The guide loop at 100 Hz and the control frame at 30 Hz (the supervisor while a session runs), the page's
        heartbeat every 0.5 s while Beat."""
        Controller = Self.Controller
        for Step in range(round(Seconds * 100)):
            Self.Clock[0] += 0.01
            Self.Device.Step(0.01)
            if Each:
                Each(Step)
            if Self.Beat and Step % 50 == 0:
                Controller.HandGuideHeartbeat(Owner)
            with Controller.lock:
                Controller.HandGuideCycle()
            if Step % 3 == 0:
                Controller.refresh()

    def Started(Self):
        Ok, Text = Self.Guide()
        Self.assertTrue(Ok, Text)
        Self.Run(2.6)
        State = Self.Controller.state_snapshot()["hand_guide"]
        Self.assertTrue(State["on"], (Self.Controller.status, State))
        Self.assertEqual(State["phase"], "guide")
        return State

    def Torque(self):
        return {Name: self.Device.Torque()[Ids[Name]] for Name in Names}

    def Register(Self, Name, Address, Length=1):
        return Self.Device.Register(Ids[Name], Address, Length)

    def LoopLimits(Self, Since=0):
        """(name, Torque_Limit) of every goal the guide loop wrote (its SYNC_WRITEs of regs 42..49) from Received[Since]."""
        Names_ = {Id: Name for Name, Id in Ids.items()}
        return [(Names_[Id], int.from_bytes(Data[-2:], "little")) for Instruction, Id, Address, Data in Self.Device.Received[Since:]
                if Instruction == 3 and Address == 42 and len(Data) == 8]

    def AssertLimp(Self, Words):
        Controller = Self.Controller
        Self.assertIsNone(Controller.HandGuide)
        Self.assertEqual(Controller.HandGuideLast["kind"], "coast")
        Self.assertIn(Words, Controller.HandGuideLast["reason"])
        Self.assertEqual(set(Self.Torque().values()), {0}, "every joint limp")
        Self.assertIsNotNone(Controller.ProtectionTrip)
        Self.assertTrue(Controller.ProtectionTrip["coast_confirmed"])
        Self.assertFalse(Controller.torque_enabled)
        Self.assertIn("Protection tripped", Controller.status)

    def AssertHolding(Self, Words):
        Controller = Self.Controller
        Self.assertIsNone(Controller.HandGuide)
        Self.assertEqual(Controller.HandGuideLast["kind"], "preloaded")
        Self.assertIn(Words, Controller.HandGuideLast["reason"])
        Self.assertEqual(set(Self.Torque().values()), {1}, "every joint holds")
        for Name in Names:
            Self.assertEqual(Self.Register(Name, 48, 2), 1000, f"{Name}: Torque_Limit back to 1000")
            Self.assertEqual(Self.Register(Name, 41), 51, f"{Name}: the hold's acceleration")
        Self.assertTrue(Controller.torque_enabled)
        Self.assertIsNone(Controller.MotionFault)
        Self.assertTrue(all(Servo.Holding for Servo in Controller.servos.values()))


class HandGuideBridgeTests(HandGuideRig):
    # -- entry --------------------------------------------------------------------------------------------------------
    def TestEntryRefusals(Self):
        Self.Hold()
        Controller = Self.Controller
        Cases = [({"explicit": False}, "local", Owner, "its own button's click"),
                 ({}, "open", Owner, "this PC or a device with the arm password"),
                 ({}, "local", None, "starts from Clancy's page")]
        for Message, Access, Connection, Words in Cases:
            Ok, Text = Self.Guide(Access, Connection, **Message)
            Self.assertFalse(Ok)
            Self.assertIn(Words, Text)
        # POST /motion has no page connection
        Self.assertIn("starts from Clancy's page", Bridge.MotionCommand({"type": "hand_guide", "enable": True, "explicit": True})["error"])
        Path(Controller.HandGuideCalibrationPath).unlink()
        Ok, Text = Self.Guide()
        Self.assertFalse(Ok)
        Self.assertIn("not calibrated: press Calibrate hand guiding", Text)
        Path(Controller.HandGuideCalibrationPath).write_text(json.dumps(Calibration(Controller, Self.Device)), encoding="utf-8")
        Self.Device.Tables[Ids["elbow_flex"]][21] = 16  # P changed since the calibration
        Ok, Text = Self.Guide()
        Self.assertIn("elbow_flex's settings changed since hand guiding was calibrated (p): calibrate hand guiding again", Text)
        Self.Device.Tables[Ids["elbow_flex"]][21] = 32
        for Volts, Words in ((8.2, "the servo supply reads 8.2 V (at least 8.5 V needed); check the 12 V supply and its cable"),):
            for Id in Ids.values():
                Self.Device.SetVoltage(Id, Volts)
            Ok, Text = Self.Guide()
            Self.assertFalse(Ok)
            Self.assertIn(Words, Text)
        for Id in Ids.values():
            Self.Device.SetVoltage(Id, 9.0)
        Self.Device.SetVoltage(Ids["gripper"], 9.6)
        Self.assertIn("more than 0.5 V apart", Self.Guide()[1])
        Self.Device.SetVoltage(Ids["gripper"], 9.0)
        Self.Device.SetTemperature(Ids["shoulder_lift"], 50)
        Self.assertIn("shoulder_lift 50 C", Self.Guide()[1])
        Self.Device.SetTemperature(Ids["shoulder_lift"], 30)
        Self.Device.Tables[Ids["wrist_flex"]][55] = 0
        Self.assertIn("wrist flex reads Lock 0", Self.Guide()[1])
        Self.Device.Tables[Ids["wrist_flex"]][55] = 1
        Self.Physics.Held[Ids["shoulder_pan"]] += 5
        Self.Frames(0.1)
        Self.assertIn("shoulder pan is moving", Self.Guide()[1])
        Self.Frames(0.6)
        Self.assertTrue(Controller.toggle_torque(False)[0])
        Self.Frames(0.6)
        Self.assertIn("Press Hold first", Self.Guide()[1])
        Self.assertEqual(Self.Torque()["shoulder_lift"], 0, "a refusal writes nothing")
        Self.assertIsNone(Controller.HandGuide)

    def TestStartGuideAndHoldHere(Self):
        Self.Hold()
        Controller = Self.Controller
        Since = len(Self.Device.Received)
        Ok, Text = Self.Guide()
        Self.assertTrue(Ok, Text)
        Self.assertIn("keep your hands off the arm for 2 s", Text)
        State = Controller.state_snapshot()["hand_guide"]
        Self.assertTrue(State["on"])
        Self.assertEqual(State["phase"], "preload")
        Self.assertEqual(State["roles"], {"shoulder_pan": "free", "shoulder_lift": "comp", "elbow_flex": "comp",
                                          "wrist_flex": "comp", "wrist_roll": "free", "gripper": "free"})
        Self.assertEqual(State["owner"], "this PC")
        Self.assertGreater(State["start_left_s"], 2.0)
        Self.Run(2.6)
        State = Controller.state_snapshot()["hand_guide"]
        Self.assertEqual(State["phase"], "guide", Controller.status)
        Self.assertEqual(State["free_states"], dict.fromkeys(HandGuide.FREE_JOINTS, "float"))
        Self.assertIn("If anything goes wrong the arm goes limp", Controller.status)
        Torque = Self.Torque()
        Self.assertEqual({Name: Torque[Name] for Name in HandGuide.FREE_JOINTS}, dict.fromkeys(HandGuide.FREE_JOINTS, 0),
                         "pan, roll and gripper float")
        Self.assertEqual({Name: Torque[Name] for Name in HandGuide.COMP_JOINTS}, dict.fromkeys(HandGuide.COMP_JOINTS, 1))
        for Name, Ceiling in (("shoulder_lift", 650), ("elbow_flex", 450), ("wrist_flex", 250)):
            Self.assertLessEqual(Self.Register(Name, 48, 2), Ceiling, f"{Name}'s effort is capped")
        Self.assertEqual(Self.Register("shoulder_lift", 41), 0, "no acceleration profile while guiding")
        # Nothing in the EEPROM block, and no Operating_Mode write, while guiding.
        Writes = Self.Device.Writes(Since=Since)
        Self.assertEqual([W for W in Writes if W[1] < 40], [])
        Self.assertTrue(all(Instruction in (0x82, 0x83) for Instruction, *_ in Self.Device.SyncPackets[-20:]))
        Self.assertTrue(Controller.MotionInProgress())
        # Hold here: the preloaded exit
        Ok, Text = Controller.handle_command({"type": "hand_guide", "enable": False}, "local", Owner)
        Self.assertTrue(Ok, Text)
        Self.AssertHolding("Hold here")
        Self.assertEqual(Self.Register("shoulder_lift", 46, 2), 0, "Goal_Speed 0 lands the preload at once")
        Self.Frames(0.2)
        Self.assertGreater(Self.Register("shoulder_lift", 46, 2), 0, "the hold's profile speed after 0.1 s")
        Self.Frames(1.0)
        Self.assertIsNone(Controller.MotionFault, Controller.status)
        Self.assertTrue(Controller.torque_enabled)
        Self.assertFalse(Controller.state_snapshot()["hand_guide"]["on"])
        Self.assertEqual(Controller.state_snapshot()["hand_guide"]["last_exit"]["kind"], "preloaded")
        Kinds = [(E["kind"], E["reason"]) for E in Self.Events]
        Self.assertIn(("hand_guide", "Hand guiding on"), Kinds)
        Self.assertTrue(any(K == "hand_guide" and "ended (preloaded)" in R for K, R in Kinds))

    def TestTheHoldingExitsStopAndHoldHoldAndStopDuringTheStart(Self):
        for Name, Act, Words in (("Stop and hold", lambda C: C.handle_command({"type": "stop_motion"}), "Stop and hold"),
                                 ("Hold pressed", lambda C: C.handle_command({"type": "toggle_torque", "enable": True}), "Hold pressed")):
            with Self.subTest(Name):
                Self.setUp()
                Self.Hold()
                Self.Started()
                Ok, Text = Act(Self.Controller)
                Self.assertTrue(Ok, Text)
                Self.AssertHolding(Words)
                Self.Frames(0.5)
                Self.assertIsNone(Self.Controller.MotionFault)
                Self.doCleanups()
        # During the start: back to the entry hold goals (the arm was never handed over)
        Self.setUp()
        Self.Hold()
        Goals = {Name: Self.Register(Name, 42, 2) for Name in HandGuide.COMP_JOINTS}
        Self.assertTrue(Self.Guide()[0])
        Self.Run(0.3)
        Self.Controller.handle_command({"type": "stop_motion"})
        Self.assertEqual(Self.Controller.HandGuideLast["kind"], "anchor")
        Self.assertEqual({Name: Self.Register(Name, 42, 2) for Name in HandGuide.COMP_JOINTS}, Goals)
        Self.assertEqual(set(Self.Torque().values()), {1})

    def TestCoastAndEmergencyStopGoLimp(Self):
        Self.Hold()
        Self.Started()
        Ok, Text = Self.Controller.handle_command({"type": "toggle_torque", "enable": False})
        Self.assertTrue(Ok, Text)
        Self.assertIsNone(Self.Controller.HandGuide)
        Self.assertEqual(set(Self.Torque().values()), {0})
        Self.assertFalse(Self.Controller.torque_enabled)
        Self.assertEqual(Self.Controller.HandGuideLast["kind"], "coast")
        # Hold again writes the normal acceleration profile a coasted session left at 0
        Self.Frames(0.3)
        Self.assertTrue(Self.Controller.toggle_torque(True)[0], Self.Controller.status)
        Self.assertEqual(Self.Register("shoulder_lift", 41), 51)
        Self.Frames(0.7)
        Self.Started()
        Ok, Text = Self.Controller.emergency_stop()
        Self.assertTrue(Ok, Text)
        Self.assertEqual(set(Self.Torque().values()), {0})
        Self.assertFalse(Self.Controller.arm_connected)
        Self.assertEqual(Self.Controller.HandGuideLast["kind"], "coast")

    # -- faults: every one makes the whole arm go limp -------------------------------------------------------------------
    def TestFaultsGoLimp(Self):
        Lift = Ids["shoulder_lift"]
        Cases = [("55 C", lambda: Self.Device.SetTemperature(Lift, 55), " C: every joint goes limp"),
                 ("firmware overload", lambda: Self.Device.Tables[Lift].__setitem__(65, 32), "firmware protection"),
                 ("supply 6.8 V", lambda: [Self.Device.SetVoltage(Id, 6.8) for Id in Ids.values()], "below 7.0 V"),
                 ("restart", lambda: Self.Device.Restart(Ids["elbow_flex"]), "elbow_flex"),
                 ("model error", lambda: Self.Controller.HandGuide.Law.model.scale.__setitem__("elbow_flex", -1.0),
                  "disagrees with the URDF")]
        for Name, Fault, Words in Cases:
            with Self.subTest(Name):
                Self.setUp()
                Self.Hold()
                Self.Started()
                Fault()
                Self.Run(0.4)  # (heat must last over GuideParams.temp_exit_hold_s, 0.25 s)
                Self.AssertLimp(Words)
                Self.Frames(0.3)
                Self.assertEqual(set(Self.Torque().values()), {0}, "and it stays limp")
                Self.doCleanups()

    def TestABriefHotReadingDoesNotEndGuiding(Self):
        """One corrupted status packet read 57-62 C on a 27 C wrist (2026-10-10): heat ends guiding only once it has
        lasted over temp_exit_hold_s (0.25 s), and a reading that drops back starts the count again."""
        Wrist = Ids["wrist_flex"]
        Self.Hold()
        Self.Started()
        for _ in range(3):
            Self.Device.SetTemperature(Wrist, 62)
            Self.Run(0.1)
            Self.Device.SetTemperature(Wrist, 27)
            Self.Run(0.1)
        Self.assertIsNotNone(Self.Controller.HandGuide, "brief hot readings, each under 0.25 s: still guiding")
        Self.Device.SetTemperature(Wrist, 62)
        Self.Run(0.4)
        Self.AssertLimp(" C: every joint goes limp")
        Self.assertIn("over 250 ms", Self.Controller.HandGuideLast["reason"])

    def TestASilentServoGoesLimpAndItsTorqueOffIsSentWhenItAnswers(Self):
        Self.Hold()
        Self.Started()
        Self.Device.Silent = {Ids["elbow_flex"]}
        Self.Run(0.1)
        Controller = Self.Controller
        Self.assertIsNone(Controller.HandGuide)
        Self.assertIn("elbow_flex stopped answering", Controller.HandGuideLast["reason"])
        Self.assertFalse(Controller.ProtectionTrip["coast_confirmed"])
        Self.assertIn("elbow_flex", Controller.ProtectionTrip["unconfirmed"])
        Self.assertEqual(Self.Torque()["shoulder_lift"], 0, "every joint that answers is limp at once")
        Self.Device.Silent = set()
        Self.Frames(0.6)
        Self.assertTrue(Controller.ProtectionTrip["coast_confirmed"])
        Self.assertEqual(set(Self.Torque().values()), {0})

    def TestAUsbDropLatchesAndCoastsTheMomentTheLinkReturns(Self):
        Self.Hold()
        Self.Started()
        Self.Device.Remove()
        Self.Run(0.05)
        Controller = Self.Controller
        Self.assertIsNone(Controller.HandGuide)
        Self.assertTrue(Controller.SerialLinkLost())
        Self.assertEqual(Controller.HandGuideLast["kind"], "coast")
        Self.assertIn("USB serial link to the arm dropped", Controller.MotionFault)
        Self.assertEqual(Self.Torque()["shoulder_lift"], 1, "nothing can be written meanwhile: the servos keep their goals")
        Self.Device.Arrive()
        Self.Frames(0.5)
        Self.assertFalse(Controller.SerialLinkLost())
        Self.assertTrue(Controller.ProtectionTrip["coast_confirmed"])
        Self.assertEqual(set(Self.Torque().values()), {0}, "limp the moment the link is back")

    def TestALateGuideLoopGoesLimpAndAGonePageHolds(Self):
        Self.Hold()
        Self.Started()
        # the guide loop stops cycling: the 30 Hz supervisor sees it 0.15 s later
        for _ in range(8):
            Self.Clock[0] += 1 / 30
            Self.Controller.refresh()
        Self.AssertLimp("hand-guiding loop paused")
        # the page that started it stops its heartbeats: Hold here after 3 s (not limp)
        Self.setUp()
        Self.Hold()
        Self.Started()
        Self.Beat = False
        Self.Run(2.5)
        Self.assertIsNotNone(Self.Controller.HandGuide, "2.5 s without the page: still guiding")
        Self.Run(1.0)
        Self.AssertHolding("the page that started it (this PC) closed or went to sleep")

    def TestTheCollisionGuardBandCatchesTheFreeJointsAndItsLossGoesLimp(Self):
        Self.Hold()
        Self.Started()
        Controller = Self.Controller
        with patch.object(Controller, "HandGuideGuardBand", lambda Session: True):
            Self.Run(0.2)
        Self.assertEqual(Controller.HandGuide.Law.fstate, dict.fromkeys(HandGuide.FREE_JOINTS, "caught"))
        Self.assertEqual({Name: Self.Torque()[Name] for Name in HandGuide.FREE_JOINTS}, dict.fromkeys(HandGuide.FREE_JOINTS, 1))
        Self.assertEqual(Self.Register("wrist_roll", 48, 2), 400)
        Self.assertEqual(Self.Register("shoulder_lift", 48, 2), 750, "walls hold at 750, under the 800 overload trip")
        Self.Run(0.2)
        Self.assertEqual(Controller.HandGuide.Law.fstate, dict.fromkeys(HandGuide.FREE_JOINTS, "float"))
        with patch.object(Controller, "HandGuideGuardBand", lambda Session: None):
            Self.Run(0.3)
        Self.AssertLimp("collision check unavailable")

    # -- the help: a hand turning the roll slowly gets it -------------------------------------------------------------
    def TestASlowSteadyTurnOfTheRollIsAssistedAndItFloatsAgainAtRest(Self):
        Self.Hold()
        Self.Started()
        Roll = Ids["wrist_roll"]

        def Turn(Step):  # 12 deg/s, steady
            if Step < 150:
                Self.Physics.Held[Roll] = Positions[Roll] + round((Step + 1) * 0.01 * 12 * 4096 / 360)
        Self.Run(1.5, Turn)
        Law = Self.Controller.HandGuide.Law
        Self.assertEqual(Law.fstate["wrist_roll"], "assist", Law.assist_counts)
        Self.assertEqual(Self.Torque()["wrist_roll"], 1)
        Goal = Signed(Self.Register("wrist_roll", 42, 2))
        Self.assertGreater(Goal, Self.Physics.Held[Roll], "the assist pushes the way the hand turns it")
        Self.assertLessEqual(Self.Register("wrist_roll", 48, 2), 400)
        Self.Run(0.5)
        Self.assertEqual(Law.fstate["wrist_roll"], "float", "at rest it floats again")
        Self.assertEqual(Self.Torque()["wrist_roll"], 0)

    # -- everything else while guiding ----------------------------------------------------------------------------------
    def TestOtherCommandsWhileGuiding(Self):
        Self.Hold()
        Self.Started()
        Controller = Self.Controller
        for Message, Words in (({"type": "move_servo", "name": "elbow_flex", "raw": 2000}, "Hand guiding is on: press Hold here first"),
                               ({"type": "go_home"}, "Hand guiding is on: press Hold here before pressing Home"),
                               ({"type": "send_ik", "target": {"x": 0, "y": 20, "z": 10}}, "Hand guiding is on"),
                               ({"type": "set_home"}, "Hand guiding is on: press Hold here before setting Home"),
                               ({"type": "set_servo_limits", "name": "gripper", "min": 642, "max": 2242}, "Hand guiding is on"),
                               ({"type": "hand_guide_calibrate", "explicit": True}, "Press Hold here first"),
                               ({"type": "connect", "port": FakePort, "explicit": True}, "already connected")):
            Ok, Text = Controller.handle_command(Message, "local", Owner)
            Self.assertIn(Words, Text, Message)
            Self.assertIsNotNone(Controller.HandGuide, f"{Message['type']} left guiding on")
        Ok, Text = Self.Guide()
        Self.assertTrue(Ok)
        Self.assertEqual(Text, "Hand guiding is already on")
        Self.assertEqual(Controller.CaptureArmPose(time.time())["reason"], "hand guiding")
        Self.assertIn("Hand guiding is on", Controller.CaptureCadReference(True)["error"])
        Self.assertIn("Hand guiding is on", Controller.PrepareCalibration()["error"])
        Self.assertTrue(Bridge.Preempts({"type": "hand_guide", "enable": False}))
        Self.assertFalse(Bridge.Preempts({"type": "hand_guide", "enable": True}))
        Self.Run(0.3)
        Self.assertIsNotNone(Controller.HandGuide)
        Self.assertIsNone(Controller.MotionFault)

    def TestAnotherDeviceMayGuideAndTheGripperKeepsAGrip(Self):
        Self.Hold()
        Controller = Self.Controller
        Controller.servos["gripper"].GraspHold = True
        Ok, Text = Self.Guide("password")
        Self.assertTrue(Ok, Text)
        Self.Run(2.6)
        State = Controller.state_snapshot()["hand_guide"]
        Self.assertEqual(State["owner"], "another device")
        Self.assertEqual(State["roles"]["gripper"], "keep")
        Self.assertEqual(Self.Torque()["gripper"], 1, "a gripper squeezing an object keeps its squeeze")
        Self.assertEqual(Self.Torque()["wrist_roll"], 0)
        Self.assertTrue(Controller.handle_command({"type": "hand_guide", "enable": False})[0])
        Self.assertTrue(Controller.servos["gripper"].GraspHold)
        # Home afterwards leaves that grip as it is too (HomeGripperPlan): it neither opens nor closes the jaws.
        Targets = {Name: Controller._clamp_raw(Servo, Controller.home_preset[Name]) for Name, Servo in Controller.servos.items()}
        Self.assertEqual(Controller.HomeGripperPlan(Targets), (("hold", None), Bridge.HOME_GRIPPER_HELD_NOTE))

    def TestTheGuideLoopRunsOnItsOwnThreadAt100Hz(Self):
        Self.Clock[0] = None  # the real clock from here
        Self.Hold()
        Controller = Self.Controller
        Controller.HandGuideThreaded = True
        Ok, Text = Self.Guide()
        Self.assertTrue(Ok, Text)
        Session = Controller.HandGuide
        Self.assertTrue(Session.Thread.is_alive())
        Started = time.perf_counter()
        Slowest = 0.0
        while time.perf_counter() - Started < 3.0:
            Controller.HandGuideHeartbeat(Owner)
            Self.Device.Step(1 / 30)
            Frame = time.perf_counter()
            Controller.refresh()
            Slowest = max(Slowest, time.perf_counter() - Frame)
            time.sleep(1 / 30)
        Self.assertIs(Controller.HandGuide, Session, Controller.status)
        Self.assertEqual(Session.Phase, "guide")
        Self.assertGreater(Session.Cycle, 200, "about 100 cycles a second")
        Self.assertLess(Slowest, 0.05, "the supervisor frame (with its guard band check) stays short")
        Self.assertTrue(Controller.handle_command({"type": "hand_guide", "enable": False})[0])
        Session.Thread.join(timeout=1.0)
        Self.assertFalse(Session.Thread.is_alive(), "the guide loop ends with its session")
        Self.AssertHolding("Hold here")

    # -- the calibration -----------------------------------------------------------------------------------------------
    def Calibrate(Self, Start=True):
        """Hold at an open pose (lift -10, elbow -40, wrist -20 deg), then the sweep physics and Calibrate hand guiding."""
        for Name, Raw in WorkPose.items():
            Self.Physics.Held[Ids[Name]] = Raw
            Self.Device.SetPosition(Ids[Name], Raw)
        Self.Hold(Calibrated=False)
        Self.Device.OnStep = SweepPhysics(Self.Device)
        if not Start:
            return None
        return Self.Controller.handle_command({"type": "hand_guide_calibrate", "explicit": True}, "local", Owner)

    def TestCalibrateHandGuidingSavesACalibrationThatThenGuides(Self):
        Ok, Text = Self.Calibrate()
        Self.assertTrue(Ok, Text)
        Self.assertIn("Calibrating hand guiding (about 3 min)", Text)
        Controller = Self.Controller
        State = Controller.state_snapshot()["hand_guide"]
        Self.assertTrue(State["calibrating"])
        Self.assertFalse(State["on"])
        Self.Run(370)
        Result = Controller.HandGuideCalibrationResult
        Self.assertIsNone(Controller.HandGuide, Controller.status)
        Self.assertTrue(Result["ok"], Result)
        Self.assertIn("finished and saved: hand guiding is ready", Controller.HandGuideLast["reason"])
        Self.AssertHolding("Hand-guiding calibration finished and saved")
        Data = json.loads(Path(Controller.HandGuideCalibrationPath).read_text(encoding="utf-8"))
        Self.assertEqual(Data["Version"], 1)
        Self.assertFalse(Data["CoastBrakes"], "the spin-down found a floating driver")
        for J in HandGuide.COMP_JOINTS:
            Self.assertAlmostEqual(Data["Joints"][J]["k_v"], SweepPhysics.Slope, delta=10, msg=J)
        for J in HandGuide.FREE_JOINTS:
            Self.assertAlmostEqual(Data["Joints"][J]["f_kin"], SweepPhysics.FreeFriction, delta=5, msg=J)
        Self.assertTrue(Controller.state_snapshot()["hand_guide"]["calibrated"])
        json.dumps(Controller.state_snapshot(), allow_nan=False)  # the page's JSON.parse takes no NaN
        # and hand guiding starts with it
        Self.Device.OnStep = Self.Physics
        Self.Physics.Held = {Id: Self.Device.Position(Id) for Id in Ids.values()}
        Self.Frames(0.7)
        Self.Started()

    def TestTheCalibrationStopsHoldingWhenBlockedOrStopped(Self):
        Ok, Text = Self.Calibrate()
        Self.assertTrue(Ok, Text)
        Self.Run(3.0)
        Self.Device.OnStep.Blocked[Ids["wrist_flex"]] = Self.Device.Position(Ids["wrist_flex"])  # a hand holds it
        Self.Run(3.0)
        Controller = Self.Controller
        Self.assertIsNone(Controller.HandGuide)
        Self.assertIn("off its calibration sweep", Controller.HandGuideCalibrationResult["error"])
        Self.assertIn("off its calibration sweep", Controller.state_snapshot()["hand_guide"]["calibration_result"]["error"])
        json.dumps(Controller.state_snapshot(), allow_nan=False)
        Self.assertEqual(set(Self.Torque().values()), {1}, "it stops holding, not limp")
        Self.assertFalse(Path(Controller.HandGuideCalibrationPath).exists(), "nothing saved")
        # Stop and hold mid-way
        Self.setUp()
        Self.assertTrue(Self.Calibrate()[0])
        Self.Run(2.0)
        Self.assertTrue(Self.Controller.handle_command({"type": "stop_motion"})[0])
        Self.assertIsNone(Self.Controller.HandGuide)
        Self.assertEqual(Self.Controller.HandGuideLast["kind"], "preloaded")
        Self.assertIn("the hand-guiding calibration stopped, holding where it is", Self.Controller.HandGuideLast["reason"])
        Self.assertEqual(set(Self.Torque().values()), {1})

    def TestTheCalibrationIsRefusedWhereItCouldHitSomething(Self):
        Self.Calibrate(Start=False)
        with patch.object(Bridge.Collision.CollisionWorld, "EdgeReason", lambda *A, **K: "gripper could contact the basket rim"):
            Ok, Text = Self.Controller.handle_command({"type": "hand_guide_calibrate", "explicit": True}, "local", Owner)
        Self.assertFalse(Ok)
        Self.assertIn("could contact the basket rim. Move the arm to a clear, open pose first", Text)
        Self.Controller.servos["gripper"].GraspHold = True
        Ok, Text = Self.Controller.handle_command({"type": "hand_guide_calibrate", "explicit": True}, "local", Owner)
        Self.assertIn("the gripper holds an object", Text)
        Self.assertIsNone(Self.Controller.HandGuide)

    def TestThePageStartsItOverItsSocketAndItsHeartbeatKeepsItGoing(Self):
        from TestSafety import LocalClient
        Self.Clock[0] = None  # the real clock: the bridge's /ws loop and the guide loop run on their own threads
        Self.Hold()
        Controller = Self.Controller
        Controller.HandGuideThreaded = True

        def Frames(Seconds, Socket=None):
            End = time.monotonic() + Seconds
            while time.monotonic() < End:
                if Socket is not None:
                    Socket.send_json({"type": "hand_guide_heartbeat"})
                Self.Device.Step(1 / 30)
                Controller.refresh()
                time.sleep(1 / 30)
        with patch.object(Bridge, "controller", Controller), LocalClient().websocket_connect("/ws") as Socket:
            Self.assertEqual(Socket.receive_json()["type"], "state")
            Socket.send_json({"type": "hand_guide", "enable": True, "explicit": True})
            Reply = Socket.receive_json()
            while Reply.get("type") != "status":
                Reply = Socket.receive_json()
            Self.assertTrue(Reply["ok"], Reply)
            Session = Controller.HandGuide
            Self.assertIsNotNone(Session)
            Self.assertTrue(Session.Owner, "the session belongs to the page's connection")
            Frames(3.5, Socket)
            Self.assertIs(Controller.HandGuide, Session, Controller.status)
            Self.assertEqual(Session.Phase, "guide")
            Self.assertLess(time.perf_counter() - Controller.HandGuideOwners[Session.Owner], 0.5)
        # the page closed: 3 s later the arm holds where it is
        Frames(2.0)
        Self.assertIs(Controller.HandGuide, Session)
        Frames(1.6)
        Self.AssertHolding("the page that started it (this PC) closed or went to sleep")

    def TestTheGuideClockResolvesMilliseconds(Self):
        # time.monotonic ticks every 15.6 ms on Windows: the law's 40 ms speed estimate at 100 Hz needs better.
        Clock = Bridge.BridgeController().HandGuideNow
        Seen, Started = set(), time.perf_counter()
        while time.perf_counter() - Started < 0.05:
            Seen.add(Clock())
        Self.assertGreater(len(Seen), 100)

    def TestTheServoDumpReadsEveryRegisterAndWritesNothing(Self):
        Self.Hold()
        Controller = Self.Controller
        Self.Device.SetLoad(Ids["elbow_flex"], -200)
        Since = len(Self.Device.Received)
        Dump = Controller.ServoDump()
        Self.assertTrue(Dump["ok"], Dump)
        Self.assertEqual(Self.Device.Writes(Since=Since), [], "a dump writes nothing")
        Elbow = Dump["servos"]["elbow_flex"]["registers"]
        Self.assertEqual(Elbow["Present_Load"], -200, "Present_Load with its sign in bit 10")
        Self.assertEqual((Elbow["P"], Elbow["D"], Elbow["Punch"], Elbow["Firmware"]), (32, 32, 16, "3.10"))
        Self.assertEqual(Elbow["Present_Position"], Positions[Ids["elbow_flex"]])
        Self.assertEqual(Elbow["Present_Voltage_V"], 9.0)
        Self.assertEqual(len(Dump["servos"]["gripper"]["raw"]), 71)
        Self.Started()
        Self.assertIn("press Hold here first", Controller.ServoDump()["error"])

    # -- the 2026-10-10 safety review -----------------------------------------------------------------------------------
    def TestABlockedCalibrationStopsWhereItIsAndPushesNoFurther(Self):
        """H1: a blocked sweep stopped "holding" at its sweep's goal: the wrist blocked at 542 kept a goal of 622 at
        Torque_Limit 1000 (about 1.4 N m) for as long as it held. Now no sweep writes more than the joint needs (x 2 + 200,
        under 750), and the stop holds every joint where it is, the wrist with only its gravity's few steps."""
        Ok, Text = Self.Calibrate()
        Self.assertTrue(Ok, Text)
        Since = len(Self.Device.Received)
        Self.Run(3.0)
        Wrist = Ids["wrist_flex"]
        Blocked = Self.Device.Position(Wrist)
        Self.Device.OnStep.Blocked[Wrist] = Blocked  # a hand (or an object) stops the wrist
        Self.Run(3.0)
        Controller = Self.Controller
        Self.assertIsNone(Controller.HandGuide)
        Self.assertIn("wrist flex is", Controller.HandGuideCalibrationResult["error"])
        Self.assertIn("off its calibration sweep", Controller.HandGuideCalibrationResult["error"])
        Limits = Self.LoopLimits(Since)
        Self.assertTrue(Limits and max(L for _, L in Limits) <= 750, "never 1000 while sweeping")
        Self.assertLessEqual(max(L for N, L in Limits if N == "wrist_flex"), 500)
        Goal = Signed(Self.Register("wrist_flex", 42, 2))
        Self.assertLessEqual(abs(Goal - Blocked), 10, f"held where it stopped (goal {Goal}, at {Blocked}), not 80 steps on")
        Self.Frames(3.0)
        Self.assertEqual(Signed(Self.Register("wrist_flex", 42, 2)), Goal, "and it stays there")
        Self.assertIsNone(Controller.MotionFault, Controller.status)
        Self.assertEqual(set(Self.Torque().values()), {1})

    def TestTheGrippersCalibrationSqueezesLightlyAndABlockedSpinStopsAtOnce(Self):
        """H1: the gripper's sweep closed at Torque_Limit 1000 (about 28 N at the jaw) and its 90 deg/s spin had no
        tracking check. Now it opens and closes at 250 at most, and a blocked spin stops it within 0.2 s, holding."""
        Options = {"Joints": ("gripper",), "Spin": ("gripper",), "Speeds": (20.0,), "RunNames": ("A",),
                   "HalfSpan": {"gripper": 15.0}}
        Gripper = Ids["gripper"]
        Self.Controller.HandGuidePlanOptions = lambda: Options
        Ok, Text = Self.Calibrate()
        Self.assertTrue(Ok, Text)
        Since = len(Self.Device.Received)
        Self.Run(0.3)
        Blocked = Self.Device.Position(Gripper)
        Self.Device.OnStep.Blocked[Gripper] = Blocked  # a finger between the jaws
        Self.Run(1.0)
        Self.assertIsNone(Self.Controller.HandGuide)
        Self.assertIn("gripper is", Self.Controller.HandGuideCalibrationResult["error"])
        Self.assertLessEqual(max(L for N, L in Self.LoopLimits(Since) if N == "gripper"), 250)
        Self.assertEqual(Signed(Self.Register("gripper", 42, 2)), Blocked, "it lets go: held where it stopped")
        # its spin, blocked as it starts
        Self.setUp()
        Self.Controller.HandGuidePlanOptions = lambda: Options
        Self.assertTrue(Self.Calibrate()[0])
        Run = Self.Controller.HandGuide.Law
        Spun, Stopped = [], []

        def Each(Step):
            Segment = Run.Segments[Run.Index] if 0 <= Run.Index < len(Run.Segments) else None
            if not Spun and Segment is not None and Segment.Kind == "spin":
                Spun.append(Self.Clock[0])
                Self.Device.OnStep.Blocked[Gripper] = Self.Device.Position(Gripper)
            if Spun and not Stopped and Self.Controller.HandGuide is None:
                Stopped.append(Self.Clock[0])
        Self.Run(8.0, Each)
        Self.assertTrue(Spun and Stopped)
        Self.assertLess(Stopped[0] - Spun[0], 0.2)
        Self.assertIn("off its calibration sweep", Self.Controller.HandGuideCalibrationResult["error"])
        Self.assertEqual(Signed(Self.Register("gripper", 42, 2)), Self.Device.Position(Gripper))

    def TestOneCorruptedReplyDoesNotEndGuidingButTwoInARowDo(Self):
        """H2: every check but heat acted on the first bad reading and dropped the arm. Now a fault must be read twice
        in a row (the status registers are read again on the next cycle: at most 10 ms later), and a position that
        jumps is a missed reply (the joint keeps its last goal)."""
        Elbow, Lift = Ids["elbow_flex"], Ids["shoulder_lift"]
        Cases = [("torque read off", 40, {Elbow: {40: b"\x00"}}, "elbow_flex torque unexpectedly off"),
                 ("Lock read 0", 40, {Ids["wrist_roll"]: {55: b"\x00"}}, "wrist_roll restarted"),
                 ("0 V", 56, {Lift: {62: b"\x00"}}, "below 7.0 V"),
                 ("overload bit", 56, {Ids["gripper"]: {65: bytes([32])}}, "gripper firmware protection bit 32"),
                 ("the 2026-10-10 reply", 56, {Elbow: {60: (0xF501).to_bytes(2, "little"), 62: b"\x00", 63: bytes([90]),
                                                       65: bytes([32])}}, "elbow_flex firmware protection")]
        for Name, Address, Changes, Words in Cases:
            with Self.subTest(Name):
                Self.setUp()
                Self.Hold()
                Self.Started()
                Corrupt(Self.Device, Address, Changes)
                Self.Run(0.3)
                Self.assertIsNotNone(Self.Controller.HandGuide, f"one {Name}: still guiding")
                Self.assertIsNone(Self.Controller.MotionFault)
                Corrupt(Self.Device, Address, Changes, Times=2)
                Self.Run(0.1)
                Self.AssertLimp(Words)
                Self.doCleanups()
        # a position glitch of +300 steps on the lift: its goal went 351 steps out at its 650 cap for a cycle
        Self.setUp()
        Self.Hold()
        Self.Started()
        Out = []
        Each = lambda Step: Out.append(abs(Signed(Self.Register("shoulder_lift", 42, 2)) - Self.Device.Position(Lift)))
        Self.Run(0.3, Each)
        Usual = max(Out)
        Corrupt(Self.Device, 56, {Lift: {56: (Self.Device.Position(Lift) + 300).to_bytes(2, "little")}})
        Self.Run(0.5, Each)
        Self.assertIsNotNone(Self.Controller.HandGuide)
        Self.assertLessEqual(max(Out), Usual + 10, (Usual, max(Out)))
        Corrupt(Self.Device, 56, {Lift: {56: (Self.Device.Position(Lift) + 300).to_bytes(2, "little")}}, Times=2)
        Self.Run(0.1)
        Self.AssertLimp("shoulder_lift position jumped")

    def TestOneCorruptedReplyDoesNotEndTheCalibrationButTwoInARowDo(Self):
        Elbow = Ids["elbow_flex"]
        for Name, Address, Changes, Words in (("torque read off", 40, {Elbow: {40: b"\x00"}}, "torque went off"),
                                              ("0 V", 56, {Elbow: {62: b"\x00"}}, "supply fell to 0.0 V"),
                                              ("overload bit", 56, {Elbow: {65: bytes([32])}}, "firmware protection (32)")):
            with Self.subTest(Name):
                Self.setUp()
                Self.assertTrue(Self.Calibrate()[0])
                Self.Run(1.0)
                Corrupt(Self.Device, Address, Changes)
                Self.Run(0.3)
                Self.assertIsNotNone(Self.Controller.HandGuide, f"one {Name}: still calibrating")
                Corrupt(Self.Device, Address, Changes, Times=2)
                Self.Run(0.1)
                Self.AssertLimp(Words)
                Self.doCleanups()

    def TestTheRollAndTheWristTurnThroughRawZeroInEitherFrame(Self):
        """H3: a servo reporting wrapping positions went limp at raw 0 ("frame jumped -4095 steps"). The wrist flexes
        (6 deg/s) and the roll turns (12 deg/s) through raw 0 by hand: guiding goes on, and every goal written is an
        offset from the position that servo reported, multi-turn or wrapping."""
        Wrist, Roll = Ids["wrist_flex"], Ids["wrist_roll"]
        Start = {Wrist: 40, Roll: 4040}
        Rate = {Wrist: -6 * 4096 / 360, Roll: 12 * 4096 / 360}
        for Wrap in (False, True):
            with Self.subTest("wrapping" if Wrap else "multi-turn"):
                Self.setUp()
                for Id, Raw in Start.items():
                    Self.Physics.Held[Id] = Raw
                    Self.Device.SetPosition(Id, Raw)
                Self.Hold()
                Self.Started()
                Seen, Read = [], {}

                def Turn(Step):
                    # the goal each servo got last cycle, against the position that cycle read (as reported)
                    for Id in Start:
                        if Self.Device.Tables[Id][40] and Id in Read:
                            Seen.append((Id, Signed(Self.Device.Register(Id, 42, 2)), Read[Id]))
                        Read[Id] = Signed(Self.Device.Position(Id))  # what this cycle reads
                    for Id in Start:
                        Raw = Start[Id] + round((Step + 1) * 0.01 * Rate[Id])
                        Self.Physics.Held[Id] = Raw % 4096 if Wrap else Raw
                Self.Run(1.6, Turn)
                Self.assertIsNotNone(Self.Controller.HandGuide, Self.Controller.HandGuideLast)
                Ends = Signed(Self.Device.Position(Wrist)), Signed(Self.Device.Position(Roll))
                Self.assertTrue(Ends[0] > 3900 and Ends[1] < 300 if Wrap else Ends[0] < 0 and Ends[1] > 4096, Ends)
                Self.assertGreaterEqual(Self.Controller.HandGuide.Law.assist_counts["wrist_roll"]["on"], 1)
                Self.assertGreater(len([S for S in Seen if S[0] == Roll]), 20, "the roll was energised")
                for Id, Goal, At in Seen:
                    Self.assertLess(abs(Goal - At), 80, (Id, Goal, At))
                Self.doCleanups()

    def TestGoingLimpReleasesEveryJointBeforeAnythingElse(Self):
        """M2: going limp wrote Goal_Speed to all six first, which energised the floating joints toward their stale goals
        (the pan 150 steps away at Torque_Limit 1000, no acceleration profile) for 10-25 ms before the torque-off. Now
        one Torque_Enable 0 to all six goes first, and nothing after it writes a goal or a speed."""
        Pan = Ids["shoulder_pan"]
        for Name, Act in (("a fault", lambda C: C.EndHandGuide("coast", "test: a fault")),
                          ("Coast", lambda C: C.toggle_torque(False)),
                          ("emergency stop", lambda C: C.emergency_stop())):
            with Self.subTest(Name):
                Self.setUp()
                Self.Hold()
                Self.Started()
                Self.Device.Tables[Pan][42:44] = (Self.Device.Position(Pan) - 150).to_bytes(2, "little")  # a stale goal
                Mark = len(Self.Device.Received)
                with Self.Controller.lock:
                    Act(Self.Controller)
                Writes = [(Id, Address) for Instruction, Id, Address, _ in Self.Device.Received[Mark:] if Instruction == 3]
                Self.assertEqual(Writes[:6], [(Ids[N], 40) for N in Names], "first: Torque_Enable 0 to all six at once")
                Self.assertEqual([W for W in Writes if W[1] in (41, 42, 44, 46, 48)], [], "then no goal, speed or limit write")
                Self.assertEqual(set(Self.Torque().values()), {0})
                Self.assertEqual(Self.Controller.HandGuideLast["kind"], "coast")
                Self.doCleanups()

    def TestAGripperKeptSqueezingIsWatchedForFaults(Self):
        """L1: a gripper left squeezing an object ("keep") was not read for faults while guiding."""
        Self.Hold()
        Self.Controller.servos["gripper"].GraspHold = True
        Self.assertTrue(Self.Guide()[0])
        Self.Run(2.6)
        Self.assertEqual(Self.Controller.state_snapshot()["hand_guide"]["roles"]["gripper"], "keep")
        Self.Device.Tables[Ids["gripper"]][65] = 32  # its overload protection
        Self.Run(0.1)
        Self.AssertLimp("gripper firmware protection bit 32")

    def TestAStartInsideTheCollisionGuardBandIsRefused(Self):
        """L4: the walls were armed only once the guard band first read clear: a start inside it had none."""
        Self.Hold()
        with patch.object(Bridge.Collision.CollisionWorld, "IntervalReason", lambda *A, **K: "the cart body"):
            Ok, Text = Self.Guide()
        Self.assertFalse(Ok)
        Self.assertIn("the arm is within 3 deg of the basket, the cart body, the camera head or the rearward limit: move it "
                      "clear first", Text)
        Self.assertIsNone(Self.Controller.HandGuide)
        Self.assertEqual(Self.Register("shoulder_lift", 41), 51, "nothing written")

    def TestTheOwnersHeartbeatGoesWithItsSession(Self):
        """L6: HandGuideOwners kept the owner's entry for good."""
        Self.Hold()
        Self.Started()
        Self.assertIn(Owner, Self.Controller.HandGuideOwners)
        Self.assertTrue(Self.Controller.handle_command({"type": "hand_guide", "enable": False}, "local", Owner)[0])
        Self.assertNotIn(Owner, Self.Controller.HandGuideOwners)

    def TestStateAndHealthReportIt(Self):
        Self.assertEqual(Bridge.health()["hand_guide_version"], "guide-v1")
        Self.Hold()
        State = Self.Controller.state_snapshot()
        Self.assertEqual(State["hand_guide_version"], "guide-v1")
        Self.assertEqual({K: State["hand_guide"][K] for K in ("on", "calibrating", "calibrated")},
                         {"on": False, "calibrating": False, "calibrated": True})


def load_tests(Loader, Tests, Pattern):
    Prefixed = unittest.TestLoader()
    Prefixed.testMethodPrefix = "Test"
    return Prefixed.loadTestsFromTestCase(HandGuideBridgeTests)


if __name__ == "__main__":
    Loader = unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Result = unittest.TextTestRunner(verbosity=2).run(Loader.loadTestsFromTestCase(HandGuideBridgeTests))
    raise SystemExit(not Result.wasSuccessful())
