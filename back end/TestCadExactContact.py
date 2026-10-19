"""CAD-exact basket, cart and body contact (CollisionModel v8), offline on simulated motors.

2026-10-04 08:32 (back end/captures/2026-10-04/recording/ai.jsonl), "grab the pill bottle": the arm
descended to the grasp point beside the bottle (X 11.0, Y 32.4, Z 2.4) clear of the cart, and the
bridge refused the close there, chained with the lift and then alone: "Target arm pose: gripper link
could contact the basket. No motion sent." The gripper link's CAD box stood 2.4 cm from the cart;
only its envelope over the 16-step tracking allowance touched it. Approved by Aaran the same day: a
pose is refused only where the CAD geometry meets the basket or body. Never opens a serial port or
the depth camera.
"""
import unittest as Unittest
from unittest.mock import patch as Patch

import numpy as Np
import BasketCollision as Collision
import bridge_server as Bridge
from BasketSurface import Surface
from ArmMotionEnvelope import Signs
from TestBasketSurface import CubeSource
from TestFlyByRoutes import Execute, V7ContactAllowance
from TestSafety import MakeController

# The model requests' "Current servo positions" at 08:32:36 and 08:32:51: the arm at the grasp point, jaws 50% open.
AtGraspRaw = {"shoulder_pan": 3045, "shoulder_lift": 891, "elbow_flex": 1448, "wrist_flex": 374, "wrist_roll": 1423, "gripper": 1438}
ClosedRaw = 642
HardwareCalls = []


def setUpModule():
    def GuardedOpener():
        HardwareCalls.append("depth camera opened")
        raise RuntimeError("an offline test reached the real depth camera")
    for Item in (Patch.object(Bridge.depth_camera, "Opener", GuardedOpener),
                 Patch.object(Bridge, "RaiseProcessPriority", lambda: HardwareCalls.append("priority raised") or "normal")):
        Item.start()
        Unittest.addModuleCleanup(Item.stop)


def PlaceRaw(Controller, Bus, Raw):
    for Name, Value in Raw.items():
        Servo = Controller.servos[Name]
        Servo.present_raw = Servo.target_raw = Servo.HardwareRaw = Bus.position[Name] = Value
        Servo.command_raw = None
    Bus.configure_motors()
    Controller.refresh()
    Bus.writes.clear()


def Angles(Controller):
    return Np.array([Controller._ik_angle_from_raw(Controller.servos[Name], Controller.servos[Name].present_raw) for Name in Bridge.MOTOR_NAMES])


class CadExactContactTests(Unittest.TestCase):
    def TestTheModelIsCadExact(Self):
        Self.assertEqual(Collision.Model["Version"], "robot-body-cad-exact-v8")
        Self.assertEqual(Collision.PlannerVersion, "flyby-cad-exact-v8")
        Self.assertEqual((Collision.Model["ClearanceCm"], Collision.Model["MountClearanceCm"], Collision.Model["ContactTrackingAllowanceRaw"]), (0, 0, 0))
        # The arm's own tracking allowance is unchanged: the route monitor, braking and the rearward limit keep it.
        Self.assertEqual(Collision.Model["JointTrackingAllowanceRaw"], 16)
        Controller, _ = MakeController()
        World = Controller.CollisionWorld()
        Self.assertEqual((World.ContactSlack, World.Slack), (0.0, 16 * 360 / 4096))

    def TestTheCloseAtThe0832GraspPoseIsAccepted(Self):
        Controller, Bus = MakeController()
        PlaceRaw(Controller, Bus, AtGraspRaw)
        World = Controller.CollisionWorld()
        Pose = dict(zip(Collision.Names, Angles(Controller)))
        Self.assertIsNone(World.PoseReason(Pose))
        # The v7 contact model refused the pose, for the gripper link's tracking envelope alone; and a jaw-only
        # move is judged on the moving jaw alone (ToolReason), so even with that model the close now passes.
        with V7ContactAllowance():
            Old = Controller.CollisionWorld()
            Self.assertEqual(Old.PoseReason(Pose), "gripper link could contact the basket")
            Self.assertIsNone(Old.PoseReason(Pose, Tracking=False))
            Start = Angles(Controller)
            Self.assertIsNone(Old.ToolReason(Start, Np.r_[Start[:-1], Controller._ik_angle_from_raw(Controller.servos["gripper"], ClosedRaw)]))
        Ok, Message = Controller.move_servo("gripper", ClosedRaw)
        Self.assertTrue(Ok, Message)
        # One stage, the arm held where it is.
        Self.assertEqual(Controller.MotionRoute["Path"], [{**AtGraspRaw, "gripper": ClosedRaw}])
        Execute(Self, Controller, Bus)
        Self.assertLessEqual(Controller._circular_raw_distance(Controller.servos["gripper"].present_raw, ClosedRaw), Bridge.GRIPPER_GOAL_TOLERANCE_RAW)
        for Name in Bridge.MOTOR_NAMES[:-1]:
            Self.assertLessEqual(Controller._circular_raw_distance(Controller.servos[Name].present_raw, AtGraspRaw[Name]), 2, Name)
        Self.assertFalse(HardwareCalls)

    def TestTheLiftFromThe0832GraspPoseIsAccepted(Self):
        # The chained move after the close: the tip 6 cm up, wrist -15.9 deg (the approved 08:32:41 proposal).
        Controller, Bus = MakeController()
        PlaceRaw(Controller, Bus, {**AtGraspRaw, "gripper": ClosedRaw})
        Display = {"x": 11.021682859686557, "y": 32.36788169456106, "z": 8.42672279933027}
        Base = Bridge._load_json(Bridge.BACKEND_ROOT / "WorkspaceFrame.json")["RobotFrame"]["ArmBasePositionCm"]
        Target = {"x": -Display["x"] - Base["x"], "y": Display["y"] - Base["y"], "z": Display["z"] - Base["z"], "wrist_pitch": -15.90778113706257}
        Ok, Message = Controller.send_ik(Target=Target)
        Self.assertTrue(Ok, Message)
        Execute(Self, Controller, Bus)

    def TestAJawMoveIsJudgedOnTheMovingJawAlone(Self):
        Controller, Bus = MakeController()
        PlaceRaw(Controller, Bus, AtGraspRaw)
        World = Controller.CollisionWorld()
        Start = Angles(Controller)
        Goal = Start.copy()
        Goal[-1] = Controller._ik_angle_from_raw(Controller.servos["gripper"], ClosedRaw)
        Self.assertIsNone(World.ToolReason(Start, Goal))
        # A tiny obstacle where the moving jaw's corner passes half way through the close is met; the same
        # obstacle beside the fixed gripper link (which does not move) does not stop a jaw move.
        Jaw = next(Box for Box in World.Boxes if Box[1] == "moving_jaw_so101_v1_link")
        Middle = Start.copy()
        Middle[-1] = (Start[-1] + Goal[-1]) / 2
        Links, _ = Collision.Cad.Forward(Controller.arm_geometry, dict(zip(Collision.Names, Middle)))
        _, Link, Center, Half, _ = Jaw
        Corner = Links[Link][:3, :3] @ (Center + Signs[0] * Half) + Links[Link][:3, 3]
        Source = CubeSource()
        Source["Vertices"] = (Np.array(Source["Vertices"]) * .001 + Corner).tolist()
        Source["Nodes"][0]["Bounds"] = [(Corner - .001).tolist(), (Corner + .001).tolist()]
        World.Surface = Surface([0, 0, 0], Source)
        Self.assertEqual(World.ToolReason(Start, Goal), "moving jaw so101 v1 link could contact the basket")
        Self.assertIsNone(World.ToolReason(Start, Start + Np.eye(6)[-1] * 0.01))
        Fixed = next(Box for Box in World.Boxes if Box[1] == "gripper_link")
        _, Link, Center, Half, _ = Fixed
        Corner = Links[Link][:3, :3] @ (Center + Signs[0] * Half) + Links[Link][:3, 3]
        Source["Vertices"] = (Np.array(CubeSource()["Vertices"]) * .001 + Corner).tolist()
        Source["Nodes"][0]["Bounds"] = [(Corner - .001).tolist(), (Corner + .001).tolist()]
        World.Surface = Surface([0, 0, 0], Source)
        Self.assertIsNone(World.ToolReason(Start, Goal))
        Self.assertIsNotNone(World.PoseReason(dict(zip(Collision.Names, Start))))

    def TestAPoseWhoseCadBoxMeetsTheBasketIsStillRefused(Self):
        # From the 08:32 grasp pose, one arm joint turned until a link's CAD box meets the cart: refused.
        Controller, Bus = MakeController()
        PlaceRaw(Controller, Bus, AtGraspRaw)
        World = Controller.CollisionWorld()
        Start = Angles(Controller)
        Into = None
        for Joint, Sign in ((Collision.Names.index(Name), Sign) for Name in ("wrist_flex", "elbow_flex", "shoulder_lift") for Sign in (1, -1)):
            for Step in range(1, 121):
                Pose = Start.copy()
                Pose[Joint] += Sign * Step * 0.5
                Reason = World.PoseReason(dict(zip(Collision.Names, Pose)))
                if Reason:
                    Into = Pose if "basket" in Reason else None
                    break
            if Into is not None:
                break
        Self.assertIsNotNone(Into, "no single joint turn met the cart")
        Self.assertIn("basket", World.PoseReason(dict(zip(Collision.Names, Into)), Tracking=False))
        Raw = {Name: Controller._ik_raw_from_angle(Controller.servos[Name], Value) for Name, Value in zip(Bridge.MOTOR_NAMES, Into)}
        Ok, Message = Controller.SendJointMotion({Name: Raw[Name] for Name in Bridge.MOTOR_NAMES[:4]}, "Test")
        Self.assertFalse(Ok)
        Self.assertIn("could contact the basket", Message)
        Self.assertIsNone(Controller.MotionRoute)


def load_tests(Loader, Tests, Pattern):
    """`python -m unittest TestCadExactContact` runs the Test* methods too (unittest names this hook)."""
    Prefixed = Unittest.TestLoader()
    Prefixed.testMethodPrefix = "Test"
    return Prefixed.loadTestsFromTestCase(CadExactContactTests)


if __name__ == "__main__":
    Loader = Unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Result = Unittest.TextTestRunner(verbosity=2).run(Loader.loadTestsFromTestCase(CadExactContactTests))
    raise SystemExit(not Result.wasSuccessful())
