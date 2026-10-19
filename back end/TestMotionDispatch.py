"""One synchronized calculation per dispatch, with the same servo commands. FakeBus only."""
import time as Time
import unittest as Unittest
from unittest.mock import patch as Patch

import bridge_server as Bridge
import MeasureRouteSpeed as Measure
from test_bridge_home import make_controller as MakeController, displace as Displace


def setUpModule():
    for Guard in (Patch.object(Bridge.depth_camera, "Opener", side_effect=RuntimeError("offline: no camera")),
                  Patch.object(Bridge, "RaiseProcessPriority", return_value="normal"),
                  Patch.object(Bridge, "ROUTE_REACH_ASSIGN_SECONDS", 60.0)):
        Guard.start()
        Unittest.addModuleCleanup(Guard.stop)


class MotionDispatchTests(Unittest.TestCase):
    def TestEachDispatchComputesOnceAndUsesFreshThrottleAndFeedback(Self):
        Controller, Bus = MakeController()
        Displace(Controller, Bus)
        Controller.SetMotionSpeed(100)
        Self.assertTrue(Controller.go_home()[0])
        with Patch.object(Controller, "RouteSpeeds", wraps=Controller.RouteSpeeds) as Calculate:
            with Controller.lock:
                Controller._write_active_slider_commands_locked()
            Self.assertEqual(Calculate.call_count, 1)
            First = {Name: Joint.command_speed for Name, Joint in Controller.servos.items()}
            Controller.SetMotionSpeed(10)
            with Controller.lock:
                Controller._write_active_slider_commands_locked()
            Self.assertEqual(Calculate.call_count, 2)
            Slower = {Name: Joint.command_speed for Name, Joint in Controller.servos.items()}
            Self.assertLess(max(map(abs, Slower.values())), max(map(abs, First.values())))
            for Name, Joint in Controller.servos.items():
                Bus.position[Name] = Joint.target_raw
            Controller.refresh()
            Self.assertEqual(Calculate.call_count, 3)
            Self.assertTrue(all(Joint.command_speed == 0 for Joint in Controller.servos.values()))
            Self.assertIsNone(Controller.MotionRoute)

    def TestHeldArmDoesNotCalculateRouteCommands(Self):
        Controller, _ = MakeController()
        with Patch.object(Controller, "RouteSpeeds", side_effect=AssertionError("idle route computation")):
            Controller.refresh()
        Self.assertIsNone(Controller.MotionFault)

    def TestPerFrameWritesMatchRepeatedCalculationThroughStagesAndSpeedChanges(Self):
        # The old dispatch calculated all six joint speeds again for every servo write.
        # Reproduce that independently, then compare every bus write and feedback frame,
        # including waypoint switches, late/sticky servos and a live throttle change.
        for Case, ModelName in ((0, "nominal"), (2, "late+fast"), (4, "sticky"), (6, "late")):
            with Self.subTest(Case=Case, Model=ModelName):
                _, Start, Request = Measure.Cases[Case]
                Fast, FastBus = Measure.MakeController(Start)
                Reference, ReferenceBus = Measure.MakeController(Start)
                OriginalSend = Reference._send_slider_velocity_locked

                def RepeatCalculation(Servo, **Options):
                    Options.pop("route_speeds", None)
                    return OriginalSend(Servo, **Options)

                Reference._send_slider_velocity_locked = RepeatCalculation
                for Controller in (Fast, Reference):
                    Controller.SetMotionSpeed(100)
                    Ok, Message = Measure.Send(Controller, Request)
                    Self.assertTrue(Ok, Message)
                Self.assertEqual(Fast.MotionRoute["Path"], Reference.MotionRoute["Path"])
                Self.assertEqual(Fast.MotionRoute["Reach"], Reference.MotionRoute["Reach"])
                Self.assertEqual(FastBus.writes, ReferenceBus.writes)
                Make = Measure.NominalModel if ModelName == "nominal" else Measure.HarshModels[ModelName]
                Models = (Make(FastBus), Make(ReferenceBus))
                Finished = False
                for Frame in range(1800):
                    if Frame in (17, 45):
                        for Controller in (Fast, Reference):
                            Controller.SetMotionSpeed(18 if Frame == 17 else 100)
                    for Controller, Bus, Model in zip((Fast, Reference), (FastBus, ReferenceBus), Models):
                        Model.Step()
                        Bus.writes.clear()
                        Controller.last_refresh_s = Time.monotonic()
                        Controller.refresh()
                    Self.assertEqual(FastBus.writes, ReferenceBus.writes, f"frame {Frame}")
                    Self.assertEqual(FastBus.position, ReferenceBus.position, f"frame {Frame}")
                    Self.assertIsNone(Fast.MotionFault)
                    Self.assertIsNone(Reference.MotionFault)
                    Self.assertEqual(Fast.MotionRoute is None, Reference.MotionRoute is None)
                    if Fast.MotionRoute is None:
                        Finished = True
                        break
                Self.assertTrue(Finished, "both models must finish")


def load_tests(Loader, Tests, Pattern):
    Prefixed = Unittest.TestLoader()
    Prefixed.testMethodPrefix = "Test"
    return Prefixed.loadTestsFromTestCase(MotionDispatchTests)


if __name__ == "__main__":
    Loader = Unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Result = Unittest.TextTestRunner(verbosity=2).run(Loader.loadTestsFromTestCase(MotionDispatchTests))
    raise SystemExit(not Result.wasSuccessful())
