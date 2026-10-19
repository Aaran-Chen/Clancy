"""/locate's optional task_object: validated, then handed to object_locator. The locator is mocked.

The app names the task's object ({"phrase": "black thing on the desk", "label": "black thing"})
so grounding can confirm it; see object_locator.locate_objects(task_object=...).

Run:  python TestLocateTaskObject.py
"""
import unittest
from unittest.mock import MagicMock, patch

import bridge_server as Bridge
import calibration
import object_locator

Image = "data:image/jpeg;base64,AAAA"


class Camera:
    """No depth camera: nothing is opened."""
    def Intrinsics(Self):
        return None

    def Depth(Self, Seq=None):
        return None


class LocateTaskObjectTests(unittest.TestCase):
    def Locate(Self, Payload):
        Locator = MagicMock(return_value={"ok": True, "objects": []})
        with patch.object(calibration, "camera_model", return_value=None), \
                patch.object(object_locator, "locate_objects", Locator), patch.object(Bridge, "depth_camera", Camera()):
            return Bridge.locate(Payload), Locator

    def TestTaskObjectIsHandedToTheLocator(Self):
        Result, Locator = Self.Locate({"image": Image, "task_object": {"phrase": "black thing on the desk", "label": "black thing"}})
        Self.assertTrue(Result["ok"], Result)
        Self.assertEqual(Locator.call_count, 1)
        Self.assertEqual(Locator.call_args.kwargs["task_object"], {"phrase": "black thing on the desk", "label": "black thing"})
        # Only the two named fields travel on.
        Result, Locator = Self.Locate({"image": Image, "task_object": {"phrase": "eraser on the desk", "label": "eraser", "x": 1}})
        Self.assertEqual(Locator.call_args.kwargs["task_object"], {"phrase": "eraser on the desk", "label": "eraser"})
        # The longest phrase allowed.
        Result, Locator = Self.Locate({"image": Image, "task_object": {"phrase": "a" * 80, "label": "a"}})
        Self.assertEqual(Locator.call_args.kwargs["task_object"]["phrase"], "a" * 80)

    def TestTaskObjectIsOptional(Self):
        for Payload in ({"image": Image}, {"image": Image, "task_object": None}):
            with Self.subTest(Payload=Payload):
                Result, Locator = Self.Locate(Payload)
                Self.assertTrue(Result["ok"], Result)
                Self.assertEqual(Locator.call_count, 1)
                Self.assertIsNone(Locator.call_args.kwargs.get("task_object"))

    def TestMalformedTaskObjectIsRefusedBeforeAnyModelCall(Self):
        for TaskObject in ("black thing", ["black thing"], {"phrase": "black thing"}, {"label": "black thing"},
                           {"phrase": 5, "label": "black thing"}, {"phrase": "black thing", "label": 5},
                           {"phrase": "", "label": "black thing"}, {"phrase": "   ", "label": "black thing"},
                           {"phrase": "black thing", "label": ""}, {"phrase": "a" * 81, "label": "a"}):
            with Self.subTest(TaskObject=TaskObject):
                Result, Locator = Self.Locate({"image": Image, "task_object": TaskObject})
                Self.assertFalse(Result["ok"])
                Self.assertIn("task_object", Result["error"])
                Locator.assert_not_called()


def load_tests(Loader, Tests, Pattern):
    """`python -m unittest TestLocateTaskObject` runs the Test* methods too (unittest names this hook)."""
    Prefixed = unittest.TestLoader()
    Prefixed.testMethodPrefix = "Test"
    return Prefixed.loadTestsFromTestCase(LocateTaskObjectTests)


if __name__ == "__main__":
    Loader = unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Result = unittest.TextTestRunner(verbosity=2).run(Loader.loadTestsFromTestCase(LocateTaskObjectTests))
    raise SystemExit(not Result.wasSuccessful())
