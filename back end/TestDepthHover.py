"""GET /depthcam/depth-at: the live view's hover depth readout (no hardware).

Usage: python TestDepthHover.py
"""
import math
import unittest
from unittest.mock import patch

import numpy as np

import TestDepthCamera as TDC
from TestDepthCamera import Bridge, DG, OC, calibration, FrontCamera, Height, Intrinsics, RenderDepth, TableZ, Width


class DepthHoverTests(unittest.TestCase):
    def setUp(Self):
        Self.Camera = OC.DepthCamera(Opener=TDC.FakeSource)
        Self.Camera.IntrinsicsValue = dict(Intrinsics)
        Self.Patch = patch.object(Bridge, "depth_camera", Self.Camera)
        Self.Patch.start()

    def tearDown(Self):
        Self.Patch.stop()

    def TestReadsDepthAndPositionUnderTheCursor(Self):
        Seq = Self.Camera.Push(np.zeros((Height, Width, 3), np.uint8), RenderDepth(FrontCamera, []))
        Point = np.array([-15.0, 28.0, TableZ])
        Camera = DG.DisplayToCamera(Point, FrontCamera)[0]
        U, V = DG.Project(DG.DisplayToCamera(Point, FrontCamera), Intrinsics)[0]
        with patch.object(calibration, "camera_model", return_value=FrontCamera):
            Result = Bridge.depthcam_depth_at(u=U + 0.5, v=V + 0.5)  # live view pixels are unmirrored
        Self.assertTrue(Result["ok"], Result)
        Self.assertEqual(Result["seq"], Seq)
        Self.assertAlmostEqual(Result["depth_cm"], Camera[2], delta=0.3)
        Self.assertLess(math.dist([Result["display_cm"][Axis] for Axis in "xyz"], Point), 0.3)
        with patch.object(calibration, "camera_model", return_value=None):
            Self.assertNotIn("display_cm", Bridge.depthcam_depth_at(u=U + 0.5, v=V + 0.5))

    def TestNeverPinsAFrame(Self):
        # A pinned AI capture must survive any amount of hovering.
        First = Self.Camera.Push(np.zeros((Height, Width, 3), np.uint8), RenderDepth(FrontCamera, []))
        Self.Camera.Depth(First)
        for _ in range(3 * OC.PinnedFrames):
            Self.Camera.Push(np.zeros((Height, Width, 3), np.uint8), RenderDepth(FrontCamera, []))
            Self.assertTrue(Bridge.depthcam_depth_at(u=Width / 2, v=Height / 2)["ok"])
        Self.assertEqual(list(Self.Camera.Pinned), [First])

    def TestReportsMissingDepth(Self):
        Self.assertFalse(Bridge.depthcam_depth_at(u=10, v=10)["ok"])  # nothing streamed yet
        Self.Camera.Push(np.zeros((Height, Width, 3), np.uint8), np.zeros((Height, Width), np.uint16))
        Self.assertEqual(Bridge.depthcam_depth_at(u=Width / 2, v=Height / 2)["error"], "no depth at that pixel")
        Self.assertFalse(Bridge.depthcam_depth_at(u=Width + 50, v=10)["ok"])
        Self.Camera.IntrinsicsValue = None
        Self.assertFalse(Bridge.depthcam_depth_at(u=Width / 2, v=Height / 2)["ok"])

    def TestRouteIsRegistered(Self):
        Routes = {(Route.path, Method) for Route in Bridge.app.routes for Method in getattr(Route, "methods", ())}
        Self.assertIn(("/depthcam/depth-at", "GET"), Routes)


def load_tests(Loader, Tests, Pattern):
    """`python -m unittest TestDepthHover` runs the Test* methods too (unittest names this hook)."""
    Prefixed = unittest.TestLoader()
    Prefixed.testMethodPrefix = "Test"
    return Prefixed.loadTestsFromTestCase(DepthHoverTests)


if __name__ == "__main__":
    Loader = unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Result = unittest.TextTestRunner(verbosity=2).run(Loader.loadTestsFromTestCase(DepthHoverTests))
    raise SystemExit(not Result.wasSuccessful())
