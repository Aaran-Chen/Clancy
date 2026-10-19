"""Offline RGB/depth and rigid-registration tests. No camera or serial access."""
import base64
import copy
import json
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

import CameraDrift
import RobotCameraReference as Robot


class RobotCameraReferenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(Cls):
        Root = Path(__file__).resolve().parent / "TestArtifacts" / "RobotCameraReference"
        Root.mkdir(parents=True, exist_ok=True)
        Cls.Temp = tempfile.TemporaryDirectory(dir=Root)
        Cls.Path = Path(Cls.Temp.name)
        Cls.Intrinsics = {"fx": 500.0, "fy": 500.0, "cx": 320.0, "cy": 240.0, "width": 640, "height": 480, "dist": [0.0] * 5}
        Rng = np.random.default_rng(410)
        Mono = Rng.integers(0, 256, (240, 320), dtype=np.uint8)
        Cls.Image = cv2.cvtColor(cv2.resize(Mono, (640, 480), interpolation=cv2.INTER_NEAREST), cv2.COLOR_GRAY2BGR)
        Cls.Depth = np.full((480, 640), 460, dtype=np.uint16)
        Cls.Arm = {"ok": True, "joints": [[0, -20, 20], [0, -20, 30], [0, -20, 40]]}
        Cls.Model = {"type": "pinhole_depth_v1", "rvec": [np.pi, 0, 0], "tvec": [0, 8, 45], "mirror_x": False,
                     "image_size": [640, 480], "table_z": -2.3, "fitted_at": "offline-fixture"}
        ImageData = "data:image/png;base64," + base64.b64encode(cv2.imencode(".png", Cls.Image)[1]).decode("ascii")
        cv2.imwrite(str(Cls.Path / "depth.png"), Cls.Depth)
        (Cls.Path / "observation.json").write_text(json.dumps({"images": [ImageData], "depth_frames_png": ["depth.png"],
            "depth_intrinsics": Cls.Intrinsics, "joints": Cls.Arm["joints"], "mirrored": False}), encoding="utf-8")
        Reference = {"method": "desk_plane", "keyframe": str(Cls.Path / "observation.json")}
        Reference["sha256"] = CameraDrift.Checksum(Reference, Cls.Model)
        Cls.Model["drift_reference"] = Reference
        Cls.Points = Rng.uniform([-20, -10, 42], [20, 15, 52], (80, 3))

    @classmethod
    def tearDownClass(Cls):
        Robot._LoadReference.cache_clear()
        Cls.Temp.cleanup()

    def Check(Self, Image=None, Depth=None, Model=None, Arm=None):
        return Robot.Check(Self.Image if Image is None else Image, Self.Depth if Depth is None else Depth,
                           Self.Intrinsics, Self.Model if Model is None else Model, Self.Arm if Arm is None else Arm)

    def Render(Self, Rvec, Translation):
        Rotation = cv2.Rodrigues(np.asarray(Rvec, dtype=float))[0]
        Translation = np.asarray(Translation, dtype=float)
        K = np.array([[500, 0, 320], [0, 500, 240], [0, 0, 1]], dtype=float)
        Homography = K @ (Rotation + np.outer(Translation, [0, 0, 1]) / 46.0) @ np.linalg.inv(K)
        Image = cv2.warpPerspective(Self.Image, Homography, (640, 480))
        U, V = np.meshgrid(np.arange(640), np.arange(480))
        Pixels = np.stack([U, V, np.ones_like(U)], axis=-1).reshape(-1, 3)
        Old = Pixels @ np.linalg.inv(Homography).T
        Old /= Old[:, 2, None]
        Camera = np.column_stack([(Old[:, 0] - 320) / 500 * 46, (Old[:, 1] - 240) / 500 * 46, np.full(len(Old), 46)])
        New = Camera @ Rotation.T + Translation
        Valid = (Old[:, 0] >= 0) & (Old[:, 0] < 640) & (Old[:, 1] >= 0) & (Old[:, 1] < 480)
        Depth = np.where(Valid, np.rint(New[:, 2] * 10), 0).astype(np.uint16).reshape(480, 640)
        return Image, Depth

    def test_archived_rgb_and_depth_accept_unchanged_mount(Self):
        Result = Self.Check()
        Self.assertIsNotNone(Result)
        Self.assertTrue(Result["ok"], Result)
        Self.assertEqual(Result["alignment_source"], "robot")

    def test_metric_translation_and_each_rotation_axis_are_measured(Self):
        for Axis in range(3):
            Rvec = np.zeros(3); Rvec[Axis] = np.radians(2)
            Rotation = cv2.Rodrigues(Rvec)[0]
            Result = Robot.FitCorrespondences(Self.Points, Self.Points @ Rotation.T)
            Self.assertIsNotNone(Result)
            Self.assertFalse(Result["ok"], Result)
            Self.assertAlmostEqual(Result["robot_rotation_deg"], 2, places=3)
        Result = Robot.FitCorrespondences(Self.Points, Self.Points + [1.2, 0, 0])
        Self.assertFalse(Result["ok"])
        Self.assertAlmostEqual(Result["robot_translation_cm"], 1.2, places=3)

    def test_rendered_moved_camera_is_blocked_using_pixels_and_depth(Self):
        for Rvec, Shift in [([0, 0, np.radians(2)], [0, 0, 0]), ([0, 0, 0], [2, 0, 0])]:
            Image, Depth = Self.Render(Rvec, Shift)
            Result = Self.Check(Image, Depth)
            Self.assertIsNotNone(Result)
            Self.assertFalse(Result["ok"], Result)
            Self.assertEqual(Result["error_code"], "camera_moved")

    def test_small_noise_and_some_bad_matches_do_not_erase_a_real_fit(Self):
        Rng = np.random.default_rng(25)
        Target = Self.Points + Rng.normal(0, 0.02, Self.Points.shape)
        Target[:10] += [5, -8, 10]
        Result = Robot.FitCorrespondences(Self.Points, Target)
        Self.assertTrue(Result["ok"], Result)
        Self.assertEqual(Result["robot_inliers"], 70)

    def test_narrow_repetitive_or_occluded_geometry_never_passes(Self):
        Line = np.column_stack([np.linspace(-20, 20, 40), np.zeros(40), np.full(40, 45)])
        Self.assertIsNone(Robot.FitCorrespondences(Line, Line))
        Self.assertIsNone(Self.Check(np.zeros_like(Self.Image)))
        Self.assertIsNone(Self.Check(Depth=np.zeros_like(Self.Depth)))
        Occluded = Self.Image.copy(); Occluded[:, :600] = 0
        Self.assertIsNone(Self.Check(Occluded))
        Fake = {"points": Self.Points, "descriptors": np.zeros((80, 32), dtype=np.uint8)}
        Self.assertIsNone(Robot._Pairs(Fake, Fake))

    def test_untrusted_archive_changed_intrinsics_and_unknown_arm_are_unavailable(Self):
        Model = copy.deepcopy(Self.Model); Model["tvec"][0] += 1
        Self.assertIsNone(Self.Check(Model=Model))
        Self.assertIsNone(Self.Check(Arm={"ok": False}))
        Intrinsics = dict(Self.Intrinsics, fx=501)
        Self.assertIsNone(Robot.Check(Self.Image, Self.Depth, Intrinsics, Self.Model, Self.Arm))

    def test_current_shelf_texture_cannot_replace_the_archive(Self):
        Random = np.random.default_rng(912).integers(0, 256, Self.Image.shape, dtype=np.uint8)
        Self.assertIsNone(Self.Check(Random))
        Before = Self.Check()
        Self.Check(Random)
        After = Self.Check()
        Self.assertEqual(Before["robot_reference"], After["robot_reference"])
        Self.assertTrue(After["ok"])

    def test_real_archive_is_audited_without_becoming_required_for_portable_tests(Self):
        import RecordedCaptures as RC
        # The calibration in use (tracked in git) names its keyframe in its own run's archive (absolute path).
        Calibration = Path(__file__).resolve().parent / "camera_calibration.json"
        Model = json.loads(Calibration.read_text(encoding="utf-8"))["camera_model"]
        Path_ = Path(Model["drift_reference"]["keyframe"])
        RC.Need(Path_, What="the calibration's archived keyframe (its run in back end/captures)")
        Record = json.loads(Path_.read_text(encoding="utf-8"))
        Image = cv2.imdecode(np.frombuffer(base64.b64decode(Record["images"][0].split(",", 1)[1]), np.uint8), 1)
        Depth = cv2.imread(str(Path_.parent / Record["depth_frames_png"][0]), -1)
        Result = Robot.Check(Image, Depth, Record["depth_intrinsics"], Model, {"ok": True, "joints": Record["joints"]})
        Self.assertTrue(Result is None or Result["ok"], Result)
        print("recorded robot-reference audit:", Result)


if __name__ == "__main__":
    unittest.main()
