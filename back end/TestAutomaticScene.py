"""Synthetic-only automatic supports and conservative one-frame scene coverage."""
import json
import os
os.environ.setdefault("SO101_LOCATE_MODE", "vlm")  # this suite tests the voted multi-call locate; TestSingleCallLocate.py the default
import time
import unittest
from itertools import product
from pathlib import Path
from unittest.mock import MagicMock, patch

import numpy as np

import TestSurfaceSupport as T
import DepthGeometry as DG
import SceneGeometry as SG
import object_locator as Locator


class Registry:
    """Contract double: no process-global bridge or hardware access."""
    def __init__(self, previous=None):
        self.invalidated = 0
        self.registered = []
        self.previous = previous

    def Previous(self, scene_id, model):
        if not self.previous or scene_id != self.previous["id"] or self.invalidated:
            raise ValueError("previous scene unavailable")
        return self.previous

    def Invalidate(self):
        self.invalidated += 1
        return self.invalidated

    def Register(self, scene, model, capture_time=None, generation=None):
        if generation != self.invalidated:
            raise ValueError("superseded capture")
        self.registered.append((scene, model, capture_time))
        return {**scene, "id": "synthetic-scene", "revision": "synthetic-revision",
                "camera_revision": "synthetic-camera", "captured_at": capture_time or time.time()}


def Measured(depth, item, camera=T.Camera):
    surfaces = DG.WorkspaceSurfaces(None, T.TableZ, Automatic=True)
    result = DG.MeasureBox(depth, T.TDC.CaptureBox(camera, *item), T.TDC.Intrinsics,
                           camera, T.TableZ, True, Surfaces=surfaces)
    return {**result, "label": "can"}


def ShelfScene():
    camera = T.TDC.CameraModel([-25, 0, 30], [-13, 34, 17])
    base = (np.array([-28, 25, 10]), np.array([0, 48, 12.7]))
    can = (np.array([-16, 32, 12.7]), np.array([-10, 38, 19.7]))
    board = (np.array([-28, 25, 36]), np.array([0, 48, 38]))
    back = (np.array([-28, 47, 12.7]), np.array([0, 49, 38]))
    sides = [(np.array([-30, 25, 12.7]), np.array([-28, 49, 38])),
             (np.array([0, 25, 12.7]), np.array([2, 49, 38]))]
    return camera, can, [base, can, board, back, *sides]


def Locate(depth, item, workspace, camera=T.Camera, registry=None, carrying=False, arm=None, detected=True, labels=("can",)):
    model = T.TDC.Verified(camera, workspace, FloorZ=T.TDC.RoomFloorZ)
    detections = [T.TDC.Detected(label, T.TDC.CaptureBox(camera, *item)) for label in labels] if detected else []
    registry = registry or Registry()
    with patch.object(Locator, "SceneSafety", registry), \
            patch.object(T.DeclaredSurfaces, "Load", return_value=[]), \
            patch.object(T.calibration, "camera_model", return_value=model), \
            patch.object(T.calibration, "status", return_value={"calibrated": True, "rms_cm": 0.3}), \
            patch.object(T.calibration, "pixel_to_plane", return_value={"ok": False}), \
            patch.object(Locator, "_pick_model", return_value="synthetic"), \
            patch.object(Locator, "_detect_once", return_value=detections), \
            patch.object(Locator, "_refine_detection", return_value=None), \
            patch.object(Locator, "_Record"):
        result = Locator.locate_objects(T.TDC.Encode(np.zeros((T.Height, T.Width, 3), np.uint8)),
                                        depth={"depth_mm": depth, "intrinsics": T.TDC.Intrinsics,
                                               "mirrored": True, "seq": 12, "time": time.time()},
                                        mirrored=True, arm=arm, task_object={"phrase": "the can", "label": "can",
                                            **({"carrying": True, "carry_scene_id": "synthetic-scene"} if carrying else {})})
    return result, registry


class AutomaticSceneTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.platform_depth = T.TDC.RenderDepth(T.Camera, T.TDC.Workspace([T.Platform, T.Can]), FloorZ=T.TDC.RoomFloorZ)
        cls.low_depth = T.TDC.RenderDepth(T.Camera, T.ShelfWorkspace([T.ShelfCans[1]]), FloorZ=T.TDC.RoomFloorZ)

    def test_raised_and_low_support_need_no_declaration(self):
        for depth, item, expected in ((self.platform_depth, T.Can, T.PlatformTop),
                                      (self.low_depth, T.ShelfCans[1], T.ShelfTop)):
            with self.subTest(z=expected):
                measured = Measured(depth, item)
                self.assertTrue(measured["ok"], measured)
                self.assertTrue(measured["automatic_surface"])
                self.assertAlmostEqual(measured["z"], expected, delta=0.3)
                support = measured["support_geometry"]
                self.assertEqual((support["Source"], support["Frame"], support["ObservedOnly"]), ("depth", "cart", True))
                self.assertLess(support["Evidence"]["height_spread_cm"], 0.2)

    def test_registry_receives_same_capture_and_invalidates_first(self):
        result, registry = Locate(self.platform_depth, T.Can, T.TDC.Workspace())
        self.assertTrue(result["ok"], result)
        self.assertEqual(registry.invalidated, 1)
        self.assertEqual(len(registry.registered), 1)
        scene = result["scene"]
        self.assertEqual(scene["capture_seq"], 12)
        self.assertEqual(scene["frame"], "display_cm")
        self.assertEqual(result["objects"][0]["scene_id"], scene["id"])
        self.assertEqual(result["objects"][0]["support_geometry"], scene["support"])
        self.assertTrue(scene["obstacles"])
        self.assertTrue(scene["free_space"])

    def test_one_object_under_two_labels_still_registers_its_scene(self):
        # 2026-10-14 16:35:13 UTC: "White holder" and "white device", 0.1 cm apart, both under the task query's
        # box, so no scene was registered and every task stopped scene_unverified.
        result, registry = Locate(self.platform_depth, T.Can, T.TDC.Workspace(), labels=("can", "tin can"))
        self.assertTrue(result["ok"], result)
        self.assertNotIn("scene_error", result)
        self.assertEqual(len(registry.registered), 1)
        matches = [item for item in result["objects"] if item["task_match"]]
        self.assertEqual(len(matches), 1, result["objects"])
        self.assertEqual(matches[0]["scene_id"], result["scene"]["id"])
        twins = [item for item in result["objects"] if item.get("task_twin_of")]
        self.assertEqual([(twin["task_twin_of"], "scene_id" in twin) for twin in twins], [(matches[0]["label"], False)])

    def test_task_matches_at_two_spots_stay_unregistered(self):
        recorded = json.loads((Path(__file__).parent / "TestFixtures" / "SceneUnverified-2026-10-14" / "Locate.json").read_text("utf-8"))
        for record in recorded:
            matches = [dict(item) for item in record["objects"] if item["task_match"]]
            with self.subTest(at=record["time"]):
                self.assertGreater(len(matches), 1)
                self.assertEqual(len(Locator._OneTaskObject(matches)), 1)
        apart = [{"label": "white holder", "x": 8.0, "y": 28.0}, {"label": "white cup", "x": 20.0, "y": 28.0}]
        self.assertEqual(Locator._OneTaskObject(apart), apart)
        self.assertTrue(all("task_twin_of" not in item for item in apart))

    def test_failed_capture_revokes_previous_scene(self):
        registry = Registry()
        with patch.object(Locator, "SceneSafety", registry), patch.object(T.calibration, "status", return_value={}):
            self.assertFalse(Locator.locate_objects("not an image")["ok"])
        self.assertEqual(registry.invalidated, 1)
        self.assertFalse(registry.registered)

    def test_carry_refresh_needs_trusted_pose_and_never_erases_new_source_obstacles(self):
        original, _ = Locate(self.platform_depth, T.Can, T.TDC.Workspace())
        previous = original["scene"]
        refused, registry = Locate(self.platform_depth, T.Can, T.TDC.Workspace(),
                                   registry=Registry(previous), carrying=True, detected=False)
        self.assertIn("trusted current arm pose", refused["scene_error"])
        self.assertFalse(registry.registered)
        # The target/another object remains at the old source. A carry refresh
        # must retain it as an obstacle, not reapply the old removal envelope.
        arm = {"ok": True, "joints": [[0, 0, 30], [0, 1, 30]], "servos_raw": []}
        refreshed, registry = Locate(self.platform_depth, T.Can, T.TDC.Workspace(),
                                     registry=Registry(previous), carrying=True, arm=arm, detected=False)
        self.assertIn("scene", refreshed, refreshed)
        self.assertEqual(registry.invalidated, 1)
        self.assertTrue(refreshed["scene"]["carrying"])
        self.assertEqual(refreshed["scene"]["task_object"], previous["task_object"])
        low, high = np.array(T.Can[0]), np.array(T.Can[1])
        self.assertTrue(any(np.all(np.array(b["max"]) > low + [0, 0, 1]) and np.all(np.array(b["min"]) < high)
                            for b in refreshed["scene"]["obstacles"]))

    def test_support_translation_is_measured_in_the_rigid_camera_frame(self):
        shift = np.array([18, -7, 3])
        platform = tuple(point + shift for point in T.Platform)
        can = tuple(point + shift for point in T.Can)
        depth = T.TDC.RenderDepth(T.Camera, T.TDC.Workspace([platform, can]), FloorZ=T.TDC.RoomFloorZ)
        measured = Measured(depth, can)
        self.assertTrue(measured["ok"], measured)
        self.assertAlmostEqual(measured["z"], T.PlatformTop + 3, delta=0.3)
        self.assertLess(abs(measured["x"] - (sum(point[0] for point in can) / 2)), 1.0)

    def test_failed_camera_gate_never_calls_model_or_register(self):
        registry = Registry()
        with patch.object(Locator, "SceneSafety", registry), patch.object(T.calibration, "status", return_value={}), \
                patch.object(T.calibration, "camera_model", return_value=T.Camera), \
                patch.object(T.CD, "CheckCapture", return_value={"ok": False, "error_code": "camera_moved", "error": "alignment lost"}), \
                patch.object(Locator, "_pick_model") as model:
            result = Locator.locate_objects(T.TDC.Encode(np.zeros((T.Height, T.Width, 3), np.uint8)))
        self.assertFalse(result["ok"])
        model.assert_not_called()
        self.assertEqual(registry.invalidated, 1)
        self.assertFalse(registry.registered)

    def test_merge_does_not_bridge_missing_cells(self):
        cells = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 2, 0], [1, 2, 0]])
        boxes = SG._MergeCells(cells, 1, np.zeros(3))
        recovered = set()
        for box in boxes:
            recovered.update(product(*(range(int(a), int(b)) for a, b in zip(box["min"], box["max"]))))
        self.assertEqual(recovered, set(map(tuple, cells)))

    def test_distorted_projection_bound_covers_interior_and_edges(self):
        rng = np.random.default_rng(441)
        lows = rng.uniform([-35, -25, 10], [35, 25, 80], (150, 3))
        sizes = rng.uniform(0.1, 9, (150, 3))
        corners = lows[:, None, :] + sizes[:, None, :] * np.array(list(product((0, 1), repeat=3)))
        lower, upper = SG._ProjectedBounds(corners, T.TDC.Intrinsics)
        points = lows[:, None, :] + sizes[:, None, :] * rng.random((150, 200, 3))
        pixels = DG.Project(points.reshape(-1, 3), T.TDC.Intrinsics).reshape(150, 200, 2)
        self.assertTrue(np.all(pixels >= lower[:, None] - 1e-8))
        self.assertTrue(np.all(pixels <= upper[:, None] + 1e-8))
        with self.assertRaisesRegex(ValueError, "lens distortion"):
            SG._ProjectedBounds(corners, {**T.TDC.Intrinsics, "dist": [0] * 8})

    def test_missing_depth_is_unknown_and_free_space_never_intersects_solids(self):
        self.assertEqual(SG._FreeBoxes(np.zeros_like(self.platform_depth), T.TDC.Intrinsics, T.Camera), [])
        free = SG._FreeBoxes(self.platform_depth, T.TDC.Intrinsics, T.Camera)
        self.assertTrue(free)
        for box in free:
            low, high = np.array(box["min"]), np.array(box["max"])
            for solid_low, solid_high in T.TDC.Workspace([T.Platform, T.Can]):
                self.assertFalse(np.all(high > solid_low + 1e-6) and np.all(low < solid_high - 1e-6), (box, solid_low, solid_high))
        # Covering the central part with unavailable pixels can only remove free voxels.
        holes = self.platform_depth.copy()
        holes[200:500, 400:850] = 0
        with_holes = SG._FreeBoxes(holes, T.TDC.Intrinsics, T.Camera)
        def volume(boxes):
            return sum(float(np.prod(np.array(b["max"]) - b["min"])) for b in boxes)
        self.assertLess(volume(with_holes), volume(free))

    def test_neighbour_top_does_not_become_the_objects_support(self):
        target = (np.array([-17, 35, T.TableZ]), np.array([-11, 41, T.TableZ + 13]))
        neighbours = [
            (np.array([-28, 30, T.TableZ]), np.array([-18, 45, T.TableZ + 5])),
            (np.array([-23, 28, T.TableZ]), np.array([-5, 33, T.TableZ + 5])),
            (np.array([-10, 30, T.TableZ]), np.array([0, 47, T.TableZ + 5])),
        ]
        for neighbour in neighbours:
            with self.subTest(neighbour=neighbour):
                depth = T.TDC.RenderDepth(T.Camera, T.TDC.Workspace([target, neighbour]), FloorZ=T.TDC.RoomFloorZ)
                result = Measured(depth, target)
                self.assertTrue(not result["ok"] or abs(result["z"] - T.TableZ) < 0.3, result)
                self.assertFalse(result.get("automatic_surface"), result)

    def test_uncertain_multiple_supports_are_refused(self):
        candidates = [{"undeclared_support_z": 3}, {"undeclared_support_z": 6}]
        with patch.object(DG, "_SupportCandidates", return_value=candidates):
            result = Measured(self.platform_depth, T.Can)
        self.assertFalse(result["ok"])
        self.assertIn("more than one", result["error"])

    def test_target_is_separate_while_neighbour_and_board_remain_occupied(self):
        neighbour = (np.array([-8, 34, T.PlatformTop]), np.array([-4, 40, T.PlatformTop + 8]))
        board = (np.array([-26, 30, T.PlatformTop]), np.array([-24, 44, T.PlatformTop + 17]))
        depth = T.TDC.RenderDepth(T.Camera, T.TDC.Workspace([T.Platform, T.Can, neighbour, board]), FloorZ=T.TDC.RoomFloorZ)
        measured = Measured(depth, T.Can)
        self.assertTrue(measured["ok"], measured)
        scene = SG.Build(depth, T.TDC.Intrinsics, T.Camera, measured, measured["support_geometry"])
        for low, high in (T.Platform, neighbour, board):
            self.assertTrue(any(np.all(np.array(b["max"]) > low) and np.all(np.array(b["min"]) < high) for b in scene["obstacles"]))
        self.assertEqual(scene["task_object"]["label"], "can")
        target_low, target_high = np.array(scene["task_object"]["min"]), np.array(scene["task_object"]["max"])
        for box in scene["obstacles"]:
            self.assertFalse(np.all(np.array(box["max"]) > target_low + 1e-6) and np.all(np.array(box["min"]) < target_high - 1e-6), box)

    def test_observed_shelf_underside_back_and_sides_remain_obstacles(self):
        camera, can, solids = ShelfScene()
        depth = T.TDC.RenderDepth(camera, solids, FloorZ=T.TDC.RoomFloorZ)
        measured = Measured(depth, can, camera)
        self.assertTrue(measured["ok"], measured)
        self.assertLess(max(measured["size_cm"]), 6.2, "background shelf walls cannot join the target component")
        scene = SG.Build(depth, T.TDC.Intrinsics, camera, measured, measured["support_geometry"])
        self.assertEqual(scene["support"]["Kind"], "shelf")
        self.assertAlmostEqual(scene["support"]["BoardAboveZCm"], 36, delta=0.1)
        for low, high in solids[2:]:
            self.assertTrue(any(np.all(np.array(b["max"]) > low) and np.all(np.array(b["min"]) < high) for b in scene["obstacles"]), (low, high))
        for box in scene["obstacles"]:
            if box["label"] == "observed support":
                self.assertLessEqual(box["max"][2], scene["support"]["TopZCm"])


# --------------------------------------------------------------------------
# No calibrated desk in view: the camera checked against the gripper tip (RobotReference)
# --------------------------------------------------------------------------
# The cart at a cabinet: the bin, the room floor and a cabinet top 4 cm above the old desk line, no desk.
CabinetTop = T.TableZ + 4.0
CabinetCan = (np.array([-22.0, 36.0, CabinetTop]), np.array([-16.0, 42.0, CabinetTop + 8.0]))


def NoDeskWorkspace(Items=(), Top=CabinetTop):
    return [T.TDC.Workspace()[1], (np.array([-40.0, 28.0, T.TDC.RoomFloorZ]), np.array([-3.0, 62.0, Top])), *Items]


def MeasuredNoDesk(depth, item, Desk=False, camera=T.Camera):
    surfaces = DG.WorkspaceSurfaces(None, T.TableZ, Automatic=True, Desk=Desk)
    return DG.MeasureBox(depth, T.TDC.CaptureBox(camera, *item), T.TDC.Intrinsics, camera, T.TableZ, True, Surfaces=surfaces)


# What CameraDrift.CheckCapture returns for a capture checked against the robot (stage 1's RobotReference.Check).
RobotVerified = {"ok": True, "angle_deg": None, "offset_cm": None, "reason": "no calibrated desk in view; the gripper tape is 0.4 cm from where the arm puts it",
                 "reference_fitted_at": "2026-09-28T15:04:00", "seq": 12, "alignment_source": "robot", "robot_check": "verified",
                 "tape_check": "ok", "tape_error_cm": 0.4, "desk_check": {"ok": False, "error_code": "camera_check_failed"}}
RobotAssumed = {**RobotVerified, "robot_check": "assumed", "tape_check": "not_visible", "tape_error_cm": None,
                "warning": "Camera assumed where it was calibrated at 15:04: the gripper tip could not be checked (the arm was moving around the capture)."}
DeskVerified = {"ok": True, "angle_deg": 0.1, "offset_cm": 0.05, "reason": "desk matches the calibration", "reference_fitted_at": "2026-09-28T15:04:00", "seq": 12}


def LocateChecked(depth, items, check, tripwire=None):
    """locate_objects on depth with the camera check's result given (CheckCapture patched), every item detected as "can N"."""
    model = T.TDC.Verified(T.Camera, T.TDC.Workspace(), FloorZ=T.TDC.RoomFloorZ)
    detections = [T.TDC.Detected(f"can {index}", T.TDC.CaptureBox(T.Camera, *item)) for index, item in enumerate(items)]
    registry = Registry()
    with patch.object(Locator, "SceneSafety", registry), \
            patch.object(T.DeclaredSurfaces, "Load", return_value=[]), \
            patch.object(T.calibration, "camera_model", return_value=model), \
            patch.object(T.calibration, "status", return_value={"calibrated": True, "rms_cm": 0.3}), \
            patch.object(T.calibration, "pixel_to_plane", return_value={"ok": False}), \
            patch.object(T.CD, "CheckCapture", return_value=json.loads(json.dumps(check))), \
            patch.object(Locator, "_pick_model", return_value="synthetic"), \
            patch.object(Locator, "_detect_once", return_value=detections), \
            patch.object(Locator, "_refine_detection", return_value=None), \
            patch.object(Locator, "_SceneTripwire", tripwire or MagicMock(return_value=None)) as wire, \
            patch.object(Locator, "_Record"):
        result = Locator.locate_objects(T.TDC.Encode(np.zeros((T.Height, T.Width, 3), np.uint8)),
                                        depth={"depth_mm": depth, "intrinsics": T.TDC.Intrinsics, "mirrored": True, "seq": 12, "time": time.time()},
                                        mirrored=True, task_object={"phrase": "the can", "label": "can 0"})
    return result, wire


class NoDeskMeasureTests(unittest.TestCase):
    """MeasureBox and locate_objects when the calibrated desk is not in view (WorkspaceSurfaces Desk=False)."""

    @classmethod
    def setUpClass(cls):
        cls.depth = T.TDC.RenderDepth(T.Camera, NoDeskWorkspace([CabinetCan]), FloorZ=T.TDC.RoomFloorZ)

    def test_a_can_on_a_cabinet_is_measured_on_the_surface_around_it(self):
        measured = MeasuredNoDesk(self.depth, CabinetCan)
        self.assertTrue(measured["ok"], measured)
        self.assertEqual((measured["support_branch"], measured["automatic_surface"]), ("surface", True))
        self.assertAlmostEqual(measured["z"], CabinetTop, delta=0.3)
        self.assertAlmostEqual(measured["height_cm"], 8.0, delta=0.4)
        self.assertEqual(measured["support_geometry"]["Source"], "depth")
        # No desk branch and nothing for the desk's tripwire.
        for key in ("desk_area", "desk_ring_offset_cm", "nominal_contact_cm", "desk_offset_cm"):
            self.assertNotIn(key, measured)

    def test_a_surface_at_the_old_desk_height_is_no_desk_without_one(self):
        # A cabinet top 0.8 cm above the old desk line: at a verified desk it is the desk (as before); with no
        # calibrated desk in view the old desk's height excludes nothing, so it is the measured surface.
        top = T.TableZ + 0.8
        can = (np.array([-22.0, 36.0, top]), np.array([-16.0, 42.0, top + 8.0]))
        depth = T.TDC.RenderDepth(T.Camera, NoDeskWorkspace([can], Top=top), FloorZ=T.TDC.RoomFloorZ)
        desk, none = MeasuredNoDesk(depth, can, Desk=True), MeasuredNoDesk(depth, can, Desk=False)
        self.assertTrue(desk["ok"] and none["ok"], (desk, none))
        self.assertEqual(desk["support_branch"], "desk")
        self.assertEqual((none["support_branch"], none["automatic_surface"]), ("surface", True))
        for measured in (desk, none):
            self.assertAlmostEqual(measured["z"], top, delta=0.3)
            self.assertAlmostEqual(measured["height_cm"], 8.0, delta=0.4)

    def test_nothing_level_around_is_withheld_saying_so(self):
        # A can on a 2.5 cm post in front of the cabinet: no level surface around it, no desk to fall back on.
        post = (np.array([-21.0, 30.0, T.TDC.RoomFloorZ]), np.array([-18.5, 32.5, T.TableZ + 10.0]))
        can = (np.array([-22.0, 29.0, T.TableZ + 10.0]), np.array([-17.5, 33.5, T.TableZ + 18.0]))
        depth = T.TDC.RenderDepth(T.Camera, [T.TDC.Workspace()[1], post, can], FloorZ=T.TDC.RoomFloorZ)
        measured = MeasuredNoDesk(depth, can)
        self.assertFalse(measured["ok"], measured)
        self.assertEqual(measured["error"], DG.NoDeskSupportError)

    def test_without_the_desk_the_largest_level_peaks_are_tried_first(self):
        # Nine small level patches above a large one (the cabinet top under a can with books behind it): at a desk the
        # eight highest are tried, as before; with none, the largest first, so the cabinet top is among them.
        rng = np.random.default_rng(5)
        patches = [np.column_stack([rng.uniform(-30, -26, 60), rng.uniform(44, 48, 60), np.full(60, 6.0 + index)]) for index in range(9)]
        cabinet = np.column_stack([rng.uniform(-34, -14, 900), rng.uniform(30, 46, 900), np.full(900, 4.1)])
        ring = np.vstack([cabinet, *patches])
        tried = []

        def measure_on(World, Valid, Crop, Ring, Branch, Prior, Surfaces, Surface=None):
            tried.append(round(float(Prior(0.0, 0.0)), 1))
            return {"ok": False, "support_found": False}
        with patch.object(DG, "_MeasureOn", side_effect=measure_on):
            for desk in (True, False):
                tried.clear()
                DG._SupportCandidates(None, None, None, ring, DG.WorkspaceSurfaces(None, -20.0, Automatic=True, Desk=desk))
                self.assertEqual(4.1 in tried, not desk, (desk, tried))
                self.assertEqual(len(tried), 8)

    def test_robot_checked_captures_measure_without_the_desk_and_only_assumed_ones_keep_the_tripwire(self):
        # Aaran's decision (2026-10-07 audit IH1): a camera the gripper tip could not check ("assumed") still lets
        # the task run, but is not treated as verified: the tripwire runs on it and its scene is not "checked" (the
        # app keeps the [UNRELIABLE] tags). One the tip verified skips the desk's tripwire.
        trip = MagicMock(return_value=("tripped", "The camera or the cart moved since calibration."))
        clear = MagicMock(return_value=None)
        for check, verdict in ((RobotVerified, "verified"), (RobotAssumed, "assumed")):
            with self.subTest(verdict):
                result, wire = LocateChecked(self.depth, [CabinetCan], check, clear)
                self.assertTrue(result["ok"], result)
                self.assertEqual(wire.called, verdict == "assumed")
                self.assertEqual(result["no_desk"], {"robot_check": verdict, "supports": "measured"})
                [measured] = result["objects"]
                self.assertEqual(measured["support_branch"], "surface")
                self.assertAlmostEqual(measured["z"], CabinetTop, delta=0.3)
                self.assertIn("scene", result)
                self.assertEqual(measured["scene_checked"], verdict == "verified")
                # The tripwire judges the assumed capture on its desk readings (taken though the desk is no support).
                if verdict == "assumed":
                    diagnostics = wire.call_args[0][0]
                    self.assertTrue(diagnostics and all("desk_area" in item for item in diagnostics), diagnostics)
                tripped, _ = LocateChecked(self.depth, [CabinetCan], check, trip)
                self.assertEqual(tripped["ok"], verdict == "verified", tripped)
                if verdict == "assumed":
                    self.assertEqual(tripped["error_code"], "camera_moved")
        # A desk-verified capture keeps the tripwire, and the reply as before.
        trip.reset_mock()
        result, wire = LocateChecked(self.depth, [CabinetCan], DeskVerified, trip)
        wire.assert_called_once()
        self.assertEqual((result["ok"], result["error_code"]), (False, "camera_moved"))
        self.assertNotIn("no_desk", result)

    def test_an_assumed_camera_never_takes_the_task_reach_as_free(self):
        # H2 is on (Aaran's decision, 2026-10-25) for a checked camera only: a capture the gripper tip could not check
        # ("assumed", audit IH1) keeps free space to what the camera observed (README, HOW_IT_WORKS); 0d65c7e gave it
        # the task's reach too.
        self.assertTrue(SG.AssumeTaskReachFree)
        clear = MagicMock(return_value=None)
        for check, coverage in ((RobotVerified, "observed_and_task_reach"), (RobotAssumed, "observed_only")):
            with self.subTest(check["robot_check"]):
                result, _ = LocateChecked(self.depth, [CabinetCan], check, clear)
                self.assertTrue(result["ok"], result)
                scene = result["scene"]
                self.assertEqual(scene["coverage"], coverage)
                if coverage == "observed_only":
                    self.assertNotIn("known_space", scene)
                else:
                    [reach] = scene["known_space"]
                    self.assertEqual(reach["max"][2], round(scene["task_object"]["max"][2] + SG.TaskReachAboveCm, 3))

    def test_an_object_measured_twice_counts_once_in_the_tripwire(self):
        # The task object withheld from the samples' box and measured again from the task query's: one object.
        check = {**DeskVerified, "angle_deg": 0.0, "offset_cm": 0.0}
        one = {"label": "cup", "box": [0.40, 0.30, 0.48, 0.42], "desk_area": True, "desk_ring_offset_cm": -3.0, "support_branch": "desk"}
        again = {**one, "box": [0.405, 0.30, 0.48, 0.43], "task_query_only": True, "refine": "task"}
        other = {**one, "label": "book", "box": [0.10, 0.30, 0.20, 0.40]}
        third = {**one, "label": "box", "box": [0.60, 0.30, 0.70, 0.40]}
        self.assertEqual(Locator._Distinct([one, again, other]), [one, other])
        self.assertIsNone(Locator._SceneTripwire([one, again, other], check))
        self.assertIsNotNone(Locator._SceneTripwire([one, again, other, third], check))
        # A query box elsewhere is another object, and counts.
        elsewhere = {**again, "box": [0.80, 0.30, 0.90, 0.40]}
        self.assertIsNotNone(Locator._SceneTripwire([one, elsewhere, other], check))

    def test_an_assumed_camera_with_no_desk_trips_on_the_basket_rim(self):
        # 2026-10-08 review (decision 4, audit IH1): with no desk in view the desk readings cannot trip, so the tripwire
        # kept for an "assumed" camera was empty. Its tripwire there is the basket rim (RobotReference.RimCheck): it
        # trips when the rim positively reads moved, and only for an assumed camera (one the tip verified skips it).
        moved = {**RobotAssumed, "rim_check": "moved", "rim_yaw_deg": 2.96, "rim_shift_cm": [0.34, 0.48]}
        reason, sentence = Locator._SceneTripwire([], moved)
        self.assertIn("basket rim reads 2.96 deg, +0.34 / +0.48 cm", reason)
        self.assertIn("Lock the camera head, then run Automatic calibration", sentence)
        for rim in ("ok", "not_judgeable", "unavailable", None):
            self.assertIsNone(Locator._SceneTripwire([], {**RobotAssumed, "rim_check": rim}), rim)
        self.assertIsNone(Locator._SceneTripwire([], {**RobotVerified, "rim_check": "moved"}), "the tip verified it")
        with patch.object(Locator, "TRIPWIRE_ASSUMED_RIM", False):
            self.assertIsNone(Locator._SceneTripwire([], moved), "switched off: the desk readings only")


# --------------------------------------------------------------------------
# The robot in its own scene (H3), the support under the object (H1), the task's reach (H2)
# --------------------------------------------------------------------------
def Cell(point):
    return np.floor((np.asarray(point, dtype=float) - SG.BoundsMin) / SG.OccupiedCellCm).astype(np.int32)


class SceneRobotTests(unittest.TestCase):
    def test_the_cart_basket_and_mast_are_no_obstacles_but_things_in_the_bin_are(self):
        # Display frame: the bin floor about Z -9, its front rim about Y 23 at Z 0, the mast behind the arms.
        on_robot = [(0.0, 10.0, -8.8), (10.0, 23.0, -0.5), (0.0, -10.0, 20.0)]
        off_robot = [(0.0, 10.0, -4.0), (-25.0, 40.0, 4.1), (-30.0, 27.0, 10.0)]
        cells = np.array([Cell(point) for point in on_robot + off_robot])
        self.assertEqual(SG.RobotCells(cells).tolist(), [True] * len(on_robot) + [False] * len(off_robot))

    def test_the_arm_is_left_out_where_it_stood_and_its_place_is_known(self):
        import bridge_server as Bridge
        controller = Bridge.BridgeController()
        boxes = controller.ArmLinkBoxes(controller._current_ik_angles())
        self.assertEqual([box.get("fixed", False) for box in boxes].count(True), 3, "each collar box's pan revolution")
        jaw = next(box for box in boxes if box["name"].startswith("moving_jaw"))
        axes, centre, half = np.asarray(jaw["axes"]), np.asarray(jaw["centre"]), np.asarray(jaw["half"])
        inside = centre + axes[:, 0] * (half[0] + SG.ArmMarginCm - 0.1)
        outside = centre + axes[:, 0] * (half[0] + SG.ArmMarginCm + 0.5)
        self.assertEqual(SG.ArmPoints(np.array([centre, inside, outside]), boxes).tolist(), [True, True, False])
        # The collar's revolution, fixed on the cart, is grown like the cart (its servo cables hang about it).
        collar = next(box for box in boxes if box.get("fixed"))
        cable = np.asarray(collar["centre"]) + [0.0, collar["half"][1] + SG.RobotMarginCm - 0.2, 0.0]
        self.assertTrue(SG.ArmPoints(cable[None], [collar])[0])
        # Where the wrist and the gripper stood is known space, around every corner of their boxes.
        space = SG.ArmSpace(boxes)
        self.assertEqual(len(space), sum(box["name"].startswith(("wrist_link", "gripper_link", "moving_jaw")) for box in boxes))
        for box in boxes:
            if not box["name"].startswith(("gripper_link", "moving_jaw")):
                continue
            corners = np.asarray(box["centre"]) + (np.array(list(product((-1, 1), repeat=3))) * box["half"]) @ np.asarray(box["axes"]).T
            self.assertTrue(any(np.all(corners >= np.asarray(known["min"]) - 1e-6) and np.all(corners <= np.asarray(known["max"]) + 1e-6) for known in space))
        # Known only KnownArmMarginCm around them, not the cut-out's 2 cm (2026-10-08 review, audit 1 IM1: a board 0.5-1.9 cm
        # beside the gripper was cut out of the scene and declared free). The space known beyond the parts themselves is
        # about a third of what 2 cm gave (the rest is the bounding boxes of turned parts, which the coverage check uses).
        self.assertEqual((SG.KnownArmMarginCm, SG.ArmMarginCm), (0.5, 2.0))
        parts = [box for box in boxes if box["name"].startswith(("wrist_link", "gripper_link", "moving_jaw"))]
        low = np.min([np.asarray(box["centre"]) - 6 for box in parts], axis=0)
        high = np.max([np.asarray(box["centre"]) + 6 for box in parts], axis=0)
        grid = np.stack(np.meshgrid(*[np.arange(a, b, 0.5) for a, b in zip(low, high)], indexing="ij"), -1).reshape(-1, 3)
        arm = np.zeros(len(grid), dtype=bool)
        for box in parts:
            arm |= np.all(np.abs((grid - np.asarray(box["centre"])) @ np.asarray(box["axes"])) <= np.asarray(box["half"]), axis=1)

        def beyond(known_boxes):
            inside = np.zeros(len(grid), dtype=bool)
            for known in known_boxes:
                inside |= np.all((grid >= np.asarray(known["min"])) & (grid <= np.asarray(known["max"])), axis=1)
            return int((inside & ~arm).sum())
        self.assertLess(beyond(space), 0.45 * beyond(SG.ArmSpace(boxes, SG.ArmMarginCm)))

    def test_the_task_reach_is_known_only_when_switched_on(self):
        # H2 (NoDeskDesign.md open question 3): Aaran chose on (2026-10-25); off, the scene is as before.
        depth = T.TDC.RenderDepth(T.Camera, NoDeskWorkspace([CabinetCan]), FloorZ=T.TDC.RoomFloorZ)
        measured = {**MeasuredNoDesk(depth, CabinetCan), "label": "can"}
        self.assertTrue(SG.AssumeTaskReachFree)
        with patch.object(SG, "AssumeTaskReachFree", False):
            plain = SG.Build(depth, T.TDC.Intrinsics, T.Camera, measured, measured["support_geometry"])
        self.assertNotIn("known_space", plain)
        self.assertEqual(plain["coverage"], "observed_only")
        with patch.object(SG, "AssumeTaskReachFree", True):
            reached = SG.Build(depth, T.TDC.Intrinsics, T.Camera, measured, measured["support_geometry"])
        [reach] = reached["known_space"]
        task = reached["task_object"]
        self.assertEqual(reach["min"], [round(task["min"][0] - SG.TaskReachCm, 3), round(task["min"][1] - SG.TaskReachCm, 3),
                                        round(reached["support"]["TopZCm"] - SG.TaskReachBelowCm, 3)])
        self.assertEqual(reach["max"][2], round(task["max"][2] + SG.TaskReachAboveCm, 3))
        self.assertEqual(reached["obstacles"], plain["obstacles"], "observed solids stay obstacles")


# --------------------------------------------------------------------------
# The support's tilt, a neighbour against the object, and what hides behind it (H1, H2)
# --------------------------------------------------------------------------
def RenderTilted(Model, Boxes, Top, Slope, Pivot, XRange=(-40.0, -3.0), YRange=(28.0, 62.0), FloorZ=T.TDC.RoomFloorZ):
    """RenderDepth with a cabinet top over XRange, YRange rising Slope cm per cm of display Y about Y Pivot (at Top
    there): the cart leaning, the arm's frame about 4 deg forward of the room at the cabinet of 2026-10-06."""
    depth = T.TDC.RenderDepth(Model, Boxes, FloorZ).astype(np.float64).ravel() / 10.0
    depth[depth <= 0] = np.inf
    rays = DG.RayMap(T.TDC.Intrinsics).astype(np.float64).reshape(-1, 2)
    rotation, _, mirror = DG.PoseFromModel(Model)
    world = np.column_stack([rays, np.ones(len(rays))]) @ rotation
    if mirror:
        world = world * DG.MirrorVector
    centre = DG.CameraToDisplay(np.zeros(3), Model)[0]
    with np.errstate(divide="ignore", invalid="ignore"):
        hit = (Top + Slope * (centre[1] - Pivot) - centre[2]) / (world[:, 2] - Slope * world[:, 1])
        at = centre + hit[:, None] * world
    on = (hit > 0) & (at[:, 0] >= XRange[0]) & (at[:, 0] <= XRange[1]) & (at[:, 1] >= YRange[0]) & (at[:, 1] <= YRange[1]) & (hit < depth)
    depth = np.where(on, hit, depth)
    return np.where(np.isfinite(depth), np.round(depth * 10.0), 0).astype(np.uint16).reshape(T.Height, T.Width)


def SupportLevelCells(scene, measured, within):
    """The scene's observed-environment cells at the support's level (their observed top within SupportBandCm of the
    object's base) within `within` cm of its envelope."""
    low, high = np.array(scene["task_object"]["min"]), np.array(scene["task_object"]["max"])
    return [box for box in scene["obstacles"] if box["label"] == "observed environment"
            and box["max"][2] - SG.DepthUncertaintyCm <= measured["z"] + SG.SupportBandCm
            and np.all(np.array(box["min"][:2]) < high[:2] + within) and np.all(np.array(box["max"][:2]) > low[:2] - within)]


class SceneSupportTests(unittest.TestCase):
    def test_a_tilted_support_is_the_support_around_the_object(self):
        # A cabinet top tilted 3 deg (the cart leaning): beside the can it leaves its level's 0.35 cm band, and was
        # "observed environment" there (the jaws and the lifted can "touched" it). Its plane holds it, its top its own.
        slope, pivot = float(np.tan(np.radians(3.0))), 39.0
        can = (np.array([-22.0, 36.0, CabinetTop]), np.array([-16.0, 42.0, CabinetTop + 8.0]))
        body = (np.array([-40.0, 28.0, T.TDC.RoomFloorZ]), np.array([-3.0, 62.0, CabinetTop - slope * 11 - 0.2]))
        depth = RenderTilted(T.Camera, [T.TDC.Workspace()[1], body, can], CabinetTop, slope, pivot)
        measured = {**MeasuredNoDesk(depth, can), "label": "can"}
        self.assertTrue(measured["ok"], measured)
        scene = SG.Build(depth, T.TDC.Intrinsics, T.Camera, measured, measured["support_geometry"])
        self.assertAlmostEqual(scene["support_tilt_deg"], 3.0, delta=0.3)
        self.assertEqual(SupportLevelCells(scene, measured, 3.0), [])
        for box in scene["obstacles"]:
            if box["label"] == "observed support":
                self.assertLessEqual(box["max"][2], max(measured["z"], CabinetTop + slope * (box["max"][1] - pivot)) + 0.05, box)
        with patch.object(SG, "_SupportPlane", return_value=None):
            level = SG.Build(depth, T.TDC.Intrinsics, T.Camera, measured, measured["support_geometry"])
        self.assertNotIn("support_tilt_deg", level)
        self.assertGreater(len(SupportLevelCells(level, measured, 3.0)), 0, "as a level band, the tilted top was environment beside the can")

    def test_a_level_support_is_built_as_before(self):
        # Within SupportMinTiltDeg of level no plane is used: the shelf's support boxes are capped at its level.
        camera, can, solids = ShelfScene()
        depth = T.TDC.RenderDepth(camera, T.TDC.Workspace(solids), FloorZ=T.TDC.RoomFloorZ)
        measured = Measured(depth, can, camera)
        scene = SG.Build(depth, T.TDC.Intrinsics, camera, measured, measured["support_geometry"])
        self.assertNotIn("support_tilt_deg", scene)
        with patch.object(SG, "_SupportPlane", return_value=None):
            self.assertEqual(SG.Build(depth, T.TDC.Intrinsics, camera, measured, measured["support_geometry"])["obstacles"], scene["obstacles"])

    def test_a_neighbour_against_the_object_is_its_contact_a_gap_is_not(self):
        # A board 0.25 cm behind the can: the can's measurement takes in the part of it inside its box, and the rest of
        # it stands against that footprint (TaskContact). 2 cm behind, or nothing behind: no contact.
        contacts = {}
        for gap in (0.25, 2.0, None):
            items = [CabinetCan] + ([(np.array([-27.0, 42.0 + gap, CabinetTop]), np.array([-11.0, 42.75 + gap, CabinetTop + 16.0]))] if gap else [])
            depth = T.TDC.RenderDepth(T.Camera, NoDeskWorkspace(items), FloorZ=T.TDC.RoomFloorZ)
            measured = {**MeasuredNoDesk(depth, CabinetCan), "label": "can"}
            self.assertTrue(measured["ok"], measured)
            contacts[gap] = SG.Build(depth, T.TDC.Intrinsics, T.Camera, measured, measured["support_geometry"])["task_contact"]
        self.assertIsNone(contacts[2.0])
        self.assertIsNone(contacts[None])
        touching = contacts[0.25]
        self.assertGreaterEqual(touching["points"], SG.TouchMinPoints)
        self.assertTrue(42.0 <= touching["min"][1] and touching["max"][1] <= 42.8, touching)
        # A carried scene (RemoveTarget False) removes nothing and says nothing of a contact.
        depth = T.TDC.RenderDepth(T.Camera, NoDeskWorkspace([CabinetCan]), FloorZ=T.TDC.RoomFloorZ)
        measured = {**MeasuredNoDesk(depth, CabinetCan), "label": "can"}
        carried = SG.Build(depth, T.TDC.Intrinsics, T.Camera, {}, measured["support_geometry"], TaskEnvelope=SG.TaskBounds(measured), RemoveTarget=False)
        self.assertNotIn("task_contact", carried)

    def test_with_the_task_reach_what_hides_behind_the_object_is_solid(self):
        # A board 2 cm behind the can, its lower part hidden from the camera by the can: the task reach (H2) takes the
        # unobserved space there as empty, but not under what the camera saw of the board (HiddenUnder).
        board = (np.array([-27.0, 44.0, CabinetTop]), np.array([-11.0, 44.75, CabinetTop + 16.0]))
        depth = T.TDC.RenderDepth(T.Camera, NoDeskWorkspace([CabinetCan, board]), FloorZ=T.TDC.RoomFloorZ)
        measured = {**MeasuredNoDesk(depth, CabinetCan), "label": "can"}
        with patch.object(SG, "AssumeTaskReachFree", False):
            plain = SG.Build(depth, T.TDC.Intrinsics, T.Camera, measured, measured["support_geometry"])
        self.assertFalse(any(box["label"].startswith("hidden") for box in plain["obstacles"]))
        with patch.object(SG, "AssumeTaskReachFree", True):
            reached = SG.Build(depth, T.TDC.Intrinsics, T.Camera, measured, measured["support_geometry"])
        hidden = [box for box in reached["obstacles"] if box["label"] == "hidden under observed environment"]
        self.assertTrue(hidden)
        task, [reach] = reached["task_object"], [box for box in reached["known_space"] if box["label"].startswith("task reach")]
        behind = [box for box in hidden if box["max"][1] - SG.DepthUncertaintyCm > board[0][1] - 1.0 and box["min"][0] < -16.0 and box["max"][0] > -22.0]
        self.assertTrue(behind, "the board's hidden part behind the can")
        for box in hidden:
            low, high = np.array(box["min"]) + SG.DepthUncertaintyCm, np.array(box["max"]) - SG.DepthUncertaintyCm
            self.assertTrue(np.all(low >= np.array(reach["min"]) - 1.0) and np.all(high <= np.array(reach["max"]) + 1.0), box)
            self.assertFalse(np.all(low < np.array(task["max"])) and np.all(high > np.array(task["min"])), ("inside the can's envelope", box))
            self.assertGreaterEqual(low[2], reached["support"]["TopZCm"] - 1.0)


class NoDeskSupportSidesTests(unittest.TestCase):
    """With no calibrated desk in view a level patch beside enough of the object's sides holds it (DepthGeometry
    SupportSides): what stands against the object no longer withholds it (the pill bottle on the bookshelf cabinet of
    2026-10-25, TestShelfCapture). Floating, held up by the arm, or on a slope it is still withheld."""

    def test_books_against_its_back_and_side_do_not_withhold_it(self):
        # Books standing against the can's back and its left side, from the cabinet top up: the cabinet top lies around
        # it only in front and on its right. The can's outline (MeasureBox Only, as the single call's) keeps the books'
        # faces out of its measurement, as the model's outline does: a box alone takes in the books touching it.
        books = [(np.array([-30.0, 42.0, CabinetTop]), np.array([-8.0, 50.0, CabinetTop + 20.0])),
                 (np.array([-30.0, 30.0, CabinetTop]), np.array([-22.0, 42.0, CabinetTop + 20.0]))]
        depth = T.TDC.RenderDepth(T.Camera, NoDeskWorkspace([CabinetCan, *books]), FloorZ=T.TDC.RoomFloorZ)
        alone = T.TDC.RenderDepth(T.Camera, NoDeskWorkspace([CabinetCan]), FloorZ=T.TDC.RoomFloorZ)
        outline = (depth == alone) & (alone != T.TDC.RenderDepth(T.Camera, NoDeskWorkspace(), FloorZ=T.TDC.RoomFloorZ)) & (depth > 0)
        surfaces = DG.WorkspaceSurfaces(None, T.TableZ, Automatic=True, Desk=False)
        box = T.TDC.CaptureBox(T.Camera, *CabinetCan)
        Measure = lambda **extra: DG.MeasureBox(depth, box, T.TDC.Intrinsics, T.Camera, T.TableZ, True, Surfaces=extra.pop("surfaces", surfaces),
                                                Only=outline, **extra)
        measured = {**Measure(), "label": "can"}
        self.assertTrue(measured["ok"], measured)
        self.assertEqual((measured["support_branch"], measured["automatic_surface"]), ("surface", True))
        self.assertAlmostEqual(measured["z"], CabinetTop, delta=0.3)
        self.assertAlmostEqual(measured["height_cm"], 8.0, delta=0.5)
        self.assertLess(np.hypot(measured["x"] + 19.0, measured["y"] - 39.0), 1.0, measured)
        support = measured["support_geometry"]
        sides = support["Evidence"]["sides"]
        self.assertGreaterEqual(len(sides), DG.SupportMinSides)
        self.assertNotIn("-x", sides)
        self.assertNotIn("+y", sides)
        # The support's extent takes in the footprint it holds.
        self.assertLessEqual(support["YRangeCm"][0], measured["y"] - measured["size_cm"][1] / 2 + 0.01)
        self.assertGreaterEqual(support["YRangeCm"][1], measured["y"] + measured["size_cm"][1] / 2 - 0.01)
        # Only the sides rule admits it: around all four sides it was withheld, as at a desk.
        with patch.object(DG, "SupportMinSides", 5):
            self.assertEqual(Measure()["error"], DG.NoDeskSupportError)
        # At a calibrated desk the rule is as before (the patch must lie around the footprint).
        self.assertFalse(Measure(surfaces=DG.WorkspaceSurfaces(None, T.TableZ, Automatic=True, Desk=True)).get("automatic_surface", False))
        # The books stay solids: against the can (its contact) and in the scene's obstacles.
        scene = SG.Build(depth, T.TDC.Intrinsics, T.Camera, measured, support)
        self.assertIsNotNone(scene["task_contact"])
        self.assertGreaterEqual(scene["task_contact"]["points"], SG.TouchMinPoints)
        tall = [box for box in scene["obstacles"] if box["max"][2] > CabinetTop + 12.0]
        self.assertTrue(tall, "the books above the can are obstacles")

    def test_a_floating_object_is_withheld(self):
        # The can 5 cm above the cabinet top, nothing under it: the cabinet top lies all around, but the can does not reach it.
        floating = (CabinetCan[0] + [0, 0, 5.0], CabinetCan[1] + [0, 0, 5.0])
        depth = T.TDC.RenderDepth(T.Camera, NoDeskWorkspace([floating]), FloorZ=T.TDC.RoomFloorZ)
        measured = MeasuredNoDesk(depth, floating)
        self.assertFalse(measured["ok"], measured)
        self.assertEqual(measured["error"], DG.NoDeskSupportError)

    def test_an_object_held_up_by_the_arm_is_withheld(self):
        # The can resting on a block the arm's mask removes (the gripper under it), 5 cm above the cabinet top: the arm
        # is no support, and the cabinet top below does not hold it.
        lifted = (CabinetCan[0] + [0, 0, 5.0], CabinetCan[1] + [0, 0, 5.0])
        arm = (np.array([-25.0, 33.0, CabinetTop + 2.0]), np.array([-13.0, 45.0, CabinetTop + 5.0]))
        depth = T.TDC.RenderDepth(T.Camera, NoDeskWorkspace([lifted, arm]), FloorZ=T.TDC.RoomFloorZ)
        low, high = arm[0] - 0.5, arm[1] + 0.5
        exclude = lambda points: np.all((points >= low) & (points <= high), axis=1)
        surfaces = DG.WorkspaceSurfaces(None, T.TableZ, Automatic=True, Desk=False)
        measured = DG.MeasureBox(depth, T.TDC.CaptureBox(T.Camera, *lifted), T.TDC.Intrinsics, T.Camera, T.TableZ, True,
                                 Surfaces=surfaces, Exclude=exclude)
        self.assertFalse(measured["ok"], measured)
        self.assertEqual(measured["error"], DG.NoDeskSupportError)
        # Unmasked, the block is a level top under the can, and the can is measured on it (the mask is what refuses it).
        unmasked = DG.MeasureBox(depth, T.TDC.CaptureBox(T.Camera, *lifted), T.TDC.Intrinsics, T.Camera, T.TableZ, True, Surfaces=surfaces)
        self.assertTrue(unmasked["ok"], unmasked)
        self.assertAlmostEqual(unmasked["z"], CabinetTop + 5.0, delta=0.3)

    def test_an_object_whose_base_a_box_in_front_hides_is_not_measured_on_the_box(self):
        # A box 4 cm tall against the can's front, wider than it: the can's lower half is hidden, so its base height
        # cannot be told. The box's top lies only in front of it: it is never the can's support.
        for gap in (0.0, 0.5):
            with self.subTest(gap=gap):
                front = (np.array([-25.0, 30.0, CabinetTop]), np.array([-13.0, 36.0 - gap, CabinetTop + 4.0]))
                depth = T.TDC.RenderDepth(T.Camera, NoDeskWorkspace([CabinetCan, front]), FloorZ=T.TDC.RoomFloorZ)
                measured = MeasuredNoDesk(depth, CabinetCan)
                self.assertFalse(measured["ok"] and abs(measured["z"] - CabinetTop) > 0.5, measured)

    def test_an_object_on_a_slope_is_withheld(self):
        # The can on a top sloping 20 deg, nothing level around it.
        slope, pivot = float(np.tan(np.radians(20.0))), 39.0
        can = (np.array([-22.0, 36.0, CabinetTop + slope * 3.0]), np.array([-16.0, 42.0, CabinetTop + slope * 3.0 + 8.0]))
        body = (np.array([-40.0, 28.0, T.TDC.RoomFloorZ]), np.array([-3.0, 62.0, CabinetTop - slope * 11 - 0.2]))
        depth = RenderTilted(T.Camera, [T.TDC.Workspace()[1], body, can], CabinetTop, slope, pivot)
        measured = MeasuredNoDesk(depth, can)
        self.assertFalse(measured["ok"], measured)
        self.assertFalse(measured.get("automatic_surface", False), measured)
        self.assertNotEqual(measured["support_branch"], "surface", measured)

    def test_the_sides_themselves(self):
        rng = np.random.default_rng(3)
        level = lambda x0, x1, y0, y1, n=3000: np.column_stack([rng.uniform(x0, x1, n), rng.uniform(y0, y1, n), np.full(n, 4.0)])
        centre, half = np.array([0.0, 40.0]), np.array([2.0, 2.0])
        self.assertEqual(DG.SupportSides(np.vstack([level(-6, 6, 34, 38), level(2, 6, 38, 42)]), centre, half), ["+x", "-y"])
        self.assertEqual(DG.SupportSides(level(-6, 6, 34, 46), centre, half), ["-x", "+x", "-y", "+y"])
        # A few points at a corner are no side; nor a band covering under half the side's length.
        self.assertEqual(DG.SupportSides(level(2.2, 3.0, 37.2, 38.0, 30), centre, half), [])
        self.assertEqual(DG.SupportSides(level(2.0, 4.0, 38.0, 39.5, 400), centre, half), [])
        self.assertEqual(DG.SupportSides(level(4.5, 6.0, 38.0, 42.0, 400), centre, half), [], "more than 2 cm off its side")
        # The line a wall (or the object's own side) cuts at the surface's height is no area; the box's own points count
        # only clear of the footprint.
        self.assertEqual(DG.SupportSides(level(3.0, 3.2, 38.0, 42.0, 400), centre, half), [])
        edge = level(1.6, 2.4, 38.0, 42.0, 400)
        self.assertEqual(DG.SupportSides(np.empty((0, 3)), centre, half, Inside=edge), [])
        self.assertEqual(DG.SupportSides(np.empty((0, 3)), centre, half, Inside=level(2.0, 4.0, 38.0, 42.0, 800)), ["+x"])


# The recorded cabinet frame of 2026-10-06 15:01:01Z (RecordedCaptures: not on disk, the tests fail, RECORDING MISSING),
# with the calibration it was taken with (its tracked copy: a new calibration does not skip them).
import RecordedCaptures as RC  # noqa: E402
CabinetDay = RC.Day("2026-10-06")
CabinetFrame = CabinetDay / "recording" / "depth" / "c1e55020492ffe59b3a27f09a5f24af9e3b1491e.png"
BottleBox = [851 / 1280, 191 / 720, 922 / 1280, 282 / 720]
# The pill bottle as investigation 3 measured it (corrected pose): axis, body and cap radii, base, shoulder and top (cm).
BottleAxis, BottleRadii, BottleHeights = np.array([-24.97, 40.69]), (2.24, 1.85), (4.2, 10.3, 12.1)
# The replies written for the app's planner (TestCabinetGrasp.mjs, which writes its own copy: SO101_CABINET_SCENES).
CabinetScenes = Path(__import__("os").environ.get("SO101_CABINET_SCENES") or Path(__file__).resolve().parent / "TestArtifacts" / "CabinetScenes")


def Plain(value):
    """JSON for numpy scalars (a reply as the bridge sends it)."""
    return value.item() if hasattr(value, "item") else str(value)


def MovedClear(depth, model, camera, shift_cm=3.0):
    """COUNTERFACTUAL: the 15:01 depth with the pill bottle slid shift_cm toward the arm's pan axis, clear of the books
    (as the refusal asks). Its pixels and its projector shadow are replaced by what stands behind (the cabinet top's
    plane around it, the books' front as a vertical plane), the bottle ray cast at its new place, its shadow moved with
    it in the image. Returns the depth and its detection box (the recorded box's margins about the moved bottle)."""
    import cv2
    raw = depth.astype(np.float64) / 10.0
    valid = (raw > 0) & (raw <= DG.MaxDepthCm)
    rays = np.concatenate([DG.RayMap(camera).astype(np.float64), np.ones(raw.shape + (1,))], axis=-1)
    world = DG.CameraToDisplay((rays * raw[..., None]).reshape(-1, 3), model).reshape(raw.shape + (3,))
    centre = DG.CameraToDisplay(np.zeros(3), model)[0]
    direction = DG.CameraToDisplay(rays.reshape(-1, 3), model).reshape(rays.shape) - centre
    rho = np.linalg.norm(world[..., :2] - BottleAxis, axis=-1)
    ring = world[valid & (rho > 3.2) & (rho < 9.0) & (np.abs(world[..., 2] - 4.3) < 0.8)]
    top = np.linalg.lstsq(np.column_stack([ring[:, :2], np.ones(len(ring))]), ring[:, 2], rcond=None)[0]
    books = world[valid & (rho > 2.9) & (rho < 8.0) & (world[..., 2] > 6.0) & (world[..., 2] < 16.0) & (world[..., 1] > 41.0)][:, :2]
    normal = np.linalg.svd(books - books.mean(axis=0))[2][1]
    normal = normal if normal @ (books.mean(axis=0) - BottleAxis) > 0 else -normal
    front = float(books.mean(axis=0) @ normal + np.percentile((books - books.mean(axis=0)) @ normal, 10))
    u0, v0, u1, v1 = DG.BoxToNative(BottleBox, raw.shape[1], raw.shape[0], False)
    region = np.zeros(raw.shape, dtype=bool)
    region[max(v0 - 30, 0):v1 + 30, max(u0 - 30, 0):u1 + 30] = True
    bottle = valid & region & (rho <= 3.5) & (world[..., 2] > world[..., :2] @ top[:2] + top[2] + 0.3) & (world[..., 2] < BottleHeights[2] + 1.0)
    shadow = cv2.dilate(bottle.astype(np.uint8), np.ones((17, 17), np.uint8)).astype(bool) & region & ~valid
    erase = (cv2.dilate(bottle.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool) & region) | shadow
    with np.errstate(divide="ignore", invalid="ignore"):
        on_top = (top[:2] @ centre[:2] + top[2] - centre[2]) / (direction[..., 2] - direction[..., :2] @ top[:2])
        on_books = (front - centre[:2] @ normal) / (direction[..., :2] @ normal)
    in_front = (centre[:2] + on_top[..., None] * direction[..., :2]) @ normal < front
    background = np.where((on_top > 0) & in_front, on_top, on_books)
    out = depth.astype(np.float64).copy()
    out[erase] = background[erase] * 10.0
    way = np.array([-13.3, 0.0]) - BottleAxis
    axis = BottleAxis + shift_cm * way / np.linalg.norm(way)
    lift = float(top[:2] @ axis + top[2]) - BottleHeights[0]

    def cylinder(radius, z0, z1):
        o = centre[:2] - axis
        a = np.sum(direction[..., :2] ** 2, axis=-1)
        b = 2 * direction[..., :2] @ o
        disc = b * b - 4 * a * (o @ o - radius * radius)
        hit = np.full(a.shape, np.inf)
        for sign in (-1.0, 1.0):
            t = (-b + sign * np.sqrt(np.maximum(disc, 0))) / (2 * a)
            z = centre[2] + t * direction[..., 2]
            hit = np.where((disc >= 0) & (t > 0) & (z >= z0) & (z <= z1) & (t < hit), t, hit)
        t = (z1 - centre[2]) / direction[..., 2]
        cap = np.sum((centre[:2] + t[..., None] * direction[..., :2] - axis) ** 2, axis=-1) <= radius * radius
        return np.where((t > 0) & cap & (t < hit), t, hit)
    base, shoulder, crown = (height + lift for height in BottleHeights)
    t = np.minimum(cylinder(BottleRadii[0], base, shoulder), cylinder(BottleRadii[1], shoulder, crown))
    new = np.isfinite(t) & ((out <= 0) | (t * 10.0 < out))
    out[new] = t[new] * 10.0
    step = (DG.Project(DG.DisplayToCamera(np.array([[*axis, base + 4.0]]), model), camera)[0]
            - DG.Project(DG.DisplayToCamera(np.array([[*BottleAxis, BottleHeights[0] + 4.0]]), model), camera)[0])
    rows, cols = np.nonzero(shadow)
    rows, cols = rows + int(round(step[1])), cols + int(round(step[0]))
    keep = (rows >= 0) & (rows < raw.shape[0]) & (cols >= 0) & (cols < raw.shape[1])
    moved = np.zeros(raw.shape, dtype=bool)
    moved[rows[keep], cols[keep]] = True
    out[moved & ~new] = 0
    size = np.tile([raw.shape[1], raw.shape[0]], 2)
    box = (np.array(BottleBox) * size + np.tile(step, 2)) / size
    return np.clip(np.round(out), 0, 65535).astype(depth.dtype), box.tolist()


class CabinetSceneTests(unittest.TestCase):
    """The 15:01 frame: the camera checked against the gripper tip, the pill bottle measured on the cabinet top, its scene."""

    @classmethod
    def setUpClass(cls):
        import cv2
        import CameraDrift as CD
        import RobotReference as RR
        from TestRobotReference import ArmFromRaw, Home1501
        import bridge_server as Bridge
        RC.Need(CabinetFrame, CabinetDay / "080101_173_vision.jpg", What="the 2026-10-06 15:01 cabinet capture")
        cls.model = RC.CalibrationOf20261002()
        cls.camera = {key: cls.model[key] for key in ("fx", "fy", "cx", "cy", "dist")} | {"width": 1280, "height": 720}
        cls.depth = cv2.imread(str(CabinetFrame), cv2.IMREAD_UNCHANGED)
        cls.image = cv2.imread(str(CabinetDay / "080101_173_vision.jpg"))
        controller = Bridge.BridgeController()
        for name, raw in Home1501.items():
            controller.servos[name].present_raw, controller.servos[name].gui_angle_raw = int(raw), None
        cls.angles = controller._current_ik_angles()
        cls.arm = {**ArmFromRaw(Home1501, RR.ReferenceRaw(cls.model)), "link_boxes": controller.ArmLinkBoxes(cls.angles)}
        cls.check = CD.CheckCapture(cls.depth, cls.camera, cls.model, 516698, cls.image, cls.arm)

    def locate(self, depth=None):
        detections = [{"label": "pill bottle", "box": BottleBox, "u": (BottleBox[0] + BottleBox[2]) / 2 * 1280, "v": BottleBox[3] * 720}]
        registry = Registry()
        with patch.object(Locator, "SceneSafety", registry), \
                patch.object(T.DeclaredSurfaces, "Load", return_value=[]), \
                patch.object(T.calibration, "camera_model", return_value=self.model), \
                patch.object(T.calibration, "status", return_value={"calibrated": True, "rms_cm": 0.75}), \
                patch.object(T.calibration, "pixel_to_plane", return_value={"ok": False}), \
                patch.object(T.CD, "CheckCapture", return_value=json.loads(json.dumps(self.check))), \
                patch.object(Locator, "_pick_model", return_value="recorded"), \
                patch.object(Locator, "_detect_once", return_value=detections), \
                patch.object(Locator, "_refine_detection", return_value=None), \
                patch.object(Locator, "_Record"):
            return Locator.locate_objects(T.TDC.Encode(self.image), depth={"depth_mm": self.depth if depth is None else depth, "intrinsics": self.camera,
                                                                          "mirrored": False, "seq": 516698, "time": time.time()},
                                          mirrored=False, arm=self.arm, task_object={"phrase": "pill bottle", "label": "pill bottle"})

    def world(self, scene):
        import bridge_server as Bridge
        import BasketCollision as Collision
        frame = json.loads((Path(__file__).resolve().parent / "WorkspaceFrame.json").read_text(encoding="utf-8"))
        frame.update(SceneObstacles=scene["obstacles"], SceneFree=[*scene["free_space"], *scene.get("known_space", []), scene["task_object"]])
        return Collision.CollisionWorld(Bridge.BridgeController().arm_geometry, frame)

    def test_an_assumed_camera_is_blocked_where_the_rim_says_it_moved_and_runs_where_it_did_not(self):
        # The review's IH1 point on a real capture, not a mocked tripwire: the 15:01 frame as a camera drooped 2 deg on
        # the robot would have seen it, the tip not judged (the arm moving), is blocked by the basket rim; the frame as
        # recorded, judged the same way, still runs (Aaran's decision 4), its measurements unverified.
        import math
        import cv2
        import CameraDrift as CD
        from TestCartMove import TurnedFrame
        moving = {"ok": False, "reason": "arm moving"}
        recorded = CD.CheckCapture(self.depth, self.camera, self.model, 516698, self.image, moving)
        self.assertEqual((recorded["ok"], recorded["robot_check"], recorded["rim_check"]), (True, "assumed", "ok"), recorded)
        depth, image = TurnedFrame(self.depth, self.image, self.camera, cv2.Rodrigues(np.array([math.radians(2.0), 0.0, 0.0]))[0])
        # Since 2026-10-25 the rim decides at the camera gate itself (RobotReference.Check: either witness off blocks), so the
        # turned frame never reaches the model; it used to pass as assumed and trip the scene tripwire after the model call.
        turned = CD.CheckCapture(depth, self.camera, self.model, 516698, image, moving)
        self.assertEqual((turned["ok"], turned["error_code"], turned["robot_check"], turned["rim_check"]), (False, "camera_moved", "moved", "moved"), turned)
        self.assertIn("basket's front rim", turned["error"])
        saved = self.check
        try:
            self.check = recorded
            result = self.locate()
            self.assertTrue(result["ok"], result)
            self.assertIn("scene", result)
            self.assertEqual([item["scene_checked"] for item in result["objects"]], [False])
            self.check = turned
            gated = self.locate(depth)
            self.assertEqual((gated["ok"], gated["error_code"]), (False, "camera_moved"), gated)
            # The tripwire behind it still holds a capture that reads so and passed the gate (an older gate's result).
            self.check = {**{Key: Value for Key, Value in turned.items() if Key not in ("error_code", "error")}, "ok": True, "robot_check": "assumed"}
            blocked = self.locate(depth)
            self.assertEqual((blocked["ok"], blocked["error_code"]), (False, "camera_moved"), blocked)
            self.assertIn("basket rim", blocked["camera_check"]["reason"])
            with patch.object(Locator, "TRIPWIRE_ASSUMED_RIM", False):
                self.assertNotEqual(self.locate(depth).get("error_code"), "camera_moved", "without the rim nothing trips here")
        finally:
            self.check = saved

    def test_the_pill_bottle_is_measured_on_the_cabinet_top(self):
        self.assertEqual((self.check["ok"], self.check["alignment_source"], self.check["robot_check"]), (True, "robot", "verified"), self.check)
        result = self.locate()
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["no_desk"], {"robot_check": "verified", "supports": "measured"})
        [bottle] = result["objects"]
        # The design's reading of the cabinet: X -25.0, Y 40.7, base Z 4.2, top 12.1 (NoDeskDesign.md section 7).
        self.assertLess(np.hypot(bottle["x"] + 25.0, bottle["y"] - 40.7), 1.0, bottle)
        self.assertAlmostEqual(bottle["z"], 4.2, delta=0.5)
        self.assertAlmostEqual(bottle["top_z"], 12.1, delta=0.5)
        self.assertEqual((bottle["support_branch"], bottle["support_geometry"]["Source"]), ("surface", "depth"))
        self.assertIn("scene", result)
        print(f"  15:01 pill bottle: X {bottle['x']} Y {bottle['y']} base {bottle['z']} top {bottle['top_z']} on '{bottle['support_surface']}'")

    def test_the_robot_is_not_in_its_scene_and_the_arm_leaves_its_place(self):
        scene = self.locate()["scene"]
        world = self.world(scene)
        # The collar's pan revolution and every link at the capture pose are clear of the scene, and the gripper's own
        # place is known: before, 238 of the 1419 boxes were the robot and every route was refused at the collar.
        self.assertIsNone(world.MountReason)
        self.assertIsNone(world.PoseReason(self.angles))
        start = np.array([self.angles[name] for name in __import__("BasketCollision").Names])
        self.assertIsNone(world.StartRelief(start))
        # As 74097be built it: the arm masked only 3 cm about its centre lines, the cart and mast kept.
        arm = self.arm
        self.arm = {key: value for key, value in arm.items() if key != "link_boxes"}
        try:
            with patch.object(SG, "RobotCells", lambda cells, *a, **k: np.zeros(len(cells), dtype=bool)):
                with_robot = self.locate()["scene"]
        finally:
            self.arm = arm
        self.assertGreater(len(with_robot["obstacles"]), len(scene["obstacles"]) + 150)
        self.assertIn("Shoulder mounting collar intersects", self.world(with_robot).MountReason or "")
        print(f"  15:01 scene: {len(scene['obstacles'])} obstacle boxes ({len(with_robot['obstacles'])} with the cart and mast in it)")

    def test_the_books_stand_against_the_bottle_and_the_cabinet_top_tilts(self):
        # As placed at 15:01 the bottle stood 0.7-1.5 cm in front of the books, and its measurement took in some of their
        # spines (its footprint reaches X -29.3, the bottle -27.2): the rest of them stands against it (TaskContact), so
        # its grasp is refused with "move it 2 to 3 cm away". The cabinet top tilts 4 deg in the arm's frame (the cart
        # leans): its plane, not a level band, holds it beside the bottle (SceneSupportTests).
        scene = self.locate()["scene"]
        contact = scene["task_contact"]
        self.assertGreaterEqual(contact["points"], 100, contact)
        self.assertTrue(-31.0 <= contact["min"][0] and contact["max"][0] <= -25.0 and 42.5 <= contact["min"][1] and contact["max"][1] <= 44.5, contact)
        self.assertTrue(10.0 <= contact["min"][2] and contact["max"][2] <= 14.0, contact)
        self.assertAlmostEqual(scene["support_tilt_deg"], 4.0, delta=1.0)
        print(f"  15:01 the books against the bottle: {contact['points']} points at X {contact['min'][0]}..{contact['max'][0]}, "
              f"Y {contact['min'][1]}..{contact['max'][1]}, Z {contact['min'][2]}..{contact['max'][2]}; cabinet top {scene['support_tilt_deg']} deg")

    def test_moved_clear_of_the_books_nothing_stands_against_the_bottle(self):
        # The bottle slid 3 cm toward the arm (the counterfactual Aaran's move would make): measured on the cabinet top,
        # nothing against it, and none of its own rim (the points its measurement leaves out where the depth jumps,
        # 0.0-0.2 cm outside its footprint) is an obstacle beside it. The replies, with the task's reach off and on, are
        # written for the app's planner (TestCabinetGrasp.mjs), with the recorded one.
        CabinetScenes.mkdir(parents=True, exist_ok=True)
        (CabinetScenes / "recorded.json").write_text(json.dumps(self.locate(), default=Plain), encoding="utf-8")
        depth, box = MovedClear(self.depth, self.model, self.camera, 3.0)
        global BottleBox
        recorded, BottleBox = BottleBox, box
        try:
            for reach in (False, True):
                with patch.object(SG, "AssumeTaskReachFree", reach):
                    result = self.locate(depth)
                (CabinetScenes / f"moved_3cm{'_reach' if reach else ''}.json").write_text(json.dumps(result, default=Plain), encoding="utf-8")
        finally:
            BottleBox = recorded
        self.assertTrue(result["ok"], result)
        [bottle] = result["objects"]
        scene = result["scene"]
        self.assertLess(np.hypot(bottle["x"] + 24.1, bottle["y"] - 37.5), 1.0, bottle)
        self.assertAlmostEqual(bottle["z"], 4.0, delta=0.4)
        self.assertIsNone(scene["task_contact"])
        low, high = np.array(bottle["bounds_cm"]["min"]), np.array(bottle["bounds_cm"]["max"])
        for cell in scene["obstacles"]:
            if cell["label"] != "observed environment":
                continue
            cell_low, cell_high = np.array(cell["min"]) + SG.DepthUncertaintyCm, np.array(cell["max"]) - SG.DepthUncertaintyCm
            beside = np.all(cell_low[:2] < high[:2] + SG.TargetRimCm) and np.all(cell_high[:2] > low[:2] - SG.TargetRimCm)
            self.assertFalse(beside and cell_high[2] > bottle["z"] + SG.SupportBandCm and cell_low[2] < bottle["top_z"], ("against the bottle", cell))
        print(f"  15:01 moved 3 cm: X {bottle['x']} Y {bottle['y']} base {bottle['z']} top {bottle['top_z']}, nothing against it")

    def test_under_the_bottle_there_is_only_its_support(self):
        # The cabinet top's noise under the bottle's base (12 cells below its 0.35 cm band before) is its support,
        # capped at its top: a synthetic render has no such points, so this is the real frame's.
        scene = self.locate()["scene"]
        low, high = np.array(scene["task_object"]["min"]), np.array(scene["task_object"]["max"])
        top = scene["support"]["TopZCm"]
        for box in scene["obstacles"]:
            cell_low, cell_high = np.array(box["min"]) + SG.DepthUncertaintyCm, np.array(box["max"]) - SG.DepthUncertaintyCm
            if np.all(cell_low[:2] >= low[:2]) and np.all(cell_high[:2] <= high[:2]) and cell_high[2] <= top + 0.35:
                self.assertEqual(box["label"], "observed support", box)


def ExportFixtures(directory):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    nearer_can = tuple(point + [0, -3, 0] for point in T.Can)
    nearer_platform = (T.Platform[0] + [0, -3, 0], T.Platform[1])
    for name, workspace, item, baseline in (("measured-raised-platform", T.TDC.Workspace([nearer_platform, nearer_can]), nearer_can, T.TDC.Workspace()),
                                             ("measured-low-shelf", T.ShelfWorkspace([T.ShelfCans[1]]), T.ShelfCans[1], T.ShelfWorkspace())):
        depth = T.TDC.RenderDepth(T.Camera, workspace, FloorZ=T.TDC.RoomFloorZ)
        result, _ = Locate(depth, item, baseline)
        assert result.get("scene"), result
        (directory / f"{name}.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    camera, can, solids = ShelfScene()
    depth = T.TDC.RenderDepth(camera, T.TDC.Workspace(solids), FloorZ=T.TDC.RoomFloorZ)
    result, _ = Locate(depth, can, T.TDC.Workspace(), camera)
    assert result.get("scene"), result
    (directory / "measured-shelf-underside.json").write_text(json.dumps(result, indent=2), encoding="utf-8")


if __name__ == "__main__":
    unittest.main(verbosity=2)
