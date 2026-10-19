"""Offline scene approval, swept CAD obstacles and carried-object regressions."""
import copy
import unittest
from unittest.mock import patch

import numpy as np

import BasketCollision as Collision
import CadKinematics as Cad
import RouteCheckPool
import SceneSafety as Scene
import bridge_server as Bridge
from TestSafety import MakeController
from TestBasketCollision import StartValues, GoalValues, Place


MODEL = {"type": "pinhole_depth_v1", "rvec": [0, 0, 0], "tvec": [0, 0, 0], "fitted_at": "test"}
FAR = {"label": "shelf", "min": [60, 60, 60], "max": [70, 70, 62]}


def snapshot(obstacles=None):
    return Scene.Register({"frame": "display_cm", "capture_seq": 1, "obstacles": obstacles or [FAR],
                           "free_space": [{"min": [-80, -20, -30], "max": [55, 55, 55]}]}, MODEL)


class SceneTests(unittest.TestCase):
    def setUp(self):
        Scene.Invalidate()
        self.addCleanup(Scene.Invalidate)

    def test_registration_is_immutable_and_stale_approvals_are_revoked(self):
        first = snapshot()
        first["obstacles"][0]["min"][0] = -100
        self.assertEqual(Scene.GetValid(first["id"], MODEL)["obstacles"][0]["min"][0], 60)
        second = snapshot()
        self.assertEqual(first["revision"], second["revision"])
        with self.assertRaisesRegex(ValueError, "changed"):
            Scene.GetValid(first["id"], MODEL)
        # A scene ages on the monotonic clock once registered: a wall-clock step (a time sync) does not expire it.
        with patch.object(Scene.time, "time", return_value=second["captured_at"] + Scene.MaximumAgeSeconds + 1):
            self.assertEqual(Scene.GetValid(second["id"], MODEL)["id"], second["id"])
        Registered = Scene._clock[second["id"]][0]
        with patch.object(Scene.time, "monotonic", return_value=Registered + Scene.MaximumAgeSeconds + 1):
            with self.assertRaisesRegex(ValueError, "stale"):
                Scene.GetValid(second["id"], MODEL)
            self.assertFalse(Scene.IsCurrent(second["id"]))
            self.assertLess(Scene.RemainingSeconds(second["id"]), 0)
        self.assertGreater(Scene.RemainingSeconds(second["id"]), Scene.MaximumAgeSeconds - 60)
        self.assertIsNone(Scene.RemainingSeconds(first["id"]))
        with self.assertRaisesRegex(ValueError, "alignment changed"):
            Scene.GetValid(second["id"], {**MODEL, "tvec": [0, 1, 0]})

    def test_no_obstacles_bad_coordinates_or_old_frame_cannot_be_registered(self):
        for obstacles in ([], [{"min": [0, 0, 0], "max": [1, 1, float("nan")]}],
                          [{"min": [0, 0, 0], "max": [1, 0, 1]}]):
            with self.assertRaises(ValueError):
                Scene.Register({"frame": "display_cm", "obstacles": obstacles}, MODEL)
        with self.assertRaisesRegex(ValueError, "stale"):
            Scene.Register({"frame": "display_cm", "obstacles": [FAR]}, MODEL, capture_time=0)

    def test_free_space_bounds_are_validated_and_change_revision(self):
        first = snapshot()
        changed = copy.deepcopy(first)
        changed["free_space"][0]["max"][0] -= 1
        second = Scene.Register(changed, MODEL)
        self.assertNotEqual(first["revision"], second["revision"])
        for free in ({}, [{"min": [0, 0, 0], "max": [1, 1, float("nan")]}]):
            with self.assertRaises(ValueError):
                Scene.Register({"frame": "display_cm", "obstacles": [FAR], "free_space": free}, MODEL)

    def test_known_space_is_validated_changes_revision_and_counts_as_known(self):
        # The scene's known space (SceneGeometry.Build: the arm where it stood, the task's reach) is checked like
        # free space, is part of the revision, and the bridge's coverage counts it as known.
        known = {"min": [-5, 10, 0], "max": [5, 20, 10], "label": "the arm at the capture (gripper_link__1)"}
        first = Scene.Register({"frame": "display_cm", "capture_seq": 1, "obstacles": [FAR], "free_space": [], "known_space": [known]}, MODEL)
        moved = Scene.Register({"frame": "display_cm", "capture_seq": 1, "obstacles": [FAR], "free_space": [],
                                "known_space": [{**known, "max": [5, 21, 10]}]}, MODEL)
        self.assertNotEqual(first["revision"], moved["revision"])
        controller, _ = MakeController()
        with patch.object(Bridge.calibration, "camera_model", return_value=MODEL):
            controller.BindScene(moved["id"], True)
        self.assertIn([-5, 10, 0], [box["min"] for box in controller.CollisionFrame()["SceneFree"]])
        self.assertEqual(snapshot()["revision"], snapshot()["revision"], "a scene without known space hashes as before")
        for bad in ({}, [{"min": [0, 0, 0], "max": [1, 1, float("nan")]}]):
            with self.assertRaises(ValueError):
                Scene.Register({"frame": "display_cm", "obstacles": [FAR], "known_space": bad}, MODEL)

    def test_the_task_contact_is_validated_and_part_of_the_revision(self):
        # SceneGeometry.TaskContact: the observed solid against the task object, or None; the app refuses its grasp on it.
        scene = {"frame": "display_cm", "capture_seq": 1, "obstacles": [FAR], "free_space": []}
        contact = {"points": 334, "min": [-29.9, 43.08, 10.32], "max": [-26.2, 43.58, 13.43]}
        clear = Scene.Register({**scene, "task_contact": None}, MODEL)
        against = Scene.Register({**scene, "task_contact": contact}, MODEL)
        self.assertIsNone(clear["task_contact"])
        self.assertEqual(against["task_contact"], contact)
        self.assertNotEqual(clear["revision"], against["revision"])
        self.assertNotEqual(Scene.Register({**scene, "task_contact": {**contact, "points": 25}}, MODEL)["revision"], against["revision"])
        for bad in ({**contact, "points": 0}, {**contact, "max": [-29.9, 43.58, 13.43]}, {"points": 3}):
            with self.assertRaises(ValueError):
                Scene.Register({**scene, "task_contact": bad}, MODEL)

    def test_scene_required_blocks_both_command_paths_before_motor_writes(self):
        controller, bus = MakeController()
        for command in ({"type": "send_ik", "scene_required": True},
                        {"type": "move_servo", "name": "gripper", "raw": 1000, "scene_required": True}):
            ok, message = controller.handle_command(command)
            self.assertFalse(ok)
            self.assertIn("measured scene", message)
        self.assertFalse(bus.writes)

    def test_full_arm_scene_collision_reaches_workers_and_cannot_use_contact_relief(self):
        controller, bus = MakeController()
        angles = dict(zip(Collision.Names, StartValues))
        Place(controller, bus, angles)
        frame = controller.CollisionFrame()
        world = controller.CollisionWorld()
        self.assertIsNone(world.PoseReason(angles))
        name, link, center, half, _ = next(box for box in world.Boxes if box[1] == "upper_arm_link")
        links, _ = Cad.Forward(controller.arm_geometry, angles)
        position = links[link][:3, :3] @ center + links[link][:3, 3]
        base = frame["RobotFrame"]["ArmBasePositionCm"]
        display = np.array([-position[0] - base["x"], position[1] + base["y"], position[2] + base["z"]])
        obstacle = {"label": "shelf side wall", "min": (display - 0.5).tolist(), "max": (display + 0.5).tolist()}
        scene = snapshot([obstacle])
        with patch.object(Bridge.calibration, "camera_model", return_value=MODEL):
            controller.BindScene(scene["id"], True)
        frame = controller.CollisionFrame()
        world = controller.CollisionWorld()
        self.assertIn("shelf side wall", world.PoseReason(angles))
        values = np.array(StartValues)
        self.assertIn("shelf side wall", world.IntervalReason(values, values, Inset={i: 20 for i in range(len(world.Boxes))}))
        answer = RouteCheckPool.Check(controller.arm_geometry, frame, values, values, 32, None, (0, 0))
        self.assertIn("shelf side wall", answer)

    def test_carried_object_is_a_rigid_box_on_the_gripper(self):
        controller, _ = MakeController()
        frame = controller.CollisionFrame()
        frame["SceneCarried"] = {"bounds": [[-2, -2, -3], [2, 2, 8]]}
        plain = Collision.CollisionWorld(controller.arm_geometry, controller.CollisionFrame())
        loaded = Collision.CollisionWorld(controller.arm_geometry, frame)
        self.assertEqual(len(loaded.Boxes), len(plain.Boxes) + 1)
        self.assertEqual(loaded.Boxes[-1][0:2], ("held object", "gripper_link"))
        self.assertTrue(np.all(loaded.SplitWeights >= plain.SplitWeights))

    def test_observed_union_does_not_fill_unseen_gaps(self):
        volume = np.array([[0., 0, 0], [3., 3, 3]])
        left = np.array([[0., 0, 0], [1., 3, 3]])
        right = np.array([[2., 0, 0], [3., 3, 3]])
        self.assertFalse(Scene.Covered(volume, [left, right]))
        self.assertTrue(Scene.Covered(volume, [left, right, np.array([[1., 0, 0], [2., 3, 3]])]))

    def test_delayed_capture_cannot_replace_newer_scene(self):
        old = Scene.Invalidate()
        new = Scene.Invalidate()
        payload = {"frame": "display_cm", "capture_seq": 2, "obstacles": [FAR]}
        accepted = Scene.Register(payload, MODEL, generation=new)
        with self.assertRaisesRegex(ValueError, "newer capture"):
            Scene.Register(payload, MODEL, generation=old)
        self.assertTrue(Scene.IsCurrent(accepted["id"]))

    def test_unknown_depth_refuses_even_a_clear_target_pose(self):
        controller, bus = MakeController()
        angles = dict(zip(Collision.Names, StartValues))
        frame = controller.CollisionFrame()
        frame["SceneFree"] = []
        self.assertIn("not observed", Collision.CollisionWorld(controller.arm_geometry, frame).PoseReason(angles))

    def test_bound_scene_dispatch_and_invalidation_use_fake_motors(self):
        controller, bus = MakeController()
        Place(controller, bus, dict(zip(Collision.Names, StartValues)))
        scene = snapshot()
        angles = controller._current_ik_angles()
        angles["shoulder_pan"] += 1
        tip = Cad.Positions(controller.arm_geometry, angles)[-1]
        target = {**tip, "wrist_pitch": Bridge.wrist_tip_pitch_degrees(controller.arm_geometry, angles)}
        with patch.object(Bridge.calibration, "camera_model", return_value=MODEL):
            ok, message = controller.send_ik(Target=target, SceneId=scene["id"], SceneRequired=True)
        self.assertTrue(ok, message)
        self.assertEqual(controller.ActiveScene["id"], scene["id"])
        self.assertIsNotNone(controller.MotionRoute)
        Scene.Invalidate()
        controller.CheckMotionRoute()
        self.assertIsNone(controller.MotionRoute)
        # Stopped and holding where it is, no latched fault and no coast (2026-10-07 audit H1).
        self.assertIsNone(controller.MotionFault)
        self.assertIn("scene changed", controller.status)
        self.assertTrue(controller.torque_enabled and bus.torque_enabled)
        self.assertIsNone(controller.ActiveScene)


if __name__ == "__main__":
    unittest.main(verbosity=2)
