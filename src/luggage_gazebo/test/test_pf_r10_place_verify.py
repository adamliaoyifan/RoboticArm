#!/usr/bin/env python3
"""PF-R10 g3: closed-loop Gazebo placement verify and retry exhaustion."""

import importlib.util
import math
import os
import random
import threading
import unittest

from geometry_msgs.msg import Pose, Quaternion, TransformStamped
from tf2_msgs.msg import TFMessage

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG_ROOT = os.path.normpath(os.path.join(_HERE, ".."))

_spec = importlib.util.spec_from_file_location(
    "pickup_box_spawner_node",
    os.path.join(_PKG_ROOT, "scripts", "pickup_box_spawner_node.py"))
_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)


class _Logger(object):
    def error(self, *_a, **_k):
        pass

    def warning(self, *_a, **_k):
        pass

    def info(self, *_a, **_k):
        pass


class _Pub(object):
    def __init__(self):
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


def _pose(x=0.0, y=0.0, z=1.0, roll=0.0, pitch=0.0, yaw=0.0):
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.position.z = float(z)
    pose.orientation = _mod._quaternion_from_rpy(roll, pitch, yaw)
    return pose


def _bare_spawner():
    node = object.__new__(_mod.PickupBoxSpawner)
    node._place_persist_sec = 0.0
    node._replace_settle_sec = 0.0
    node._place_read_timeout_sec = 0.0
    node._place_max_attempts = 4
    node._place_xy_tol_m = 0.02
    node._place_z_tol_m = 0.03
    node._place_tilt_tol_rad = 0.087
    node._place_yaw_tol_rad = 0.12
    node._latest_world_poses = {}
    node._pose_lock = threading.Lock()
    node._set_pose_calls = []
    node.get_logger = lambda: _Logger()

    def _replace(model_name, pose):
        node._set_pose_calls.append((model_name, pose))
        return None

    node._replace_model_at = _replace
    return node


class TestAngleHelpers(unittest.TestCase):

    def test_yaw_wrap_is_small_near_pi(self):
        err = _mod._angle_err(math.pi - 0.01, -math.pi + 0.01)
        self.assertLess(err, 0.03)

    def test_identity_quaternion_is_zero_rpy(self):
        q = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        roll, pitch, yaw = _mod._rpy_from_quaternion(q)
        self.assertAlmostEqual(roll, 0.0, places=6)
        self.assertAlmostEqual(pitch, 0.0, places=6)
        self.assertAlmostEqual(yaw, 0.0, places=6)


class TestGzPoseVParse(unittest.TestCase):

    def test_parses_named_model_pose(self):
        text = """
header { stamp { sec: 1 nsec: 0 } }
pose {
  name: "pickup_box_0001_carryon"
  id: 8
  position { x: -1.02 y: 0.04 z: 0.81 }
  orientation { x: 0 y: 0 z: 0.1 w: 0.995 }
}
pose {
  name: "ground_plane"
  position { x: 0 y: 0 z: 0 }
  orientation { w: 1 }
}
"""
        poses = _mod._parse_gz_pose_v(text)
        self.assertIn("pickup_box_0001_carryon", poses)
        p = poses["pickup_box_0001_carryon"]
        self.assertAlmostEqual(p.position.x, -1.02)
        self.assertAlmostEqual(p.position.y, 0.04)
        self.assertAlmostEqual(p.position.z, 0.81)
        self.assertAlmostEqual(p.orientation.z, 0.1)
        self.assertAlmostEqual(p.orientation.w, 0.995)


class TestPoseInfoCache(unittest.TestCase):

    def test_prefers_exact_name_then_shortest_link(self):
        node = _bare_spawner()
        requested = _pose(x=1.0, y=2.0, z=3.0)
        msg = TFMessage()
        exact = TransformStamped()
        exact.child_frame_id = "pickup_box_0001_carryon"
        exact.transform.translation.x = 1.0
        exact.transform.translation.y = 2.0
        exact.transform.translation.z = 3.0
        exact.transform.rotation = requested.orientation
        nested = TransformStamped()
        nested.child_frame_id = "pickup_box_0001_carryon::base_link"
        nested.transform.translation.x = 9.0
        nested.transform.translation.y = 9.0
        nested.transform.translation.z = 9.0
        nested.transform.rotation.w = 1.0
        msg.transforms = [nested, exact]
        node._on_pose_info(msg)
        got = node._lookup_model_pose("pickup_box_0001_carryon")
        self.assertAlmostEqual(got.position.x, 1.0)
        self.assertAlmostEqual(got.position.y, 2.0)


class TestEnforceIntendedPose(unittest.TestCase):

    def test_matching_pose_succeeds_with_zero_set_pose(self):
        node = _bare_spawner()
        requested = _pose(x=-1.0, y=0.05, z=0.8, yaw=0.3)
        node._latest_world_poses["box_ok"] = _pose(
            x=-1.0, y=0.05, z=0.8, yaw=0.3)
        err = node._enforce_intended_pose("box_ok", requested)
        self.assertIsNone(err)
        self.assertEqual(node._set_pose_calls, [])

    def test_tilted_pose_retries_then_fails_closed(self):
        node = _bare_spawner()
        requested = _pose(x=0.0, y=0.0, z=0.8, yaw=0.2)
        node._latest_world_poses["box_tilt"] = _pose(
            x=0.0, y=0.0, z=0.8, roll=0.4, yaw=0.2)
        err = node._enforce_intended_pose("box_tilt", requested)
        self.assertIsNotNone(err)
        self.assertTrue(err.startswith("PLACE_VERIFY_FAILED"), err)
        self.assertIn("tilt=", err)
        self.assertEqual(len(node._set_pose_calls), 3)

    def test_absent_pose_retries_then_fails_closed(self):
        node = _bare_spawner()
        err = node._enforce_intended_pose("missing_box", _pose())
        self.assertIsNotNone(err)
        self.assertIn("absent from pose/info", err)
        self.assertEqual(len(node._set_pose_calls), 3)

    def test_retry_then_match_uses_one_set_pose(self):
        node = _bare_spawner()
        requested = _pose(x=0.1, y=-0.1, z=0.7, yaw=-0.4)
        node._latest_world_poses["box_fix"] = _pose(
            x=0.4, y=-0.1, z=0.7, yaw=-0.4)

        def _replace(model_name, pose):
            node._set_pose_calls.append((model_name, pose))
            node._latest_world_poses[model_name] = pose
            return None

        node._replace_model_at = _replace
        err = node._enforce_intended_pose("box_fix", requested)
        self.assertIsNone(err)
        self.assertEqual(len(node._set_pose_calls), 1)


class TestSpawnNextFailClosedOnPlace(unittest.TestCase):

    def test_place_verify_failure_deletes_and_does_not_publish(self):
        node = object.__new__(_mod.PickupBoxSpawner)
        node._visual_kind = "mesh"
        node._model_prefix = "pickup_box"
        node._size_mode = "catalog"
        node._generation = 3
        node._published_id = None
        node._finalized_models = []
        node._current_yaw = 0.0
        node._current_mass = 8.0
        node._models_root_dir = os.path.join(_PKG_ROOT, "models")
        node._observable_cache = {}
        node._rng = random.Random(7)
        node._sequence = 41
        node._current_box = None
        node._current_model = None
        node._current_ref = None
        node._visual_settle_sec = 0.0
        node._box_pub = _Pub()
        node._size_eval_pub = _Pub()
        node._finalized_pub = _Pub()
        calls = {"clear": 0, "spawn": 0, "delete": 0}

        def _sample_box():
            return ({"id": "carryon"}, [0.55, 0.40, 0.25], 8.0, False,
                    "carryon")

        def _entry_pose(_entry, size=None):
            return Pose(), 0.0

        def _clear(_req, _resp):
            calls["clear"] += 1
            class _R(object):
                success = True
                message = ""
            return _R()

        def _spawn_model(*_a, **_k):
            calls["spawn"] += 1
            return None

        def _delete_model(*_a, **_k):
            calls["delete"] += 1
            return None

        def _enforce(_name, _pose):
            return "PLACE_VERIFY_FAILED: tilt=0.400"

        node._sample_box = _sample_box
        node._entry_pose = _entry_pose
        node.handle_clear = _clear
        node._spawn_model = _spawn_model
        node._delete_model = _delete_model
        node._enforce_intended_pose = _enforce
        node.get_logger = lambda: _Logger()

        out = node.handle_spawn_next(None, _mod.SpawnNextBox.Response())
        self.assertFalse(out.success)
        self.assertTrue(out.message.startswith("PLACE_VERIFY_FAILED"),
                        out.message)
        self.assertEqual(calls["clear"], 1)
        self.assertEqual(calls["spawn"], 1)
        self.assertEqual(calls["delete"], 1)
        self.assertIsNone(node._current_box)
        self.assertEqual(node._box_pub.published, [])
        self.assertEqual(node._size_eval_pub.published, [])


if __name__ == "__main__":
    unittest.main()
