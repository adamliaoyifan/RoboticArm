#!/usr/bin/env python3
"""Live TF lookups against the in-process publisher and optional full launch."""

import math
import os
import subprocess
import sys
import tempfile
import time
import unittest

from luggage_description.scene_tf_config_utils import (
    container_opening_target_point,
    load_scene_tf_config,
    robot_base_in_world,
)
from luggage_description.scene_tf_publisher import (
    ContainerTfPublisher,
    rpy_to_quaternion,
    static_transform_payloads,
)

CONFIG = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", "config", "scene_tf.yaml.example"))


def _spin_until(executor, predicate, timeout_sec=5.0):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
        if predicate():
            return True
    return False


def _quat_to_yaw(q):
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class TestLiveStaticTf(unittest.TestCase):
    _inited = False

    @classmethod
    def setUpClass(cls):
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from tf2_ros import Buffer, TransformListener
        except ImportError as exc:
            raise unittest.SkipTest("rclpy/tf2_ros not available: %s" % exc)

        rclpy.init()
        cls._inited = True
        cls._rclpy = rclpy
        cls.pub_node = rclpy.create_node(
            "container_tf_publisher",
            parameter_overrides=[
                rclpy.parameter.Parameter(
                    "scene_tf_config",
                    rclpy.parameter.Parameter.Type.STRING,
                    CONFIG,
                ),
            ],
        )
        ContainerTfPublisher(cls.pub_node)
        cls.listen_node = rclpy.create_node("scene_tf_test_listener")
        cls.buffer = Buffer()
        cls.listener = TransformListener(cls.buffer, cls.listen_node)
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.pub_node)
        cls.executor.add_node(cls.listen_node)

    @classmethod
    def tearDownClass(cls):
        if not getattr(cls, "_inited", False):
            return
        cls.executor.remove_node(cls.pub_node)
        cls.executor.remove_node(cls.listen_node)
        cls.pub_node.destroy_node()
        cls.listen_node.destroy_node()
        cls._rclpy.shutdown()

    def _lookup(self, parent, child):
        from rclpy.time import Time

        ok = _spin_until(
            self.executor,
            lambda: self.buffer.can_transform(parent, child, Time()),
        )
        self.assertTrue(ok, "timed out waiting for %s -> %s" % (parent, child))
        return self.buffer.lookup_transform(parent, child, Time())

    def test_world_to_container_matches_yaml(self):
        tf_msg = self._lookup("world", "container_link")
        self.assertAlmostEqual(tf_msg.transform.translation.x, 1.5, places=6)
        self.assertAlmostEqual(tf_msg.transform.translation.y, 0.0, places=6)
        self.assertAlmostEqual(tf_msg.transform.translation.z, 0.0, places=6)

    def test_static_edges_are_present(self):
        config = load_scene_tf_config(CONFIG)
        for item in static_transform_payloads(config):
            self._lookup(item["parent"], item["child"])

    def test_elfin_base_is_not_on_tf_static(self):
        from rclpy.time import Time

        _spin_until(
            self.executor,
            lambda: self.buffer.can_transform("world", "container_link", Time()),
            timeout_sec=2.0,
        )
        self.assertFalse(
            self.buffer.can_transform("world", "elfin_base_link", Time()),
            "elfin_base_link must come from robot_state_publisher, not static TF",
        )

    def test_task_roi_params_live_on_this_node(self):
        center = self.pub_node.get_parameter("task_roi.container_center").value
        self.assertGreater(abs(center[1]), 1.0)
        dims = self.pub_node.get_parameter("task_roi.container_dims").value
        self.assertAlmostEqual(dims[2], 1.48, places=3)


class TestLibraryOpeningInBase(unittest.TestCase):
    """Graph-free numeric pins used by the live TF exit criteria."""

    def test_opening_target_and_base_pose(self):
        config = load_scene_tf_config(CONFIG)
        xyz, rpy = robot_base_in_world(config)
        self.assertAlmostEqual(xyz[2], 0.86, places=4)
        self.assertAlmostEqual(rpy[2], 1.5708, places=4)
        qx, qy, qz, qw = rpy_to_quaternion(rpy)
        self.assertAlmostEqual(_quat_to_yaw((qx, qy, qz, qw)), rpy[2], places=4)
        opening = container_opening_target_point(config)
        self.assertAlmostEqual(opening[0], -0.27, places=4)
        self.assertAlmostEqual(opening[1], -0.755, places=4)
        self.assertAlmostEqual(opening[2], 0.44, places=4)


def _probe_scene_launch():
    """Child-process probe: launch scene.launch.py and check live TF numbers."""
    try:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.time import Time
        from tf2_ros import Buffer, TransformListener
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory("luggage_description")
    except Exception as exc:
        sys.stderr.write("skip-or-fail probe imports: %s\n" % exc)
        return 2

    log_path = tempfile.mkstemp(prefix="scene_launch_", suffix=".log")[1]
    log_file = open(log_path, "w")
    proc = subprocess.Popen(
        [
            "ros2", "launch", "luggage_description", "scene.launch.py",
            "use_rviz:=false",
        ],
        stdout=log_file,
        stderr=subprocess.STDOUT,
        text=True,
    )

    def _launch_log():
        log_file.flush()
        try:
            with open(log_path, "r") as handle:
                return handle.read()
        except OSError:
            return ""

    rclpy.init()
    node = rclpy.create_node("scene_launch_tf_probe")
    buffer = Buffer()
    _listener = TransformListener(buffer, node)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        def ready():
            return (
                buffer.can_transform("world", "elfin_base_link", Time())
                and buffer.can_transform("world", "container_link", Time())
                and buffer.can_transform(
                    "elfin_base_link", "container_opening_frame", Time())
                and buffer.can_transform(
                    "world", "camera_depth_optical_frame", Time())
                and buffer.can_transform("world", "suction_panel", Time())
            )

        deadline = time.monotonic() + 45.0
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                sys.stderr.write(
                    "scene.launch.py exited %s\n%s\n"
                    % (proc.returncode, _launch_log())
                )
                return 1
            executor.spin_once(timeout_sec=0.05)
            if ready():
                break
        else:
            sys.stderr.write(
                "timed out waiting for scene TF tree\n%s\n" % _launch_log()
            )
            return 1
        config = load_scene_tf_config(CONFIG)
        base_xyz, base_rpy = robot_base_in_world(config)
        base_tf = buffer.lookup_transform("world", "elfin_base_link", Time())
        assert abs(base_tf.transform.translation.z - base_xyz[2]) < 5e-4
        yaw = _quat_to_yaw((
            base_tf.transform.rotation.x,
            base_tf.transform.rotation.y,
            base_tf.transform.rotation.z,
            base_tf.transform.rotation.w,
        ))
        assert abs(yaw - base_rpy[2]) < 5e-3, (yaw, base_rpy[2])
        container = buffer.lookup_transform("world", "container_link", Time())
        assert abs(container.transform.translation.x - 1.5) < 1e-6
        opening = container_opening_target_point(config)
        opening_tf = buffer.lookup_transform(
            "elfin_base_link", "container_opening_frame", Time())
        assert abs(opening_tf.transform.translation.x - opening[0]) < 5e-3
        assert abs(opening_tf.transform.translation.y - opening[1]) < 5e-3
        assert abs(opening_tf.transform.translation.z - opening[2]) < 5e-3
        sys.stdout.write("scene launch TF probe ok\n")
        return 0
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
        log_file.close()


class TestSceneLaunchGraph(unittest.TestCase):
    """Start scene.launch.py in an isolated process (avoids rclpy re-init)."""

    def test_world_base_opening_and_camera_frames(self):
        try:
            from ament_index_python.packages import get_package_share_directory
            get_package_share_directory("luggage_description")
        except Exception:
            self.skipTest("luggage_description share not installed")
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = env.get("LUGGAGE_TEST_ROS_DOMAIN_ID", "113")
        result = subprocess.run(
            [sys.executable, os.path.abspath(__file__), "--probe-launch"],
            capture_output=True,
            text=True,
            timeout=90,
            env=env,
        )
        if result.returncode == 2:
            self.skipTest(result.stderr.strip() or "probe imports unavailable")
        self.assertEqual(
            result.returncode,
            0,
            "stdout:\n%s\nstderr:\n%s" % (result.stdout, result.stderr),
        )


if __name__ == "__main__":
    if "--probe-launch" in sys.argv:
        sys.exit(_probe_scene_launch())
    unittest.main()
