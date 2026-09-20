#!/usr/bin/env python3
"""Launch-free IntegrateCargoView contract (OCC-1).

Runs the production cargo_volume_mapper_node in one executor with a
static TF and a synthetic untracked cargo cloud. No Gazebo.
"""

import importlib.util
import os
import threading
import time
import unittest

import numpy as np
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2

from luggage_description._share import ENV_SCENE_TF
from luggage_description.scene_tf_config_utils import (
    container_inner_geometry_descriptor,
    default_scene_tf_config_path,
    load_scene_tf_config,
)
from luggage_perception import ros_message_adapters as adapters
from luggage_perception.cargo_view_integration import (
    REASON_GEOMETRY_HASH_MISMATCH,
    REASON_NOT_SETTLED,
    REASON_REVISION_MISMATCH,
    REASON_STAMP_ALREADY_INTEGRATED,
    REASON_STAMP_STALE,
    REASON_TF_MISSING,
    REASON_VIEW_EMPTY,
    SCHEMA_VERSION,
)

_HERE = os.path.dirname(os.path.abspath(__file__))
_WS_SRC = os.path.normpath(os.path.join(_HERE, "..", ".."))


def _load_node_module():
    path = os.path.join(
        _WS_SRC, "luggage_perception", "scripts", "cargo_volume_mapper_node.py")
    spec = importlib.util.spec_from_file_location("cargo_volume_mapper_node", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _construct(node_class, scene_path):
    previous = os.environ.get(ENV_SCENE_TF)
    os.environ[ENV_SCENE_TF] = scene_path
    try:
        return node_class()
    finally:
        if previous is None:
            os.environ.pop(ENV_SCENE_TF, None)
        else:
            os.environ[ENV_SCENE_TF] = previous


class TestCargoMapIntegrateLive(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        try:
            import rclpy  # noqa: F401
            from luggage_msgs.srv import IntegrateCargoView  # noqa: F401
            from tf2_ros import StaticTransformBroadcaster  # noqa: F401
        except ImportError as exc:
            raise unittest.SkipTest("rclpy/luggage_msgs unavailable: %s" % exc)
        cls.scene_path = default_scene_tf_config_path()
        cls.hash = str(container_inner_geometry_descriptor(
            load_scene_tf_config(cls.scene_path))["geometry_hash"])
        cls.mod = _load_node_module()

    def setUp(self):
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from tf2_ros import StaticTransformBroadcaster

        self.rclpy = rclpy
        rclpy.init()
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.nodes = []
        self.mapper = _construct(self.mod.CargoVolumeMapperNode, self.scene_path)
        self.helper = rclpy.create_node("occ1_integrate_helper")
        self._tf_pub = StaticTransformBroadcaster(self.helper)
        for node in (self.mapper, self.helper):
            self.nodes.append(node)
            self.executor.add_node(node)
        self._spin_thread = threading.Thread(
            target=self.executor.spin, daemon=True)
        self._spin_thread.start()
        self.addCleanup(self._shutdown)
        self._publish_tf("world", "container_link")
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._cloud_pub = self.helper.create_publisher(
            PointCloud2, "/luggage/semantic/cargo_points_untracked", qos)
        time.sleep(0.2)

    def _shutdown(self):
        self.executor.shutdown()
        for node in self.nodes:
            node.destroy_node()
        if hasattr(self.mapper, "shutdown_tf"):
            self.mapper.shutdown_tf()
        self.rclpy.shutdown()

    def _publish_tf(self, parent, child, xyz=(0.0, 0.0, 0.0), stamp=None):
        msg = TransformStamped()
        msg.header.stamp = stamp or Time(sec=0, nanosec=0)
        msg.header.frame_id = parent
        msg.child_frame_id = child
        msg.transform.translation.x = float(xyz[0])
        msg.transform.translation.y = float(xyz[1])
        msg.transform.translation.z = float(xyz[2])
        msg.transform.rotation.w = 1.0
        self._tf_pub.sendTransform(msg)

    def _stamp(self, sec, nsec=0):
        return Time(sec=int(sec), nanosec=int(nsec))

    def _publish_cloud(self, stamp, points, frame="container_link"):
        msg = adapters.cloud_msg_from_points(points, stamp, frame)
        for _ in range(8):
            self._cloud_pub.publish(msg)
            time.sleep(0.02)
        return msg

    def _call(self, **fields):
        from luggage_msgs.srv import IntegrateCargoView
        client = self.helper.create_client(
            IntegrateCargoView, "/cargo_map/integrate_cargo_view")
        self.assertTrue(client.wait_for_service(timeout_sec=5.0))
        request = IntegrateCargoView.Request()
        request.schema_version = int(fields.get("schema_version", SCHEMA_VERSION))
        request.view_request_id = str(fields.get("view_request_id", "t"))
        request.candidate_id = str(fields.get("candidate_id", "c"))
        request.source_acquisition_stamp = fields["stamp"]
        request.expected_map_revision = int(
            fields.get("expected_map_revision", self.mapper._mapper.stats()["map_revision"]))
        request.geometry_hash = str(fields.get("geometry_hash", self.hash))
        request.settled = bool(fields.get("settled", True))
        request.frame_count = int(fields.get("frame_count", 1))
        future = client.call_async(request)
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and not future.done():
            time.sleep(0.02)
        self.assertTrue(future.done(), "IntegrateCargoView timed out")
        return future.result()

    def _patch(self, cx, cy, cz=None, n=80):
        floor_z = float(self.mapper._mapper.center[2] - 0.5 * self.mapper._mapper.inner_h)
        z = floor_z + 0.20 if cz is None else float(cz)
        rng = np.random.RandomState(abs(hash((cx, cy))) % (2 ** 31))
        pts = rng.normal(scale=0.02, size=(n, 3))
        pts[:, 0] += float(cx)
        pts[:, 1] += float(cy)
        pts[:, 2] = z + rng.normal(scale=0.01, size=n)
        return pts

    def test_settled_false(self):
        stamp = self._stamp(10)
        before = self.mapper._mapper.stats()["map_revision"]
        resp = self._call(stamp=stamp, settled=False)
        self.assertFalse(resp.success)
        self.assertEqual(resp.reason_code, REASON_NOT_SETTLED)
        self.assertEqual(self.mapper._mapper.stats()["map_revision"], before)

    def test_hash_mismatch(self):
        stamp = self._stamp(11)
        resp = self._call(stamp=stamp, geometry_hash="not-this-hull")
        self.assertFalse(resp.success)
        self.assertEqual(resp.reason_code, REASON_GEOMETRY_HASH_MISMATCH)

    def test_revision_stale(self):
        stamp = self._stamp(12)
        resp = self._call(stamp=stamp, expected_map_revision=0)
        self.assertFalse(resp.success)
        self.assertEqual(resp.reason_code, REASON_REVISION_MISMATCH)

    def test_stamp_stale(self):
        stamp = self._stamp(13)
        resp = self._call(stamp=stamp)
        self.assertFalse(resp.success)
        self.assertEqual(resp.reason_code, REASON_STAMP_STALE)

    def test_missing_tf(self):
        stamp = self._stamp(14)
        self._publish_cloud(stamp, self._patch(0.0, 0.0), frame="no_such_frame")
        resp = self._call(stamp=stamp)
        self.assertFalse(resp.success)
        self.assertEqual(resp.reason_code, REASON_TF_MISSING)

    def test_empty_cloud(self):
        stamp = self._stamp(15)
        self._publish_cloud(stamp, np.zeros((0, 3)))
        resp = self._call(stamp=stamp)
        self.assertFalse(resp.success)
        self.assertEqual(resp.reason_code, REASON_VIEW_EMPTY)

    def test_success_sensor_cells(self):
        stamp = self._stamp(16)
        self._publish_cloud(stamp, self._patch(0.0, 0.0, n=200))
        before = self.mapper._mapper.stats()["map_revision"]
        resp = self._call(stamp=stamp)
        self.assertTrue(resp.success, resp.message)
        self.assertGreater(resp.resulting_map_revision, before)
        surface = self.mapper._mapper.surface_map_2d()
        occupied = [
            1 for ix in range(surface["nx"]) for iy in range(surface["ny"])
            if (surface["state"][ix][iy] == "occupied"
                and surface["confidence"][ix][iy] == "sensor")
        ]
        self.assertTrue(occupied)

    def test_duplicate_stamp(self):
        stamp = self._stamp(17)
        self._publish_cloud(stamp, self._patch(0.05, 0.0, n=120))
        first = self._call(stamp=stamp)
        self.assertTrue(first.success, first.message)
        after = first.resulting_map_revision
        second = self._call(stamp=stamp, expected_map_revision=after)
        self.assertFalse(second.success)
        self.assertEqual(second.reason_code, REASON_STAMP_ALREADY_INTEGRATED)
        self.assertEqual(
            self.mapper._mapper.stats()["map_revision"], after)

    def test_two_clusters_accumulate(self):
        s1 = self._stamp(18)
        s2 = self._stamp(19)
        self._publish_cloud(s1, self._patch(-0.35, 0.0, n=150))
        self.assertTrue(self._call(stamp=s1).success)
        self._publish_cloud(s2, self._patch(0.35, 0.0, n=150))
        self.assertTrue(self._call(stamp=s2).success)
        surface = self.mapper._mapper.surface_map_2d()
        from luggage_perception.cargo_view_integration import (
            footprint_sensor_coverage)
        left, _, _ = footprint_sensor_coverage(
            surface, (-0.5, -0.15, -0.2, 0.15))
        right, _, _ = footprint_sensor_coverage(
            surface, (0.2, -0.15, 0.5, 0.15))
        self.assertGreater(left, 0.0)
        self.assertGreater(right, 0.0)

    def test_geometry_lock_then_integrate(self):
        self.mapper._mapper.mark_placed_box(
            [0.0, 0.0, self.mapper._mapper.center[2]],
            [0.2, 0.2, 0.2])
        stamp = self._stamp(20)
        self._publish_cloud(stamp, self._patch(0.0, 0.0, n=80))
        resp = self._call(stamp=stamp)
        self.assertTrue(resp.success, resp.message)
        surface = self.mapper._mapper.surface_map_2d()
        confs = {
            surface["confidence"][ix][iy]
            for ix in range(surface["nx"])
            for iy in range(surface["ny"])
            if surface["state"][ix][iy] == "occupied"
        }
        self.assertIn("geometry", confs)


if __name__ == "__main__":
    unittest.main()
