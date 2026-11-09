#!/usr/bin/env python3
"""Live cargo_volume_mapper + placement_planner geometry identity (GEO-8).

Launch-free: both production nodes run in one executor, no Gazebo and no
MoveIt. The negative case gives the mapper a different container hull and
requires ComputePlacement to fail closed instead of answering from its own
floor prior.
"""

import copy
import importlib.util
import os
import tempfile
import time
import unittest

import yaml

from luggage_description._share import ENV_SCENE_TF
from luggage_description.scene_tf_config_utils import (
    container_inner_geometry_descriptor,
    default_scene_tf_config_path,
    load_scene_tf_config,
)

_HERE = os.path.dirname(os.path.abspath(__file__))
_WS_SRC = os.path.normpath(os.path.join(_HERE, "..", ".."))


def _load_node_module(name, package, relative):
    path = os.path.join(_WS_SRC, package, "scripts", relative)
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _mutated_scene(path):
    """Same container with a different floor chamfer -> different hash."""
    scene = copy.deepcopy(load_scene_tf_config(path))
    chamfer = scene["container"]["inner"]["chamfer"]
    chamfer["floor_y"] = round(float(chamfer["floor_y"]) - 0.10, 4)
    handle = tempfile.NamedTemporaryFile(
        "w", suffix="_scene_tf.yaml", delete=False, encoding="utf-8")
    yaml.safe_dump(scene, handle)
    handle.close()
    return handle.name


class TestGeometryIdentityLive(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        try:
            import rclpy  # noqa: F401
            from luggage_msgs.srv import ComputePlacement  # noqa: F401
        except ImportError as exc:
            raise unittest.SkipTest("rclpy/luggage_msgs unavailable: %s" % exc)
        cls.scene_path = default_scene_tf_config_path()
        cls.mutated_path = _mutated_scene(cls.scene_path)
        cls.mapper_mod = _load_node_module(
            "cargo_volume_mapper_node", "luggage_perception",
            "cargo_volume_mapper_node.py")
        cls.planner_mod = _load_node_module(
            "placement_planner_node", "luggage_packing",
            "placement_planner_node.py")

    @classmethod
    def tearDownClass(cls):
        if getattr(cls, "mutated_path", None):
            os.unlink(cls.mutated_path)

    def setUp(self):
        import rclpy
        from rclpy.executors import MultiThreadedExecutor

        self.rclpy = rclpy
        # TEST-ONLY wiring (the real machine needs none of this): the
        # planner's producer in this harness is the G cargo mapper, so
        # the planner runs the grid_g validity backend — the production
        # observer deployments use the observer_o defaults instead.
        rclpy.init(args=[
            "--ros-args", "-p", "validity_backend:=grid_g",
            "-p", "validity_topic:=/cargo_map/validity",
            "-p", "validity_status_service:=/cargo_map/get_validity",
            "-p", "frame_scene_revision:=gil-test-scene",
            # no reachability atlas in the unit env: identity would
            # hard-reject before the geometry assertions (same as e2e)
            "-p", "atlas_enabled:=false",
            # chain-aligned robot window: real computes take ~0.3-0.5 s
            "-p", "robot_state_max_age_ms:=2000"])
        # MultiThreaded (production-shaped), created AFTER rclpy.init: a
        # single-threaded executor starves the planner's service handler
        # behind the mapper's cloud/republish callbacks and the robot
        # sample ages out
        self.executor = MultiThreadedExecutor()
        self.nodes = []
        self.addCleanup(self._shutdown)

    def _shutdown(self):
        self._stop_joints = True
        if getattr(self, "_joints_thread", None) is not None:
            self._joints_thread.join(timeout=2.0)
        for node in self.nodes:
            self.executor.remove_node(node)
            node.destroy_node()
        self.executor.shutdown()
        self.rclpy.shutdown()

    def _start(self, mapper_scene, planner_scene):
        # Each node resolves its own hull from its own scene config, which is
        # exactly the split GEO-8 has to detect.
        mapper = _construct(
            self.mapper_mod.CargoVolumeMapperNode, mapper_scene)
        planner = _construct(
            self.planner_mod.PlacementPlannerNode, planner_scene)
        for node in (mapper, planner):
            self.nodes.append(node)
            self.executor.add_node(node)
        # joints publisher EARLY: a publisher created at compute time
        # loses its own first message to DDS discovery (volatile QoS)
        from sensor_msgs.msg import JointState as _JointState
        self._joints_pub = planner.create_publisher(
            _JointState, "/joint_states", 10)
        self._fresh_joints(planner)
        return mapper, planner

    def _spin(self, seconds=1.0):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.02)

    def _spin_until(self, predicate, timeout_sec=10.0):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.02)
            if predicate():
                return True
        return False

    def _request(self, geometry_hash=""):
        from luggage_msgs.msg import DetectedLuggage
        from luggage_msgs.srv import ComputePlacement

        request = ComputePlacement.Request()
        box = DetectedLuggage()
        box.width, box.depth, box.height = 0.55, 0.40, 0.25
        box.height_valid = True
        box.height_source = 1  # MEASURED_SUPPORT
        box.pose.orientation.w = 1.0
        request.box = box
        request.geometry_hash = geometry_hash
        return request

    def _feed_observation(self, planner):
        """Drive the REAL G intake (what the campaign client does): a
        static TF optical_frame<-container_link, a small cloud on the
        production topic, then the production IntegrateCargoView service
        with geometry+revision pinned from the mapper's own stats. The
        mapper's lifecycle then goes VALID by itself and publishes the
        validity + map streams the planner's mandatory gate requires —
        no stubs."""
        from sensor_msgs.msg import PointCloud2, PointField
        from geometry_msgs.msg import TransformStamped
        from luggage_msgs.srv import GetCargoMapStats, IntegrateCargoView
        import struct as _struct
        import tf2_ros as _tf2

        static_broadcaster = _tf2.StaticTransformBroadcaster(planner)
        tf = TransformStamped()
        now = planner.get_clock().now()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = "container_link"
        tf.child_frame_id = "optical_frame"
        static_broadcaster.sendTransform(tf)
        self._spin(0.5)

        cloud_pub = planner.create_publisher(
            PointCloud2, "/luggage/semantic/cargo_points_untracked", 10)
        stamp = planner.get_clock().now().nanoseconds
        points = []
        for dx in range(-3, 4):
            for dy in range(-3, 4):
                points.append((0.10 + dx * 0.01, -0.30 + dy * 0.01, 0.80))
        fields = []
        for index, name in enumerate(("x", "y", "z")):
            field = PointField()
            field.name = name
            field.offset = index * 4
            field.datatype = PointField.FLOAT32
            field.count = 1
            fields.append(field)
        cloud = PointCloud2()
        cloud.header.frame_id = "optical_frame"
        cloud.header.stamp.sec = int(stamp // 1_000_000_000)
        cloud.header.stamp.nanosec = int(stamp % 1_000_000_000)
        cloud.height, cloud.width = 1, len(points)
        cloud.point_step, cloud.row_step = 12, 12 * len(points)
        cloud.is_bigendian = False
        cloud.fields = fields
        cloud.data = b"".join(
            _struct.pack("<fff", *point) for point in points)
        cloud_pub.publish(cloud)
        self._spin(0.3)

        stats_client = planner.create_client(
            GetCargoMapStats, "/cargo_map/get_stats")
        integrate = planner.create_client(
            IntegrateCargoView, "/cargo_map/integrate_cargo_view")
        self.assertTrue(self._spin_until(
            stats_client.service_is_ready, 10.0), "mapper stats service")
        self.assertTrue(self._spin_until(
            integrate.service_is_ready, 10.0), "mapper integrate service")
        future = stats_client.call_async(GetCargoMapStats.Request())
        self.assertTrue(self._spin_until(future.done, 10.0))
        stats = future.result()
        request = IntegrateCargoView.Request()
        request.schema_version = 1
        request.view_request_id = "gil-obs-1"
        request.source_acquisition_stamp.sec = int(stamp // 1_000_000_000)
        request.source_acquisition_stamp.nanosec = int(
            stamp % 1_000_000_000)
        request.settled = True
        request.frame_count = 1
        request.geometry_hash = str(getattr(stats, "geometry_hash", ""))
        request.expected_map_revision = int(
            getattr(stats, "map_revision", 0) or 0)
        future = integrate.call_async(request)
        self.assertTrue(self._spin_until(future.done, 30.0))
        # The gen-N map publishes while the lifecycle still reads
        # BUILDING and is finally rejected; once the planner's validity
        # state is VALID, a SECOND observation's map revision arrives
        # under the settled generation and is admitted.
        self.assertTrue(self._spin_until(
            lambda: str(getattr(
                planner._admission.validity, "state", "")) == "VALID",
            15.0), "mapper validity never reached VALID")
        request.view_request_id = "gil-obs-2"
        stamp2 = planner.get_clock().now().nanoseconds
        cloud.data = b"".join(
            _struct.pack("<fff", p[0], p[1], p[2] + 0.02) for p in points)
        cloud.header.stamp.sec = int(stamp2 // 1_000_000_000)
        cloud.header.stamp.nanosec = int(stamp2 % 1_000_000_000)
        cloud_pub.publish(cloud)
        self._spin(0.3)
        request.source_acquisition_stamp.sec = int(stamp2 // 1_000_000_000)
        request.source_acquisition_stamp.nanosec = int(
            stamp2 % 1_000_000_000)
        request.expected_map_revision = int(
            getattr(stats, "map_revision", 0) or 0) + 1
        future = integrate.call_async(request)
        self.assertTrue(self._spin_until(future.done, 30.0))
        result = future.result()
        return result

    def _fresh_joints(self, planner):
        """TEST STUB (real machines stream real joint states): a 10 Hz
        joints publisher for the life of the test — a single sample
        ages out whenever the service handler queues behind mapper
        callbacks, exactly like the production chain where /joint_states
        is continuous."""
        if getattr(self, "_joints_thread", None) is not None:
            return
        import threading as _threading
        from sensor_msgs.msg import JointState as _JointState

        def _stream():
            import time as _time
            while self.rclpy.ok() and not getattr(self, "_stop_joints", False):
                msg = _JointState()
                msg.header.stamp = planner.get_clock().now().to_msg()
                msg.name = ["elfin_joint%d" % i for i in range(1, 7)]
                msg.position = [0.0] * 6
                self._joints_pub.publish(msg)
                _time.sleep(0.1)

        self._joints_thread = _threading.Thread(target=_stream, daemon=True)
        self._joints_thread.start()
        self._spin(0.3)   # let the first samples deliver

    def _compute(self, planner, request):
        from luggage_msgs.srv import ComputePlacement
        client = planner.create_client(
            ComputePlacement, "/placement_planner/compute_placement")
        self.assertTrue(
            self._spin_until(client.service_is_ready, 10.0),
            "compute_placement service never came up")
        self._fresh_joints(planner)
        future = client.call_async(request)
        self.assertTrue(
            self._spin_until(future.done, 30.0), "ComputePlacement timed out")
        return future.result()

    # ------------------------------------------------------------------

    def test_matching_geometry_answers_with_its_hull_hash(self):
        expected = str(container_inner_geometry_descriptor(
            load_scene_tf_config(self.scene_path))["geometry_hash"])
        mapper, planner = self._start(self.scene_path, self.scene_path)
        self._feed_observation(planner)
        self.assertEqual(mapper._geometry_hash, expected)
        self.assertTrue(
            self._spin_until(lambda: planner._surface is not None, 10.0),
            "planner never accepted the cargo map")
        self.assertEqual(
            str(planner._surface.get("geometry_hash")), expected)

        response = self._compute(planner, self._request(expected))
        self.assertTrue(response.success, response.message)
        self.assertEqual(response.geometry_hash, expected)
        self.assertEqual(response.reason_code, "")
        # identity contract: ZERO geometry rejections in the matching
        # case. Total rejections may be nonzero: the mapper's FIRST
        # publication legitimately races the validity binding (lifecycle
        # admission rejection — startup behavior, not an identity fault).
        self.assertEqual(planner._geometry_rejected_maps, 0)

    def test_foreign_cargo_map_fails_closed_without_floor_prior(self):
        mapper, planner = self._start(self.mutated_path, self.scene_path)
        self._feed_observation(planner)
        self.assertNotEqual(mapper._geometry_hash, planner._geometry_hash)
        self.assertTrue(
            self._spin_until(lambda: planner._rejected_maps > 0, 10.0),
            "planner accepted a map built for another hull")
        self.assertIsNone(planner._surface)

        response = self._compute(planner, self._request())
        self.assertFalse(response.success)
        self.assertEqual(response.reason_code, "CARGO_MAP_GEOMETRY_MISMATCH")
        self.assertEqual(response.geometry_hash, planner._geometry_hash)
        # Fail closed means no slot at all, not a floor-prior guess.
        self.assertEqual(
            (response.slot.width, response.slot.depth, response.slot.height),
            (0.0, 0.0, 0.0))

    def test_request_hash_pin_is_enforced(self):
        _mapper, planner = self._start(self.scene_path, self.scene_path)
        self._feed_observation(planner)
        self.assertTrue(self._spin_until(
            lambda: planner._surface is not None, 10.0),
            "planner never accepted the cargo map")
        response = self._compute(planner, self._request("not-this-hull"))
        self.assertFalse(response.success)
        self.assertEqual(response.reason_code, "CARGO_MAP_GEOMETRY_MISMATCH")


def _construct(node_class, scene_path):
    """Instantiate a production node against one scene config."""
    previous = os.environ.get(ENV_SCENE_TF)
    os.environ[ENV_SCENE_TF] = scene_path
    try:
        return node_class()
    finally:
        if previous is None:
            os.environ.pop(ENV_SCENE_TF, None)
        else:
            os.environ[ENV_SCENE_TF] = previous


if __name__ == "__main__":
    unittest.main()
