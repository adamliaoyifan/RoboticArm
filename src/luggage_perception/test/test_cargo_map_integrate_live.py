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
        # A prior suite in the same pytest process may have left the
        # default context initialized; reuse it instead of raising
        # "Context.init() must only be called once".
        if not rclpy.ok():
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
        if self.rclpy.ok():
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
            IntegrateCargoView, fields.get("endpoint", "/cargo_map/integrate_cargo_view"))
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
        request.operation_id = str(fields.get("operation_id", ""))
        request.expected_epoch = str(fields.get("expected_epoch", ""))
        request.expected_content_digest = str(
            fields.get("expected_content_digest", ""))
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

    def test_full_depth_endpoint_isolated_from_empty_cargo_at_same_stamp(self):
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        topic="/luggage/perception/container_depth_points"
        pub=self.helper.create_publisher(PointCloud2,topic,
            QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT))
        hull=self.mapper._mapper._hull
        camera=(-hull.half_x-.3,0.,.5*(hull.floor_z+hull.ceiling_z))
        self._publish_tf("container_link","depth_bootstrap_test",camera)
        stamp=self._stamp(31)
        cloud=adapters.cloud_msg_from_points(np.array([[hull.length+.3,0.,0.]]),
                                            stamp,"depth_bootstrap_test")
        for _ in range(15):
            pub.publish(cloud)
            self._cloud_pub.publish(adapters.cloud_msg_from_points(np.zeros((0,3)),stamp,"container_link"))
            time.sleep(.02)
        before=self.mapper._mapper.stats()["map_revision"]
        endpoint="/cargo_map/integrate_container_depth_view"
        refused=self._call(stamp=stamp,endpoint=endpoint,settled=False)
        self.assertFalse(refused.success)
        self.assertEqual(self.mapper._mapper.stats()["map_revision"],before)
        accepted=self._call(stamp=stamp,endpoint=endpoint)
        self.assertTrue(accepted.success,accepted.message)
        self.assertEqual(self.mapper._map_source,"measured")
        self.assertEqual(self.mapper._last_acquisition_ns,31_000_000_000)
        self.assertTrue(self.mapper._cargo_observed)
        self.assertEqual(len(self.mapper._mapper.commit_ledger()),0)
        # Same camera acquisition cannot be counted a second time via the
        # cargo endpoint, even though its buffer is independently populated.
        duplicate=self._call(stamp=stamp)
        self.assertFalse(duplicate.success)
        self.assertEqual(duplicate.reason_code,REASON_STAMP_ALREADY_INTEGRATED)

    def test_depth_observer_real_ros_client_reaches_owner(self):
        from copy import deepcopy
        from sensor_msgs.msg import Image, CameraInfo, JointState
        from luggage_perception.container_depth_observer import ContainerDepthObserver
        from rclpy.callback_groups import ReentrantCallbackGroup
        observer=ContainerDepthObserver(self.helper,group=ReentrantCallbackGroup())
        image_pub=self.helper.create_publisher(Image,"/luggage/preprocessed/camera/depth/image",10)
        info_pub=self.helper.create_publisher(CameraInfo,"/luggage/preprocessed/camera/depth/camera_info",10)
        joint_pub=self.helper.create_publisher(JointState,"/joint_states",10)
        hull=self.mapper._mapper._hull
        tf=TransformStamped();tf.header.frame_id="container_link";tf.child_frame_id="observer_optical"
        tf.transform.translation.x=-hull.half_x-.3
        tf.transform.translation.z=.5*(hull.floor_z+hull.ceiling_z)
        tf.transform.rotation.y=2.**-.5;tf.transform.rotation.w=2.**-.5
        self._tf_pub.sendTransform(tf)
        for _ in range(24):
            stamp=self.helper.get_clock().now().to_msg()
            joints=JointState();joints.header.stamp=stamp
            joints.name=['elfin_joint%d'%i for i in range(1,7)]
            joints.position=[0.]*6;joints.velocity=[0.]*6
            image=Image();image.header.stamp=stamp;image.header.frame_id="observer_optical"
            image.height=2;image.width=2;image.step=4;image.encoding="16UC1"
            image.data=np.array([[int((hull.length+.3)*1000),0],[0,0]],dtype=np.uint16).tobytes()
            info=CameraInfo();info.header=deepcopy(image.header);info.height=2;info.width=2
            info.k=[2.,0.,0.,0.,2.,0.,0.,0.,1.]
            joint_pub.publish(joints);image_pub.publish(image);info_pub.publish(info)
            time.sleep(.05)
        before=self.mapper._mapper.stats()["map_revision"]
        result=observer.observe_once("ros-depth-client",timeout=2.)
        self.assertEqual(result["map_revision"],before+1)
        self.assertEqual(self.mapper._map_source,"measured")
        self.assertEqual(self.mapper._last_acquisition_ns,result["acquisition_stamp_ns"])
        self.assertEqual(len(self.mapper._mapper.commit_ledger()),0)

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

    def test_volume_service_exact_revision_read_only_and_stale_refusal(self):
        from luggage_msgs.srv import GetCargoVolumeV2
        self.mapper._frame_scene_revision = 'volume-test-scene'
        stamp = self._stamp(41)
        self._publish_cloud(stamp, self._patch(0.,0.,n=200))
        integrated = self._call(stamp=stamp)
        self.assertTrue(integrated.success, integrated.message)
        client = self.helper.create_client(GetCargoVolumeV2, '/cargo_map/get_volume_v2')
        self.assertTrue(client.wait_for_service(timeout_sec=5.))
        request = GetCargoVolumeV2.Request(scene_revision='volume-test-scene',
            geometry_hash=self.hash, expected_map_revision=integrated.resulting_map_revision)
        before = bytes(self.mapper._mapper._grid)
        future = client.call_async(request)
        deadline = time.monotonic()+5.
        while not future.done() and time.monotonic()<deadline:
            time.sleep(.01)
        self.assertTrue(future.done())
        result = future.result()
        self.assertTrue(result.success, result.reason)
        self.assertEqual(bytes(result.states), before)
        self.assertEqual(bytes(self.mapper._mapper._grid), before)
        self.assertEqual(result.acquisition_stamp_ns,41_000_000_000)
        self.assertEqual(result.map_revision,request.expected_map_revision)
        request.expected_map_revision += 1
        future = client.call_async(request)
        deadline = time.monotonic()+5.
        while not future.done() and time.monotonic()<deadline:
            time.sleep(.01)
        self.assertTrue(future.done())
        self.assertFalse(future.result().success)
        self.assertEqual(future.result().reason,'SNAPSHOT_IDENTITY_MISMATCH')

    def test_volume_read_waits_for_inflight_map_mutation(self):
        from luggage_msgs.srv import GetCargoVolumeV2
        self.mapper._frame_scene_revision = 'volume-test-scene'
        stamp = self._stamp(42)
        self._publish_cloud(stamp,self._patch(0.,0.,n=200))
        old_revision = self.mapper._mapper._revision
        entered,release = threading.Event(),threading.Event()
        original = self.mapper._mapper.integrate_points
        def paused_integration(*args,**kwargs):
            entered.set()
            if not release.wait(5.):
                raise RuntimeError('test integration release timeout')
            return original(*args,**kwargs)
        self.mapper._mapper.integrate_points = paused_integration
        results=[]
        worker=threading.Thread(target=lambda:results.append(self._call(stamp=stamp)))
        worker.start()
        try:
            self.assertTrue(entered.wait(5.))
            client=self.helper.create_client(GetCargoVolumeV2,'/cargo_map/get_volume_v2')
            self.assertTrue(client.wait_for_service(timeout_sec=5.))
            future=client.call_async(GetCargoVolumeV2.Request(scene_revision='volume-test-scene',
                geometry_hash=self.hash,expected_map_revision=old_revision))
            time.sleep(.2)
            self.assertFalse(future.done(),'snapshot read overlapped an owner mutation')
        finally:
            release.set();worker.join(timeout=6.)
            self.mapper._mapper.integrate_points=original
        self.assertTrue(results and results[0].success)
        deadline=time.monotonic()+5.
        while not future.done() and time.monotonic()<deadline:
            time.sleep(.01)
        self.assertTrue(future.done())
        self.assertFalse(future.result().success)
        self.assertEqual(future.result().reason,'SNAPSHOT_IDENTITY_MISMATCH')

    def test_duplicate_stamp(self):
        stamp = self._stamp(17)
        self._publish_cloud(stamp, self._patch(0.05, 0.0, n=120))
        first = self._call(stamp=stamp)
        self.assertTrue(first.success, first.message)
        after = first.resulting_map_revision
        # [B1 unified idempotency] Same synthesized id + same view content
        # (premise revision moved, points and stamp identical) is an
        # OPERATION replay.
        second = self._call(stamp=stamp, expected_map_revision=after)
        self.assertFalse(second.success)
        self.assertEqual(second.reason_code, "ALREADY_PROCESSED")
        # An UNKNOWN operation submitting the same physical frame is the
        # duplicate-frame case.
        third = self._call(stamp=stamp, view_request_id="other",
                           expected_map_revision=after)
        self.assertFalse(third.success)
        self.assertEqual(third.reason_code, REASON_STAMP_ALREADY_INTEGRATED)
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
        # [B1] The store is the sole writer: the geometry seed goes through
        # a transaction (a direct mark_placed_box would leave the store's
        # snapshot at a stale identity and the next premise would fail).
        from luggage_perception.occupancy_map_store import (
            MapUpdate, UpdateKind)
        center_z = self.mapper._mapper.center[2]
        self.mapper._store.apply_update(MapUpdate(
            operation_id="seed:geometry-lock", kind=UpdateKind.ADD_BOX_V1,
            box=([0.0, 0.0, center_z], [0.2, 0.2, 0.2], 0.0)))
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

    def test_operation_id_retry_is_processed_once(self):
        """[B1] A literal retry of one integration (same operation id,
        same view) replays as ALREADY_PROCESSED with the current identity
        echoed — never re-applied; a different operation id on the same
        frame is the duplicate-frame refusal."""
        from luggage_msgs.srv import ResetCargoMap
        stamp = self._stamp(30)
        self._publish_cloud(stamp, self._patch(0.1, 0.1, n=80))
        first = self._call(stamp=stamp, operation_id="op-live-1")
        self.assertTrue(first.success, first.message)
        self.assertEqual(first.status, "accepted")
        self.assertTrue(first.epoch)
        self.assertTrue(first.content_digest)
        revision = int(first.resulting_map_revision)
        second = self._call(stamp=stamp, operation_id="op-live-1",
                            expected_map_revision=revision)
        self.assertFalse(second.success)
        self.assertEqual(second.reason_code, "ALREADY_PROCESSED")
        self.assertEqual(int(second.resulting_map_revision), revision)
        self.assertEqual(second.epoch, first.epoch)
        third = self._call(stamp=stamp, operation_id="op-live-2",
                           expected_map_revision=revision)
        self.assertFalse(third.success)
        self.assertEqual(third.reason_code,
                         REASON_STAMP_ALREADY_INTEGRATED)

    def test_reset_rotates_epoch(self):
        """[B1] Reset rotates the epoch: a write carrying the pre-reset
        identity is refused with EPOCH_MISMATCH, never silently applied."""
        from luggage_msgs.srv import ResetCargoMap
        client = self.helper.create_client(
            ResetCargoMap, "/cargo_map/reset")
        self.assertTrue(client.wait_for_service(timeout_sec=5.0))
        before_epoch = self.mapper._store.epoch
        future = client.call_async(ResetCargoMap.Request())
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and not future.done():
            time.sleep(0.02)
        self.assertTrue(future.done())
        reset = future.result()
        self.assertTrue(reset.success)
        self.assertNotEqual(reset.epoch, before_epoch)
        self.assertTrue(reset.content_digest)
        stamp = self._stamp(31)
        self._publish_cloud(stamp, self._patch(-0.1, 0.1, n=60))
        stale = self._call(
            stamp=stamp, expected_epoch=before_epoch,
            expected_map_revision=int(
                self.mapper._mapper.stats()["map_revision"]))
        self.assertFalse(stale.success)
        self.assertEqual(stale.reason_code, "EPOCH_MISMATCH")


if __name__ == "__main__":
    unittest.main()
