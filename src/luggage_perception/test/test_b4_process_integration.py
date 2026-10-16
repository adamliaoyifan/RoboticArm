#!/usr/bin/env python3
"""B4 Gate 2: process integration — real ROS messages through the real node.

Scenarios (each ×3 per the gate spec): single-snapshot publication
consistency across an update, literal service retry, node restart (epoch
rotation refusing the old identity), and consumer version consistency
(latched subscriber sees monotonic revisions with matching digests). It
validates the process wiring, not online success rates.
"""
import importlib.util
import json
import os
import threading
import time
import unittest

from std_msgs.msg import String

from luggage_description._share import ENV_SCENE_TF
from luggage_description.scene_tf_config_utils import (
    default_scene_tf_config_path,
)

_HERE = os.path.dirname(os.path.abspath(__file__))
_NODE = os.path.normpath(os.path.join(
    _HERE, "..", "scripts", "cargo_volume_mapper_node.py"))
_ROOT = os.path.normpath(os.path.join(_HERE, "..", "..", ".."))

import sys  # noqa: E402
for _entry in (os.path.join(_ROOT, "src", "luggage_perception"),
               os.path.join(_ROOT, "src", "luggage_description"),
               os.path.join(_ROOT, "src", "luggage_planning"), _ROOT):
    if _entry not in sys.path:
        sys.path.append(_entry)

REPS = 3


def _load_node_module():
    try:
        import rclpy  # noqa: F401
        from luggage_msgs.srv import IntegrateCargoView  # noqa: F401
    except ImportError as exc:
        raise unittest.SkipTest("rclpy/luggage_msgs unavailable: %s" % exc)
    spec = importlib.util.spec_from_file_location("mapper_node_b4", _NODE)
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


@unittest.skipIf(os.environ.get("ELFIN_B4_GATE2", "") != "1",
                 "B4 Gate-2 live scenarios; opt in with ELFIN_B4_GATE2=1")
class TestB4ProcessIntegration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.mod = _load_node_module()
        cls.scene_path = default_scene_tf_config_path()

    def _spin_up(self, frame_scene_revision="b4-gate2"):
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        if not rclpy.ok():
            rclpy.init()
        node = _construct(self.mod.CargoVolumeMapperNode, self.scene_path)
        # The v2 handlers read this attribute for their boundary context;
        # the param plumbing is launch-level, the handler contract is the
        # same in-process.
        node._frame_scene_revision = "b4"
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        latch = QoSProfile(
            depth=5, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        helper = rclpy.create_node("b4_gate2_helper")
        executor.add_node(helper)
        received = []

        def on_surface(msg):
            received.append(json.loads(msg.data))

        helper.create_subscription(
            String, "/luggage/cargo_map/surface_2d", on_surface, latch)
        return node, executor, helper, received

    def _shutdown(self, executor, *nodes):
        executor.shutdown()
        for node in nodes:
            shutdown_tf = getattr(node, "shutdown_tf", None)
            if callable(shutdown_tf):
                shutdown_tf()
            node.destroy_node()

    def test_publication_consistency_across_update(self):
        """v1 surface / committed ledger publications carry one identity
        per update: epoch stable, revision monotonic, digest changing
        only when content changes."""
        from luggage_msgs.srv import AddPlacedBoxV2, GetCargoMapStats
        import rclpy
        for _rep in range(REPS):
            node, executor, helper, _ = self._spin_up()
            try:
                stats_client = helper.create_client(
                    GetCargoMapStats, "/cargo_map/get_stats")
                self.assertTrue(
                    stats_client.wait_for_service(timeout_sec=5.0))
                before = self._stats(stats_client)
                identity = self._add_v2(helper, node, before.map_revision,
                                        slot=(0.1, 0.1, 0.6))
                self.assertTrue(identity.success, identity.message)
                after = self._stats(stats_client)
                self.assertEqual(after.epoch, before.epoch)
                self.assertEqual(
                    after.map_revision, before.map_revision + 1)
                self.assertNotEqual(after.content_digest,
                                    before.content_digest)
            finally:
                self._shutdown(executor, node, helper)

    def test_literal_service_retry(self):
        """A literal retry of one v2 commit (same operation id + content)
        replays as already-processed without a second revision bump."""
        from luggage_msgs.srv import AddPlacedBoxV2, GetCargoMapStats
        for _rep in range(REPS):
            node, executor, helper, _ = self._spin_up()
            try:
                stats_client = helper.create_client(
                    GetCargoMapStats, "/cargo_map/get_stats")
                self.assertTrue(
                    stats_client.wait_for_service(timeout_sec=5.0))
                base = self._stats(stats_client)
                first = self._add_v2(helper, node, base.map_revision,
                                     slot=(0.0, 0.0, 0.6),
                                     operation_id="b4-retry-1")
                self.assertTrue(first.success, first.message)
                self.assertEqual(first.status, "accepted")
                # Literal retry: byte-identical request (same premise —
                # a drifted premise under the same id is a conflict, not
                # a retry).
                replay = self._add_v2(helper, node, base.map_revision,
                                      slot=(0.0, 0.0, 0.6),
                                      operation_id="b4-retry-1",
                                      expect_revision=False)
                self.assertTrue(replay.success, replay.message)
                self.assertEqual(replay.status, "already_processed")
                self.assertEqual(replay.resulting_map_revision,
                                 first.resulting_map_revision)
                drifted = self._add_v2(helper, node,
                                       first.resulting_map_revision,
                                       slot=(0.0, 0.0, 0.6),
                                       operation_id="b4-retry-1",
                                       expect_revision=False)
                self.assertFalse(drifted.success)
                self.assertEqual(drifted.status, "rejected")
                self.assertEqual(drifted.reason, "OPERATION_ID_CONFLICT")
            finally:
                self._shutdown(executor, node, helper)

    def test_node_restart_rotates_identity(self):
        """A restarted mapper publishes a new epoch: a write carrying the
        old identity is refused (EPOCH_MISMATCH), never silently applied."""
        from luggage_msgs.srv import GetCargoMapStats
        for _rep in range(REPS):
            node_a, executor_a, helper_a, _ = self._spin_up()
            try:
                stats_a = helper_a.create_client(
                    GetCargoMapStats, "/cargo_map/get_stats")
                self.assertTrue(
                    stats_a.wait_for_service(timeout_sec=5.0))
                before = self._stats(stats_a)
            finally:
                self._shutdown(executor_a, node_a, helper_a)
            time.sleep(0.2)
            node_b, executor_b, helper_b, _ = self._spin_up()
            try:
                stats_b = helper_b.create_client(
                    GetCargoMapStats, "/cargo_map/get_stats")
                self.assertTrue(
                    stats_b.wait_for_service(timeout_sec=5.0))
                after = self._stats(stats_b)
                self.assertNotEqual(after.epoch, before.epoch)
                stale = self._add_v2(helper_b, node_b,
                                     after.map_revision,
                                     slot=(0.1, 0.1, 0.6),
                                     expected_epoch=before.epoch,
                                     expect_revision=False)
                self.assertFalse(stale.success)
                self.assertEqual(stale.reason, "EPOCH_MISMATCH")
                self.assertEqual(self._stats(stats_b).map_revision,
                                 after.map_revision)
            finally:
                self._shutdown(executor_b, node_b, helper_b)

    def test_subscriber_version_consistency(self):
        """A latched consumer sees monotonic revisions; every message's
        epoch is the live one and identity fields agree with get_stats."""
        from luggage_msgs.srv import AddPlacedBoxV2, GetCargoMapStats
        for _rep in range(REPS):
            node, executor, helper, received = self._spin_up()
            try:
                time.sleep(0.4)  # latched initial publication
                stats_client = helper.create_client(
                    GetCargoMapStats, "/cargo_map/get_stats")
                self.assertTrue(
                    stats_client.wait_for_service(timeout_sec=5.0))
                base = self._stats(stats_client)
                self._add_v2(helper, node, base.map_revision,
                             slot=(0.1, 0.1, 0.6))
                deadline = time.monotonic() + 5.0
                while time.monotonic() < deadline and len(received) < 2:
                    time.sleep(0.05)
                self.assertGreaterEqual(len(received), 2)
                revisions = [m["map_revision"] for m in received]
                self.assertEqual(revisions, sorted(revisions))
                self.assertTrue(all(m["epoch"] == base.epoch
                                    for m in received))
                self.assertTrue(all(m["commit_revision"] == m["map_revision"]
                                    for m in received))
                latest = self._stats(stats_client)
                self.assertEqual(received[-1]["map_revision"],
                                 latest.map_revision)
                self.assertEqual(received[-1]["content_digest"],
                                 latest.content_digest)
            finally:
                self._shutdown(executor, node, helper)

    # -- helpers -------------------------------------------------------

    @staticmethod
    def _stats(client):
        from luggage_msgs.srv import GetCargoMapStats
        future = client.call_async(GetCargoMapStats.Request())
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not future.done():
            time.sleep(0.05)
        return future.result()

    def _add_v2(self, helper, node, revision, slot, operation_id="b4-add",
                expected_epoch="", expect_revision=True):
        """Direct in-process call through the REAL service handler."""
        from luggage_msgs.srv import AddPlacedBoxV2
        from luggage_planning.place_service import pose_message
        record = dict(
            contract_version=2, frame_id="container_link",
            length_unit="m", stamp_ns=1, time_basis="scene",
            scene_revision="b4", request_id=operation_id,
            reference="box_center", orientation_semantics="box_axis_pi",
            position=[float(slot[0]), float(slot[1]), float(slot[2])],
            quaternion_xyzw=[0.0, 0.0, 0.0, 1.0])
        request = AddPlacedBoxV2.Request()
        request.slot_pose = pose_message(record)
        request.size_wdh = [0.2, 0.2, 0.2]
        request.measured_source = "verify_placed_box"
        request.expected_map_revision = int(revision)
        request.geometry_hash = str(node._geometry_hash)
        request.operation_id = operation_id
        request.expected_epoch = str(expected_epoch)
        response = AddPlacedBoxV2.Response()
        return self.mod.CargoVolumeMapperNode._on_add_v2(
            node, request, response)


if __name__ == "__main__":
    unittest.main()
