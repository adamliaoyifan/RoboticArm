#!/usr/bin/env python3
"""Node adapters over the store transaction (Plan B / B1).

Offline stand-in tests (no ROS graph, no real store): a recording store
stub captures the MapUpdate each service handler builds, and the
UpdateResult-to-response mapping is pinned for every status. The gates
themselves (anchor evidence, premise, idempotency) are pinned against the
real store in test_occupancy_map_store*.py.
"""
import importlib.util
import os
import unittest
from types import SimpleNamespace

_HERE = os.path.dirname(os.path.abspath(__file__))
_NODE = os.path.normpath(os.path.join(
    _HERE, "..", "scripts", "cargo_volume_mapper_node.py"))
_ROOT = os.path.normpath(os.path.join(_HERE, "..", "..", ".."))

import sys  # noqa: E402
for _entry in (
        os.path.join(_ROOT, "src", "luggage_planning"),
        os.path.join(_ROOT, "src", "luggage_description"),
        os.path.join(_ROOT, "src", "luggage_perception"),
        _ROOT):
    if _entry not in sys.path:
        sys.path.append(_entry)

from luggage_planning.place.place_service import pose_message  # noqa: E402
from luggage_perception.occupancy_map_store import (  # noqa: E402
    SnapshotIdentity,
    UpdateStatus,
)


def _load():
    try:
        spec = importlib.util.spec_from_file_location("mapper_node_t", _NODE)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    except ImportError as exc:
        raise unittest.SkipTest("mapper node imports unavailable: %s" % exc)
    return module


def _slot_record(request_id="req-1", position=(0.20, -0.10, 0.14)):
    return dict(
        contract_version=2, frame_id="container_link", length_unit="m",
        stamp_ns=1_000_000_000, time_basis="scene",
        scene_revision="scene-1", request_id=request_id,
        reference="box_center", orientation_semantics="box_axis_pi",
        position=list(position), quaternion_xyzw=[0.0, 0.0, 0.0, 1.0])


class _StoreRecorder(object):
    """Captures MapUpdates; returns a scripted UpdateResult."""

    def __init__(self, result_factory):
        self.updates = []
        self.result_factory = result_factory

    def apply_update(self, update):
        self.updates.append(update)
        return self.result_factory(update)


def _result(status, reason="", revision=7, epoch="e9", digest="d9",
            details=None):
    identity = SnapshotIdentity(epoch, revision, digest)
    snapshot = SimpleNamespace(identity=identity)
    return SimpleNamespace(
        status=status, reason=reason, identity=identity, snapshot=snapshot,
        details=details or {},
        accepted=status is UpdateStatus.ACCEPTED)


class TestAddV2Adapter(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.mod = _load()

    def _node(self, result_factory):
        node = SimpleNamespace()
        node._gate_mutation = None
        node._geometry_hash = "gh-live"
        node._store = _StoreRecorder(result_factory)
        node._publish_all = lambda snapshot=None: None
        node._mapper = SimpleNamespace(commit_ledger=lambda: [])
        # Bind the real response mapper (it only needs _publish_all/_mapper).
        node._finish_v2 = self.mod.CargoVolumeMapperNode._finish_v2.__get__(
            node)
        return node

    def _request(self, *, operation_id="", epoch="", digest="",
                 revision=7, tolerance=0.0, verified=(0.0, 0.0, 0.0),
                 source="verify_placed_box"):
        request = SimpleNamespace(
            slot_pose=pose_message(_slot_record()),
            size_wdh=[0.51, 0.33, 0.22],
            measured_source=source,
            expected_map_revision=revision,
            geometry_hash="gh-live",
            verified_center_container=list(verified),
            verify_tolerance_m=float(tolerance),
            operation_id=operation_id,
            expected_epoch=epoch,
            expected_content_digest=digest)
        return request

    def _decode(self, node, request):
        # Slot decoding is input shaping, already covered by the F19
        # contract tests; stub it with the container-frame slot the
        # pose_message record above describes.
        node._decode_slot_v2 = lambda slot_pose, rid: {
            "position": [0.20, -0.10, 0.14],
            "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]}
        return node

    def test_accepted_commit_maps_fields(self):
        node = self._node(lambda u: _result(
            UpdateStatus.ACCEPTED, details={"committed_now": True}))
        node = self._decode(node, None)
        response = SimpleNamespace(success=None, message="", status="",
                                   reason="", epoch="", content_digest="",
                                   resulting_map_revision=0)
        self.mod.CargoVolumeMapperNode._on_add_v2(
            node, self._request(), response)
        update = node._store.updates[0]
        self.assertEqual(update.kind.value, "ADD_BOX_V2")
        self.assertEqual(update.operation_id, "addv2:req-1")
        self.assertEqual(update.geometry_hash, "gh-live")
        self.assertEqual(update.measured_source, "verify_placed_box")
        self.assertEqual(update.box[0], [0.20, -0.10, 0.14])
        # tolerance == 0 → verified_center must NOT be asserted (legacy).
        self.assertIsNone(update.verified_center)
        self.assertTrue(response.success)
        self.assertEqual(response.status, "accepted")
        self.assertEqual(response.epoch, "e9")
        self.assertEqual(response.content_digest, "d9")
        self.assertEqual(response.resulting_map_revision, 7)

    def test_operation_id_and_premise_legs_flow(self):
        node = self._node(lambda u: _result(UpdateStatus.ACCEPTED))
        node = self._decode(node, None)
        response = SimpleNamespace(success=None, message="", status="",
                                   reason="", epoch="", content_digest="",
                                   resulting_map_revision=0)
        self.mod.CargoVolumeMapperNode._on_add_v2(
            node, self._request(operation_id="op-77", epoch="e1",
                                digest="cafe", revision=3,
                                tolerance=0.10,
                                verified=(0.2, -0.1, 0.15)),
            response)
        update = node._store.updates[0]
        self.assertEqual(update.operation_id, "op-77")
        self.assertEqual(update.expected_snapshot,
                         SnapshotIdentity("e1", 3, "cafe"))
        self.assertAlmostEqual(update.verified_center[0], 0.2)

    def test_rejected_result_maps_message(self):
        node = self._node(lambda u: _result(
            UpdateStatus.REJECTED, reason="EVIDENCE_ANCHOR_MISMATCH",
            details={"message": "verify anchor mismatch: slot 0.650 m"}))
        node = self._decode(node, None)
        response = SimpleNamespace(success=None, message="", status="",
                                   reason="", epoch="", content_digest="",
                                   resulting_map_revision=0)
        self.mod.CargoVolumeMapperNode._on_add_v2(
            node, self._request(tolerance=0.10), response)
        self.assertFalse(response.success)
        self.assertEqual(response.status, "rejected")
        self.assertEqual(response.reason, "EVIDENCE_ANCHOR_MISMATCH")
        self.assertIn("0.650 m", response.message)

    def test_already_processed_replay_reports_success(self):
        node = self._node(lambda u: _result(UpdateStatus.ALREADY_PROCESSED))
        node = self._decode(node, None)
        response = SimpleNamespace(success=None, message="", status="",
                                   reason="", epoch="", content_digest="",
                                   resulting_map_revision=0)
        self.mod.CargoVolumeMapperNode._on_add_v2(
            node, self._request(), response)
        self.assertTrue(response.success)
        self.assertEqual(response.status, "already_processed")
        self.assertIn("replayed", response.message)

    def test_missing_geometry_hash_refused_before_store(self):
        node = self._node(lambda u: _result(UpdateStatus.ACCEPTED))
        node = self._decode(node, None)
        request = self._request()
        request.geometry_hash = ""
        response = SimpleNamespace(success=None, message="", status="",
                                   reason="", epoch="", content_digest="",
                                   resulting_map_revision=0)
        self.mod.CargoVolumeMapperNode._on_add_v2(node, request, response)
        self.assertFalse(response.success)
        self.assertIn("geometry_hash required", response.message)
        self.assertEqual(node._store.updates, [])


class TestV1OperationId(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.mod = _load()

    def test_v1_operation_id_deterministic(self):
        make = self.mod.CargoVolumeMapperNode._v1_operation_id
        self.assertEqual(
            make("add_placed_box", [0.1, 0.2, 0.3], [0.2, 0.2, 0.2], 0.0),
            make("add_placed_box", [0.1, 0.2, 0.3], [0.2, 0.2, 0.2], 0.0))
        self.assertNotEqual(
            make("add_placed_box", [0.1, 0.2, 0.3], [0.2, 0.2, 0.2], 0.0),
            make("add_placed_box", [0.1, 0.2, 0.4], [0.2, 0.2, 0.2], 0.0))
        self.assertTrue(make("add_placed_box", [0.1], [0.2], 0.0)
                        .startswith("v1:add_placed_box:"))


if __name__ == "__main__":
    unittest.main()
