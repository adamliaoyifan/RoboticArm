"""AddPlacedBoxV2 verify-anchor gate in cargo_volume_mapper_node.

witness_02 root cause: a stale pre-select compute slot was committed
0.65 m from the verified box and became a phantom occupancy footprint.
The gate rejects a supplied verify pose that disagrees with the asserted
slot; tolerance <= 0 keeps the legacy un-anchored commit.

Offline: ``_on_add_v2`` runs on a stand-in object, no ROS graph.
"""
import importlib.util
import math
import os
import unittest
from types import SimpleNamespace

_HERE = os.path.dirname(os.path.abspath(__file__))
_NODE = os.path.normpath(os.path.join(
    _HERE, "..", "scripts", "cargo_volume_mapper_node.py"))


def _load():
    try:
        spec = importlib.util.spec_from_file_location("mapper_node_t", _NODE)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    except ImportError as exc:
        raise unittest.SkipTest("mapper node imports unavailable: %s" % exc)
    return module


class _Mapper(object):
    def __init__(self):
        self.marks = []
        self._revision = 4

    def mark_placed_box(self, center, size, yaw=0.0, measured_source=None,
                        verify_anchor=None):
        self.marks.append((list(center), list(size), yaw, verify_anchor))
        return True

    def commit_ledger(self):
        return list(self.marks)


class TestVerifyAnchorDelta(unittest.TestCase):
    def test_delta_and_missing(self):
        from luggage_perception.cargo_volume_mapper import (
            verify_anchor_delta_m)
        self.assertAlmostEqual(
            verify_anchor_delta_m([0.53, -0.51, 0.639], [0.53, 0.14, 0.639]),
            0.65, places=6)
        self.assertAlmostEqual(
            verify_anchor_delta_m([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]), 0.0)
        self.assertIsNone(verify_anchor_delta_m(None, [0.0, 0.0, 0.0]))
        self.assertIsNone(verify_anchor_delta_m([0.0, 0.0, 0.0], None))


class TestOnAddV2AnchorGate(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.mod = _load()

    def _node(self, slot_position):
        node = SimpleNamespace()
        node._gate_mutation = None
        node._decode_slot_v2 = lambda msg, rid: {
            "position": list(slot_position),
            "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]}
        node._ledger_guard = lambda rev, gh, response=None: True
        node._mapper = _Mapper()
        node._adopt_release = lambda committed_now, footprint: None
        node._publish_all = lambda: None
        return node

    def _request(self, *, verified, tolerance, measured="verify_placed_box"):
        return SimpleNamespace(
            slot_pose=SimpleNamespace(identity=SimpleNamespace(
                request_id="req-1")),
            size_wdh=[0.51, 0.33, 0.22],
            measured_source=measured,
            expected_map_revision=3,
            geometry_hash="gh",
            verified_center_container=list(verified),
            verify_tolerance_m=float(tolerance))

    def _response(self):
        return SimpleNamespace(success=None, message="",
                               resulting_map_revision=0)

    def test_anchored_match_commits(self):
        node = self._node([0.53, 0.14, 0.639])
        response = self._response()
        self.mod.CargoVolumeMapperNode._on_add_v2(
            node, self._request(verified=[0.53, 0.14, 0.64],
                                tolerance=0.10), response)
        self.assertTrue(response.success)
        self.assertEqual(len(node._mapper.marks), 1)

    def test_witness02_phantom_is_rejected(self):
        node = self._node([0.53, -0.51, 0.639])   # stale compute slot
        response = self._response()
        self.mod.CargoVolumeMapperNode._on_add_v2(
            node, self._request(verified=[0.53, 0.14, 0.64],
                                tolerance=0.10), response)
        self.assertFalse(response.success)
        self.assertIn("verify anchor mismatch", response.message)
        self.assertEqual(node._mapper.marks, [])

    def test_legacy_unanchored_commit_unchanged(self):
        node = self._node([0.53, -0.51, 0.639])
        response = self._response()
        self.mod.CargoVolumeMapperNode._on_add_v2(
            node, self._request(verified=[0.0, 0.0, 0.0], tolerance=0.0),
            response)
        self.assertTrue(response.success)
        self.assertEqual(len(node._mapper.marks), 1)

    def test_nonfinite_anchor_is_rejected(self):
        node = self._node([0.53, 0.14, 0.639])
        response = self._response()
        self.mod.CargoVolumeMapperNode._on_add_v2(
            node, self._request(verified=[float("nan"), 0.0, 0.0],
                                tolerance=0.10), response)
        self.assertFalse(response.success)
        self.assertIn("not finite", response.message)
        self.assertEqual(node._mapper.marks, [])

    def test_nan_tolerance_is_rejected_not_silently_legacy(self):
        """NaN > 0 compares False; without the finite check it would skip
        the gate entirely and commit un-anchored (probe-confirmed)."""
        node = self._node([0.53, -0.51, 0.639])   # 0.65 m off
        response = self._response()
        self.mod.CargoVolumeMapperNode._on_add_v2(
            node, self._request(verified=[0.53, 0.14, 0.64],
                                tolerance=float("nan")), response)
        self.assertFalse(response.success)
        self.assertIn("must be finite", response.message)
        self.assertEqual(node._mapper.marks, [])

    def test_infinite_tolerance_is_rejected(self):
        node = self._node([0.53, -0.51, 0.639])
        response = self._response()
        self.mod.CargoVolumeMapperNode._on_add_v2(
            node, self._request(verified=[0.53, 0.14, 0.64],
                                tolerance=float("inf")), response)
        self.assertFalse(response.success)
        self.assertEqual(node._mapper.marks, [])

    def test_tolerance_above_mapper_ceiling_is_rejected(self):
        node = self._node([0.53, -0.51, 0.639])
        response = self._response()
        self.mod.CargoVolumeMapperNode._on_add_v2(
            node, self._request(verified=[0.53, 0.14, 0.64],
                                tolerance=self.mod
                                .VERIFY_ANCHOR_TOLERANCE_MAX_M + 0.01),
            response)
        self.assertFalse(response.success)
        self.assertIn("ceiling", response.message)
        self.assertEqual(node._mapper.marks, [])

    def test_anchored_commit_records_anchor_on_ledger(self):
        node = self._node([0.53, 0.14, 0.639])
        response = self._response()
        self.mod.CargoVolumeMapperNode._on_add_v2(
            node, self._request(verified=[0.53, 0.14, 0.64],
                                tolerance=0.10), response)
        self.assertTrue(response.success)
        _center, _size, _yaw, anchor = node._mapper.marks[0]
        self.assertEqual(anchor["request_id"], "req-1")
        self.assertAlmostEqual(anchor["delta_m"], 0.001, places=6)
        self.assertEqual(anchor["tolerance_m"], 0.10)

    def test_legacy_commit_carries_no_anchor(self):
        node = self._node([0.53, 0.14, 0.639])
        response = self._response()
        self.mod.CargoVolumeMapperNode._on_add_v2(
            node, self._request(verified=[0.0, 0.0, 0.0], tolerance=0.0),
            response)
        self.assertTrue(response.success)
        _center, _size, _yaw, anchor = node._mapper.marks[0]
        self.assertIsNone(anchor)


class TestMapperLedgerVerifyAnchor(unittest.TestCase):
    """Real CargoVolumeMapper: anchor trail persistence + validation."""

    def _mapper(self):
        from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
        return CargoVolumeMapper(
            inner_size=[1.0, 1.0, 1.0],
            center_base=[0.0, 0.0, 0.5],
            yaw=0.0,
            resolution=0.1,
        )

    def test_ledger_snapshot_carries_verify_anchor(self):
        mapper = self._mapper()
        self.assertTrue(mapper.mark_placed_box(
            [0.0, 0.0, 0.15], [0.2, 0.2, 0.2],
            measured_source="verify_placed_box",
            verify_anchor={"request_id": "req-9", "delta_m": 0.04,
                           "tolerance_m": 0.10}))
        entry = mapper.commit_ledger()[0]
        self.assertEqual(entry["verify_anchor"]["request_id"], "req-9")
        self.assertEqual(entry["measured_source"], "verify_placed_box")

    def test_out_of_tolerance_anchor_is_refused(self):
        mapper = self._mapper()
        with self.assertRaises(ValueError):
            mapper.mark_placed_box(
                [0.0, 0.0, 0.15], [0.2, 0.2, 0.2],
                verify_anchor={"delta_m": 0.65, "tolerance_m": 0.10})
        self.assertEqual(mapper.commit_ledger(), [])


if __name__ == "__main__":
    unittest.main()
