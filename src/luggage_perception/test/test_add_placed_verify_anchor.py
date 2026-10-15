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

    def mark_placed_box(self, center, size, yaw=0.0, measured_source=None):
        self.marks.append((list(center), list(size), yaw))
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


if __name__ == "__main__":
    unittest.main()
