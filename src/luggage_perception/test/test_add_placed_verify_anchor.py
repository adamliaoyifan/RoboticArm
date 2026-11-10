"""Verify-anchor primitives for AddPlacedBoxV2 commits.

The node-level gate moved into OccupancyMapStore.apply_update (B1): the
behavior cases — anchored match, witness_02 0.65 m phantom, legacy
un-anchored, NaN/Inf/negative/over-ceiling tolerance, ledger trail — are
pinned against the real store in test_occupancy_map_store_anchor.py, and
the adapter wiring (request → MapUpdate → response) in
test_cargo_map_node_adapters.py. What remains here are the pure
delta primitive and the mapper-ledger anchor persistence.
"""
import math
import unittest

from luggage_perception.cargo_volume_mapper import (
    CargoVolumeMapper,
    verify_anchor_delta_m,
)


class TestVerifyAnchorDelta(unittest.TestCase):
    def test_delta_and_missing(self):
        self.assertAlmostEqual(
            verify_anchor_delta_m([0.53, -0.51, 0.639], [0.53, 0.14, 0.639]),
            0.65, places=6)
        self.assertAlmostEqual(
            verify_anchor_delta_m([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]), 0.0)
        self.assertIsNone(verify_anchor_delta_m(None, [0.0, 0.0, 0.0]))
        self.assertIsNone(verify_anchor_delta_m([0.0, 0.0, 0.0], None))

    def test_witness_geometry_is_0_65(self):
        delta = verify_anchor_delta_m(
            [0.53, -0.51, 0.639], [0.53, 0.14, 0.639])
        self.assertTrue(math.isclose(delta, 0.65, abs_tol=1e-6))


class TestMapperLedgerVerifyAnchor(unittest.TestCase):
    """Real CargoVolumeMapper: anchor trail persistence + validation."""

    def _mapper(self):
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
