#!/usr/bin/env python3
"""SupportIndex unit tests (G3 STRUCTURAL-CORE §3).

The vectorized windowed max must be bit-identical to the solver's cell
scan; stats memo must be shared (one evaluation per window); plateau
extraction must be scan-order deterministic and never lift the floor or
fill unknown gaps.
"""

import os
import sys
import unittest

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from test_placement_hull_gates import (  # noqa: E402
    BOX, DEPLOY_DESCRIPTOR, KERNEL, uld_map)
from luggage_packing.placement_solver import (  # noqa: E402
    _default_params, _window_stats)
from luggage_packing.placement_support_index import (  # noqa: E402
    SupportIndex, plateau_components, structural_neighbor_records)


def _index(surface, yaws=(0.0, 1.5707963267948966)):
    return SupportIndex(
        surface, BOX, list(yaws), _default_params())


class TestSupportIndex(unittest.TestCase):

    def test_wmax_bit_identical_to_cell_scan(self):
        for res in (0.05, 0.02):
            surface = uld_map(res=res, descriptor=DEPLOY_DESCRIPTOR)
            # A step pattern so windowed max is non-trivial.
            nx, ny = surface["nx"], surface["ny"]
            for ix in range(nx):
                for iy in range(ny):
                    surface["height"][ix][iy] = 0.1 * ((ix * 7 + iy * 3) % 5)
            index = _index(surface)
            for cls in index.classes():
                wmax = index.window_peak_prefilter(cls)
                cells_x, cells_y = index.window_cells(cls)
                rng_x, rng_y = index.window_origin_range(cls)
                for ix0 in rng_x:
                    for iy0 in rng_y:
                        scan_peak, _u, _h, _c = _window_stats(
                            surface, ix0, iy0, cells_x, cells_y)
                        self.assertEqual(
                            float(wmax[ix0, iy0]), float(scan_peak),
                            (res, cls, ix0, iy0))

    def test_stats_memo_shared_between_calls(self):
        surface = uld_map(res=0.05, descriptor=DEPLOY_DESCRIPTOR)
        index = _index(surface)
        first = index.window_stats(0, 3, 4)
        second = index.window_stats(0, 3, 4)
        self.assertIs(first, second)
        self.assertEqual(index.stats_memo_size(), 1)

    def test_identity_is_stable_and_input_bound(self):
        surface = uld_map(res=0.05, descriptor=DEPLOY_DESCRIPTOR)
        a = _index(surface).identity
        b = _index(uld_map(res=0.05, descriptor=DEPLOY_DESCRIPTOR)).identity
        self.assertEqual(a, b)
        other = _index(uld_map(res=0.05, descriptor=None)).identity
        self.assertNotEqual(a["map_geometry_hash"],
                            other["map_geometry_hash"])

    def test_plateaus_scan_order_deterministic(self):
        surface = uld_map(res=0.05, state="sensor", height=0.0,
                          descriptor=DEPLOY_DESCRIPTOR)
        nx, ny = surface["nx"], surface["ny"]
        for ix in range(10, 20):
            for iy in range(10, 16):
                surface["height"][ix][iy] = 0.30
        for ix in range(22, 28):
            for iy in range(20, 24):
                surface["height"][ix][iy] = 0.60
        runs = [plateau_components(surface) for _ in range(3)]
        self.assertEqual(runs[0], runs[1])
        self.assertEqual(runs[0], runs[2])
        self.assertEqual([p["z_top_rel"] for p in runs[0]],
                         [0.30, 0.60])

    def test_floor_is_never_a_plateau(self):
        surface = uld_map(res=0.05, descriptor=DEPLOY_DESCRIPTOR)
        self.assertEqual([], plateau_components(surface))
        records = structural_neighbor_records(surface)
        self.assertEqual([], records)

    def test_neighbor_records_carry_measured_tops(self):
        surface = uld_map(res=0.05, state="sensor", height=0.0,
                          descriptor=DEPLOY_DESCRIPTOR)
        for ix in range(10, 20):
            for iy in range(10, 16):
                surface["height"][ix][iy] = 0.30
        records = structural_neighbor_records(surface)
        self.assertEqual(len(records), 1)
        nb = records[0]
        floor_container_z = surface["center_base"][2] - 0.5 * 1.48
        self.assertAlmostEqual(nb["top_z"],
                               floor_container_z + 0.30, places=9)
        self.assertAlmostEqual(nb["half_l"], 0.25, places=6)
        self.assertAlmostEqual(nb["half_w"], 0.15, places=6)


if __name__ == "__main__":
    unittest.main()
