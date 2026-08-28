#!/usr/bin/env python3
import os
import sys
import unittest


from luggage_perception.voxel_log_odds import (  # noqa: E402
    FREE,
    OCCUPIED,
    UNKNOWN,
    LogOddsGrid,
    log_odds_to_probability,
    probability_to_log_odds,
)


class TestVoxelLogOdds(unittest.TestCase):
    def test_probability_roundtrip(self):
        for probability in (0.1, 0.4, 0.5, 0.7, 0.9):
            result = log_odds_to_probability(
                probability_to_log_odds(probability)
            )
            self.assertAlmostEqual(result, probability)

    def test_requires_repeated_hits_and_misses_can_clear(self):
        grid = LogOddsGrid(
            1, occupied_threshold=1.2, free_threshold=-0.35
        )
        self.assertEqual(grid.apply_hit(0), UNKNOWN)
        self.assertEqual(grid.apply_hit(0), OCCUPIED)
        state = OCCUPIED
        for _unused in range(6):
            state = grid.apply_miss(0)
        self.assertIn(state, (UNKNOWN, FREE))

    def test_clamps_and_geometry_lock(self):
        grid = LogOddsGrid(1, log_odds_min=-1.0, log_odds_max=1.0)
        for _unused in range(100):
            grid.apply_hit(0)
        self.assertEqual(grid.values[0], 1.0)
        grid.force_occupied(0, lock_geometry=True)
        for _unused in range(100):
            grid.apply_miss(0)
        self.assertEqual(grid.state_at(0), OCCUPIED)
        self.assertEqual(grid.values[0], 1.0)

    def test_clear_returns_unknown(self):
        grid = LogOddsGrid(1)
        grid.force_occupied(0)
        grid.clear(0)
        self.assertEqual(grid.state_at(0), UNKNOWN)


if __name__ == "__main__":
    unittest.main()
