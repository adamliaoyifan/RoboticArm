#!/usr/bin/env python3
"""Point-cloud realism filter simulating RealSense D435 depth degradation.

Pure-Python (no ROS) so it can be unit tested directly. Apply after
transforming points to base frame but before integration into the voxel map.
"""

from __future__ import division

import math
import random


class DepthRealismFilter:
    """Degrade ideal Gazebo depth points to approximate real D435 behavior."""

    def __init__(
        self,
        enabled=True,
        max_reliable_range=2.5,
        hard_max_range=3.0,
        range_noise_sigma=0.004,
        dropout_rate=0.02,
        edge_dropout_rate=0.0,
        random_seed=None,
    ):
        self.enabled = bool(enabled)
        self.max_reliable_range = float(max_reliable_range)
        self.hard_max_range = float(hard_max_range)
        self.range_noise_sigma = float(range_noise_sigma)
        self.dropout_rate = float(dropout_rate)
        self.edge_dropout_rate = float(edge_dropout_rate)
        self._rng = random.Random(random_seed)
        self._last_stats = {
            "raw_count": 0,
            "filtered_count": 0,
            "dropped_range": 0,
            "dropped_dropout": 0,
            "noise_applied": 0,
        }

    @property
    def last_stats(self):
        return dict(self._last_stats)

    def filter_points(self, points, origin=None):
        """Filter a list of (x, y, z) points in base frame.

        Args:
            points: list of (x, y, z) tuples in base frame.
            origin: (ox, oy, oz) camera position in base frame for range calc.
                    If None, range is computed as distance from world origin.

        Returns:
            Filtered list of (x, y, z) tuples.
        """
        if not self.enabled:
            self._last_stats = {
                "raw_count": len(points),
                "filtered_count": len(points),
                "dropped_range": 0,
                "dropped_dropout": 0,
                "noise_applied": 0,
            }
            return list(points)

        ox, oy, oz = origin if origin else (0.0, 0.0, 0.0)
        result = []
        dropped_range = 0
        dropped_dropout = 0
        noise_applied = 0

        for x, y, z in points:
            dx = x - ox
            dy = y - oy
            dz = z - oz
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)

            if dist > self.hard_max_range:
                dropped_range += 1
                continue

            if dist > self.max_reliable_range:
                drop_prob = (dist - self.max_reliable_range) / (
                    self.hard_max_range - self.max_reliable_range
                )
                if self._rng.random() < drop_prob:
                    dropped_range += 1
                    continue

            if self.dropout_rate > 0.0 and self._rng.random() < self.dropout_rate:
                dropped_dropout += 1
                continue

            if self.range_noise_sigma > 0.0 and dist > 0.01:
                sigma = self.range_noise_sigma * dist
                noise = self._rng.gauss(0.0, sigma)
                scale = (dist + noise) / dist
                x = ox + dx * scale
                y = oy + dy * scale
                z = oz + dz * scale
                noise_applied += 1

            result.append((x, y, z))

        self._last_stats = {
            "raw_count": len(points),
            "filtered_count": len(result),
            "dropped_range": dropped_range,
            "dropped_dropout": dropped_dropout,
            "noise_applied": noise_applied,
        }
        return result
