#!/usr/bin/env python3
"""Shared log-odds occupancy math for cargo and world voxel grids."""

from __future__ import division

import math

UNKNOWN = 0
FREE = 1
OCCUPIED = 2


def probability_to_log_odds(probability):
    probability = min(1.0 - 1e-9, max(1e-9, float(probability)))
    return math.log(probability / (1.0 - probability))


def log_odds_to_probability(value):
    value = float(value)
    if value >= 0.0:
        exp_neg = math.exp(-value)
        return 1.0 / (1.0 + exp_neg)
    exp_pos = math.exp(value)
    return exp_pos / (1.0 + exp_pos)


class LogOddsGrid:
    def __init__(
        self,
        size,
        p_hit=0.70,
        p_miss=0.40,
        log_odds_min=-2.0,
        log_odds_max=3.5,
        occupied_threshold=0.50,
        free_threshold=-0.35,
        enabled=True,
    ):
        self.size = int(size)
        self.enabled = bool(enabled)
        self.hit_increment = probability_to_log_odds(p_hit)
        self.miss_increment = probability_to_log_odds(p_miss)
        self.minimum = float(log_odds_min)
        self.maximum = float(log_odds_max)
        self.occupied_threshold = float(occupied_threshold)
        self.free_threshold = float(free_threshold)
        if self.free_threshold >= self.occupied_threshold:
            raise ValueError("free_threshold must be below occupied_threshold")
        self.values = [0.0] * self.size
        self.geometry_locked = [False] * self.size

    def reset(self):
        self.values = [0.0] * self.size
        self.geometry_locked = [False] * self.size

    def classify_value(self, value):
        if value >= self.occupied_threshold:
            return OCCUPIED
        if value <= self.free_threshold:
            return FREE
        return UNKNOWN

    def state_at(self, index):
        return self.classify_value(self.values[index])

    def apply_hit(self, index):
        if not self.enabled:
            self.values[index] = self.maximum
            return OCCUPIED
        value = self.values[index] + self.hit_increment
        self.values[index] = min(self.maximum, max(self.minimum, value))
        return self.state_at(index)

    def apply_miss(self, index):
        if self.geometry_locked[index]:
            return self.state_at(index)
        if not self.enabled:
            self.values[index] = self.minimum
            return FREE
        value = self.values[index] + self.miss_increment
        self.values[index] = min(self.maximum, max(self.minimum, value))
        return self.state_at(index)

    def force_occupied(self, index, lock_geometry=True):
        self.values[index] = self.maximum
        if lock_geometry:
            self.geometry_locked[index] = True
        return OCCUPIED

    def clear(self, index):
        self.values[index] = 0.0
        self.geometry_locked[index] = False

    def states(self):
        return [self.classify_value(value) for value in self.values]

    def diagnostics(self):
        if not self.values:
            return {"min": 0.0, "max": 0.0, "mean_probability": 0.5}
        return {
            "min": min(self.values),
            "max": max(self.values),
            "mean_probability": sum(
                log_odds_to_probability(value) for value in self.values
            ) / float(len(self.values)),
        }
