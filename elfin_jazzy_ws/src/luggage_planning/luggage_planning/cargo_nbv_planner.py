#!/usr/bin/env python3
"""Next-best-view planner over Cargo occupancy snapshots."""

from __future__ import division

import math


class CargoNBVPlanner:
    """Greedy NBV over pre-defined joint candidates using frontier coverage."""

    def __init__(self, candidates, joint_names, weights, unknown_threshold, max_views):
        self.candidates = candidates
        self.joint_names = list(joint_names)
        self.weights = weights
        self.unknown_threshold = float(unknown_threshold)
        self.max_views = int(max_views)
        self._visited = set()

    def reset(self):
        self._visited = set()

    @staticmethod
    def _joint_delta(current, target):
        if not current or len(current) != len(target):
            return 1.0
        return math.sqrt(sum((c - t) ** 2 for c, t in zip(current, target)))

    def _coverage_score(self, frontier_points, candidate_values):
        if not frontier_points:
            return 0.0
        # Proxy: prefer candidates that are not yet visited; reward more when
        # many frontier points exist (handled externally via frontier_count).
        return float(len(frontier_points))

    def plan_next(self, stats, frontier_points, current_joints, views_used):
        if views_used >= self.max_views:
            return self._done("max views reached")

        unknown_ratio = float(stats.get("unknown_ratio", 1.0))
        if unknown_ratio <= self.unknown_threshold:
            return self._done("unknown below threshold")

        if int(stats.get("frontier_count", 0)) == 0 and unknown_ratio < 0.5:
            return self._done("no frontier remaining")

        best_idx = None
        best_score = -1e9
        coverage_w = self.weights.get("coverage_weight", 1.0)
        path_w = self.weights.get("path_weight", 0.3)
        smooth_w = self.weights.get("smooth_weight", 0.2)

        coverage_base = self._coverage_score(frontier_points, [])

        for idx, candidate in enumerate(self.candidates):
            if idx in self._visited:
                continue
            values = candidate.get("values", [])
            if len(values) != len(self.joint_names):
                continue
            delta = self._joint_delta(current_joints, values)
            score = (
                coverage_w * coverage_base
                - path_w * delta
                - smooth_w * delta
                - 0.01 * idx
            )
            if score > best_score:
                best_score = score
                best_idx = idx

        if best_idx is None:
            return self._done("no remaining NBV candidates")

        self._visited.add(best_idx)
        chosen = self.candidates[best_idx]
        return {
            "done": False,
            "joint_values": chosen["values"],
            "joint_names": self.joint_names,
            "view_index": best_idx,
            "message": "NBV selected %s" % chosen.get("name", "candidate"),
        }

    def _done(self, message):
        return {
            "done": True,
            "joint_values": [],
            "joint_names": self.joint_names,
            "view_index": -1,
            "message": message,
        }
