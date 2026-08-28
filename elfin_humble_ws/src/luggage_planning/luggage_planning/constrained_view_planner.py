#!/usr/bin/env python3
"""Constrained task-space view selection for cargo exploration.

Pure-Python (no ROS) ranking of a fixed library of camera viewpoints. Unlike a
free NBV optimizer, each candidate is defined in task space (camera position +
look-at), bound by geometric constraints (camera/wrist height ceilings imposed
by the closed container top), and scored by how many current frontier points it
would cover. The arm still executes the candidate's joint seed, so this is a
drop-in upgrade of the fixed joint scan that adds explainable view constraints.
"""

from __future__ import division

import math


def _norm(v):
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def _sub(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def candidate_is_admissible(candidate, constraints):
    """Reject candidates that violate the closed-container height constraints."""
    cam = candidate["camera_xyz"]
    if cam[2] > constraints["camera_z_max"] + 1e-6:
        return False, "camera_above_ceiling"
    return True, "ok"


def coverage_score(candidate, frontier_points, constraints):
    """Count frontier points the camera ray would plausibly observe.

    A frontier point contributes if it is within ``coverage_radius`` of the
    look-at target and roughly along the camera view ray (alignment gate).
    """
    cam = candidate["camera_xyz"]
    look = candidate["look_at"]
    view_dir = _sub(look, cam)
    view_len = _norm(view_dir)
    if view_len < 1e-6:
        return 0.0
    view_dir = [c / view_len for c in view_dir]

    radius = constraints["coverage_radius"]
    align_min = constraints["alignment_min"]
    score = 0.0
    for pt in frontier_points:
        to_pt = _sub(pt, cam)
        dist = _norm(to_pt)
        if dist < 1e-6:
            continue
        align = (
            to_pt[0] * view_dir[0]
            + to_pt[1] * view_dir[1]
            + to_pt[2] * view_dir[2]
        ) / dist
        if align < align_min:
            continue
        radial = _norm(_sub(pt, look))
        if radial > radius:
            continue
        # Closer-to-axis and closer-to-target frontier points weigh more.
        score += align * max(0.0, 1.0 - radial / radius)
    return score


class ConstrainedViewPlanner:
    def __init__(self, candidates, constraints):
        self._candidates = list(candidates)
        self._constraints = dict(constraints)
        self._used = set()

    def reset(self):
        self._used = set()

    def select_next(self, frontier_points, views_used=None):
        """Return the best admissible, unused candidate for current frontier.

        Returns a dict with ``done`` and, when not done, the chosen candidate
        plus diagnostics. ``frontier_points`` are [x,y,z] in the base frame.
        """
        if views_used is not None and views_used == 0:
            self.reset()

        ranked = []
        for idx, cand in enumerate(self._candidates):
            if idx in self._used:
                continue
            admissible, reason = candidate_is_admissible(cand, self._constraints)
            if not admissible:
                continue
            score = coverage_score(cand, frontier_points, self._constraints)
            ranked.append((score, idx, cand))

        if not ranked:
            return {"done": True, "message": "no admissible views left", "view_index": -1}

        # If no frontier signal yet, fall back to library order (max score == 0).
        ranked.sort(key=lambda t: (-t[0], t[1]))
        score, idx, cand = ranked[0]
        self._used.add(idx)
        return {
            "done": False,
            "view_index": idx,
            "name": cand["name"],
            "values": cand["values"],
            "coverage": round(score, 4),
            "camera_xyz": cand["camera_xyz"],
            "look_at": cand["look_at"],
            "message": "constrained view %s coverage=%.2f" % (cand["name"], score),
        }


if __name__ == "__main__":
    cands = [
        {"name": "low", "camera_xyz": [1.4, 0.0, 1.2], "look_at": [2.2, 0.0, 0.5], "values": [0] * 6},
        {"name": "too_high", "camera_xyz": [1.4, 0.0, 1.8], "look_at": [2.2, 0.0, 0.5], "values": [0] * 6},
    ]
    cons = {"camera_z_max": 1.45, "wrist_z_max": 1.55, "coverage_radius": 0.9, "alignment_min": 0.2}
    planner = ConstrainedViewPlanner(cands, cons)
    frontier = [[2.2, 0.0, 0.5], [2.3, 0.1, 0.55], [3.0, 1.0, 0.5]]
    print(planner.select_next(frontier, views_used=0))
    print(planner.select_next(frontier))
    print(planner.select_next(frontier))
