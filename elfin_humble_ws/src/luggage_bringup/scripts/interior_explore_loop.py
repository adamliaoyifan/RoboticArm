#!/usr/bin/env python3
"""Deterministic state core for bounded, receding-horizon interior exploration."""

from __future__ import division

import time


BOOTSTRAP_OPENING = "BOOTSTRAP_OPENING"
SELECT_CORRIDOR = "SELECT_CORRIDOR"
ENTER_APERTURE = "ENTER_APERTURE"
OBSERVE_INSIDE = "OBSERVE_INSIDE"
REPLAN_DEPTH = "REPLAN_DEPTH"
RETREAT = "RETREAT"
DONE = "DONE"
FAULT = "FAULT"

# Legal state transitions (see docs/plans/archive/2026-08/urdf_self_filter_task_roi_execution_plan.md
# section 7.3). Used by the bag harness to validate recorded loop events.
LEGAL_TRANSITIONS = {
    BOOTSTRAP_OPENING: (SELECT_CORRIDOR, FAULT),
    SELECT_CORRIDOR: (ENTER_APERTURE, DONE, FAULT),
    ENTER_APERTURE: (OBSERVE_INSIDE, SELECT_CORRIDOR, RETREAT, FAULT),
    OBSERVE_INSIDE: (REPLAN_DEPTH, RETREAT),
    REPLAN_DEPTH: (ENTER_APERTURE, RETREAT),
    RETREAT: (DONE, FAULT),
    DONE: (BOOTSTRAP_OPENING,),
    FAULT: (BOOTSTRAP_OPENING,),
}

# Monotonic session id so each reset() starts a distinguishable session without
# depending on ROS or wall-clock randomness (keeps the core deterministic).
_SESSION_COUNTER = [0]


class InteriorExploreLoop:
    """Track loop invariants without depending on ROS or MoveIt."""

    def __init__(
            self, max_depth_steps=4, max_views=12, max_seconds=120.0,
            max_candidate_attempts=4, stagnation_limit=2, clock=None):
        self.max_depth_steps = max(1, int(max_depth_steps))
        self.max_views = max(1, int(max_views))
        self.max_seconds = max(0.0, float(max_seconds))
        self.max_candidate_attempts = max(1, int(max_candidate_attempts))
        self.stagnation_limit = max(1, int(stagnation_limit))
        self._clock = clock or time.monotonic
        self.reset()

    def reset(self):
        _SESSION_COUNTER[0] += 1
        self.session_id = _SESSION_COUNTER[0]
        self.sequence = 0
        self.state = BOOTSTRAP_OPENING
        self.started_at = self._clock()
        self.inserted = False
        self.lane_id = None
        self.depth = 0.0
        self.depth_steps = 0
        self.views = 0
        self.candidate_attempts = 0
        self.stagnation_count = 0
        self.pending_candidate_id = None
        self.rejected_candidate_ids = set()
        self.map_revision = None
        self.terminal_reason = ""

    def _budget_reason(self):
        if self.max_seconds and self._clock() - self.started_at >= self.max_seconds:
            return "time_budget"
        if self.views >= self.max_views:
            return "view_budget"
        if self.depth_steps >= self.max_depth_steps:
            return "depth_step_budget"
        if self.candidate_attempts >= self.max_candidate_attempts:
            return "candidate_budget"
        if self.stagnation_count >= self.stagnation_limit:
            return "stagnation"
        return ""

    def _set_retreat(self, reason):
        """Move to RETREAT/DONE without bumping the event sequence.

        Internal callers (observation_committed, select_candidate) use this so a
        single orchestrator call produces one event; the public
        ``request_retreat`` bumps the sequence for direct orchestrator calls.
        """
        self.terminal_reason = str(reason)
        self.state = RETREAT if self.inserted else DONE
        return self.state

    def snapshot(self):
        """Return a deterministic, non-mutating view of the loop state.

        The orchestrator publishes this (enriched with perception/planning
        fields) as a loop event after each state-machine call. Pure inspection;
        does not modify any field.
        """
        return {
            "session_id": self.session_id,
            "sequence": self.sequence,
            "state": self.state,
            "inserted": self.inserted,
            "candidate_id": self.pending_candidate_id,
            "lane_id": self.lane_id,
            "insertion_depth": self.depth,
            "map_revision": self.map_revision,
            "views": self.views,
            "depth_steps": self.depth_steps,
            "candidate_attempts": self.candidate_attempts,
            "stagnation_count": self.stagnation_count,
            "rejected_candidate_ids": sorted(self.rejected_candidate_ids),
            "terminal_reason": self.terminal_reason,
            "elapsed_sec": self._clock() - self.started_at,
            "budgets": {
                "max_views": self.max_views,
                "max_depth_steps": self.max_depth_steps,
                "max_candidate_attempts": self.max_candidate_attempts,
                "stagnation_limit": self.stagnation_limit,
                "max_seconds": self.max_seconds,
            },
        }

    def geometry_ready(self, valid, reason=""):
        if self.state != BOOTSTRAP_OPENING:
            raise ValueError("geometry_ready only valid during bootstrap")
        self.sequence += 1
        if not valid:
            self.terminal_reason = reason or "invalid_geometry"
            self.state = FAULT
            return self.state
        self.state = SELECT_CORRIDOR
        return self.state

    def select_candidate(
            self, candidate_id, lane_id, depth, retreat_valid,
            hard_feasible=True):
        if self.state not in (SELECT_CORRIDOR, REPLAN_DEPTH):
            raise ValueError("candidate selection outside selection state")
        self.sequence += 1
        candidate_id = str(candidate_id)
        if candidate_id in self.rejected_candidate_ids:
            raise ValueError("rejected candidate cannot be selected again")
        self.candidate_attempts += 1
        if not hard_feasible or not retreat_valid:
            self.rejected_candidate_ids.add(candidate_id)
            self.terminal_reason = (
                "retreat_invalid" if not retreat_valid else "hard_constraint")
            budget = self._budget_reason()
            if budget:
                self._set_retreat(budget)
            return self.state
        if self.inserted:
            if lane_id != self.lane_id:
                self._set_retreat("lane_change_requires_retreat")
                return self.state
            if float(depth) <= self.depth:
                self.rejected_candidate_ids.add(candidate_id)
                self.terminal_reason = "non_monotonic_depth"
                return self.state
        self.pending_candidate_id = candidate_id
        self.lane_id = lane_id
        self.depth = float(depth)
        self.state = ENTER_APERTURE
        return self.state

    def motion_failed(self, reason):
        self.sequence += 1
        if self.pending_candidate_id is not None:
            self.rejected_candidate_ids.add(self.pending_candidate_id)
        self.pending_candidate_id = None
        self.terminal_reason = str(reason or "motion_failed")
        if self.inserted:
            self.state = RETREAT
        elif self._budget_reason():
            self.state = FAULT
        else:
            self.state = SELECT_CORRIDOR
        return self.state

    def entered(self):
        if self.state != ENTER_APERTURE or self.pending_candidate_id is None:
            raise ValueError("entered without pending candidate")
        self.sequence += 1
        self.inserted = True
        self.depth_steps += 1
        self.state = OBSERVE_INSIDE
        return self.state

    def observation_committed(
            self, map_revision, improvement, geometry_valid=True,
            done=False):
        if self.state != OBSERVE_INSIDE:
            raise ValueError("observation outside observe state")
        self.sequence += 1
        if not geometry_valid:
            return self._set_retreat("geometry_stale")
        # Strictly increasing map revision: non-increasing (<=) is a failure,
        # not just equality. First observation (previous None) is allowed.
        if self.map_revision is not None and map_revision <= self.map_revision:
            return self._set_retreat("map_not_advanced")
        self.map_revision = map_revision
        self.views += 1
        self.pending_candidate_id = None
        self.candidate_attempts = 0
        if float(improvement) <= 0.0:
            self.stagnation_count += 1
        else:
            self.stagnation_count = 0
        reason = "done" if done else self._budget_reason()
        if reason:
            return self._set_retreat(reason)
        self.state = REPLAN_DEPTH
        return self.state

    def request_retreat(self, reason):
        self.sequence += 1
        return self._set_retreat(reason)

    def retreated(self, success, reason=""):
        if self.state != RETREAT:
            raise ValueError("retreated outside retreat state")
        self.sequence += 1
        self.inserted = False
        self.pending_candidate_id = None
        if success:
            self.state = DONE
        else:
            self.state = FAULT
            self.terminal_reason = reason or "retreat_failed"
        return self.state
