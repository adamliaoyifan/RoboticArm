#!/usr/bin/env python3
"""ST09-B bounded route search over the q-state graph (Section B).

[ST09-I] The legacy Section A profile/Dijkstra search was deleted with
the geometric selection bypass; route feasibility is decided here under
the frozen budgets, and MoveIt-internal Cartesian/RRTConnect generation
stays inside the edge builder.
"""

# -- Section B: ST09-B bounded route search over the q-state graph -------

#: Frozen planning budgets (a B test asserts equality with
#: st09_contracts/resolved_planning_contract.json).
from __future__ import division

import json
import math
import time
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Sequence, Tuple

ROUTE_TOTAL_WALL_S = 10.0
SINGLE_NEW_EDGE_WALL_S = 2.0
MAX_EDGE_REPAIR_ATTEMPTS = 4

#: B-defined dev policy (not frozen): fraction of the total wall reserved
#: for the post-release retreat chain so a full entry never starves exit.
RETREAT_MIN_SHARE = 0.3

#: Edge-class strings the MoveIt adapter accepts (moveit_edge_adapter
#: owns the enum; this module stays ROS-free and compares strings).
EDGE_CLASSES = ("PORTAL_TRANSIT", "ORIENTATION_TRANSITION", "TRAVERSE",
                "INSERT", "DESCEND", "RETREAT", "RETURN")

#: Canonical hop tables for the small route graph (portal / clearance /
#: slot approach / exit / return). The final entry hop lands on the goal
#: node itself (the DESCEND endpoint q); ``slot_upper`` is the intermediate
#: inserted-attitude node above the goal; the first retreat hop starts
#: from the release twin.
DEFAULT_ENTRY_HOPS = (("portal", "PORTAL_TRANSIT"),
                      ("slot_approach", "TRAVERSE"),
                      ("slot_upper", "INSERT"),
                      ("slot", "DESCEND"))
DEFAULT_RETREAT_HOPS = (("exit", "RETREAT"), ("home", "RETURN"))


def _reason(name):
    from st09_contracts.reasons import Reason
    return Reason(str(name))


class RoutePlanningError(Exception):
    """Named route-search refusal (frozen Reason taxonomy).

    ``detail`` carries per-edge failures, prior gaps, and budget numbers
    for evidence; budget exhaustion is PLANNING_BUDGET_EXHAUSTED and never
    BIN_FULL.
    """

    def __init__(self, reason, detail=None):
        self.reason = _reason(reason)
        self.detail = dict(detail or {})
        super(RoutePlanningError, self).__init__(
            "%s: %s" % (self.reason.value, self.detail))

    @property
    def code(self):
        return self.reason.value


def map_edge_failure(kind, detail=None):
    """Map one edge-build failure kind onto the frozen Reason set."""
    if kind == "budget":
        return _reason("PLANNING_BUDGET_EXHAUSTED")
    if kind == "no_goal":
        return _reason("NO_FEASIBLE_ROUTE")
    return _reason("EDGE_BUILD_FAILED")


@dataclass(frozen=True)
class RouteBudgetPolicy:
    """Budget numbers; defaults mirror the frozen planning contract."""

    total_wall_s: float = ROUTE_TOTAL_WALL_S
    single_edge_wall_s: float = SINGLE_NEW_EDGE_WALL_S
    max_edge_repair_attempts: int = MAX_EDGE_REPAIR_ATTEMPTS
    retreat_min_share: float = RETREAT_MIN_SHARE


class RouteBudget(object):
    """One shared ledger across entry and retreat chains.

    Attempts count only for edges actually (re)built — a verified cache
    hit costs neither an attempt nor wall time, and neither does a cheap
    re-probe of a stored candidate edge (its real gate is certification).
    The per-edge 2 s wall is enforced by the MoveIt adapter's planning
    timeout; this ledger enforces the attempt cap and the total window and
    records every measured duration for evidence.
    """

    def __init__(self, policy=None, clock=None):
        self.policy = policy or RouteBudgetPolicy()
        self._clock = clock or time.monotonic
        self._start = None
        self._attempts: Dict[tuple, int] = {}
        self._durations: Dict[tuple, float] = {}

    def start(self):
        if self._start is None:
            self._start = self._clock()

    def now(self):
        """Clock reading for duration measurement by the search."""
        return float(self._clock())

    def elapsed_s(self):
        if self._start is None:
            return 0.0
        return float(self._clock() - self._start)

    def remaining_s(self):
        return max(0.0, self.policy.total_wall_s - self.elapsed_s())

    def entry_deadline_s(self):
        """Total wall minus the reserved retreat share."""
        return max(0.0, self.policy.total_wall_s * (1.0 - self.policy.retreat_min_share))

    def entry_exhausted(self):
        return self.elapsed_s() >= self.entry_deadline_s()

    def exhausted(self):
        return self.elapsed_s() >= self.policy.total_wall_s

    def attempts(self, edge_key):
        return int(self._attempts.get(tuple(edge_key), 0))

    def begin_edge(self, edge_key):
        """Count one supplement attempt; refuse past the caps.

        Returns True when the attempt may proceed. A refusal is a named
        PLANNING_BUDGET_EXHAUSTED error, never a silent skip.
        """
        key = tuple(edge_key)
        if self.attempts(key) >= self.policy.max_edge_repair_attempts:
            return False
        if self.exhausted():
            return False
        self._attempts[key] = self.attempts(key) + 1
        return True

    def finish_edge(self, edge_key, wall_s):
        key = tuple(edge_key)
        self._durations[key] = self._durations.get(key, 0.0) + float(wall_s)

    def snapshot(self):
        """Evidence view: elapsed, attempts, and measured durations."""
        return {
            "elapsed_s": round(self.elapsed_s(), 6),
            "remaining_s": round(self.remaining_s(), 6),
            "attempts": {"%s->%s:%s" % key: count
                         for key, count in sorted(self._attempts.items())},
            "durations_s": {"%s->%s:%s" % key: round(value, 6)
                            for key, value in sorted(self._durations.items())},
            "policy": {
                "total_wall_s": self.policy.total_wall_s,
                "single_edge_wall_s": self.policy.single_edge_wall_s,
                "max_edge_repair_attempts": self.policy.max_edge_repair_attempts,
                "retreat_min_share": self.policy.retreat_min_share,
            },
        }


@dataclass(frozen=True)
class RouteHop:
    """One staged hop: destination region and the edge class to build.

    ``load_state`` is a string ("unloaded"/"carrying") compared against
    node/edge load states so this module stays ROS-free.
    """

    to_region: str
    edge_class: str
    load_state: str = "carrying"

    def __post_init__(self):
        object.__setattr__(self, "to_region", str(self.to_region))
        object.__setattr__(self, "edge_class", str(self.edge_class))
        if self.edge_class not in EDGE_CLASSES:
            raise ValueError("RouteHop: unknown edge class %r" % self.edge_class)
        object.__setattr__(self, "load_state", str(self.load_state))


def hops_from_table(table, load_state):
    return tuple(RouteHop(region, edge_class, load_state)
                 for region, edge_class in table)


@dataclass(frozen=True)
class RouteChain:
    """A resolved chain of verified edges between concrete q-nodes."""

    node_ids: Tuple[str, ...]
    edges: Tuple[object, ...]
    duration_ns: int
    joint_travel_rad: float


def route_bearing_sort_key(candidate, exec_ns, joint_travel_rad, goal_id):
    """Final comparison key among route-bearing candidates.

    Extends the frozen PlacementCandidate.sorting_key() (support layer
    ascending, depth descending, route clearance descending, frozen score
    descending, stable id) with execution time, joint travel, and the
    stable goal id.
    """
    return tuple(candidate.sorting_key()) + (
        int(exec_ns), float(joint_travel_rad), str(goal_id))


class RouteSearch(object):
    """Staged search over the q-graph with per-hop alternatives.

    ``edge_builder(from_node, to_node, edge_class, candidate=None,
    scene=None)`` must return a verified
    :class:`~st09_contracts.graph.GraphEdge` or raise
    :class:`RoutePlanningError`. The scene is threaded per chain (carrying
    for entry hops, unloaded-with-obstacles for the retreat). Verified
    cached edges are reused without budget cost; store candidates are
    passed to the builder for a cheap re-probe (the real gate is
    certification, not this search).
    """

    def __init__(self, graph, edge_builder, budget=None):
        self.graph = graph
        self.edge_builder = edge_builder
        self.budget = budget or RouteBudget()
        self.edge_failures: List[dict] = []

    # -- edge resolution -------------------------------------------------

    def _cached_edge(self, from_id, to_id, edge_class):
        for edge in self.graph.outgoing(from_id):
            if edge.destination_node_id == to_id and \
                    str(edge.attributes.get("edge_class", "")) == edge_class:
                return edge
        return None

    def _candidate_edge(self, from_id, to_id, edge_class):
        for edge in self.graph.outgoing_candidates(from_id):
            if edge.destination_node_id == to_id and \
                    str(edge.attributes.get("edge_class", "")) == edge_class:
                return edge
        return None

    def _edge_for(self, from_node, to_node, hop, scene=None):
        edge_class = hop.edge_class
        cached = self._cached_edge(from_node.node_id, to_node.node_id, edge_class)
        if cached is not None:
            return cached  # verified this session: free reuse
        candidate = self._candidate_edge(
            from_node.node_id, to_node.node_id, edge_class)
        key = (from_node.node_id, to_node.node_id, edge_class)
        if candidate is not None:
            replacement = self._build(from_node, to_node, hop, key,
                                      candidate=candidate, scene=scene)
            if replacement is not None:
                self.graph.promote_candidate(candidate.edge_id, replacement)
            return replacement
        if not self.budget.begin_edge(key):
            self.edge_failures.append({
                "edge": list(key), "kind": "budget",
                "reason": "PLANNING_BUDGET_EXHAUSTED"})
            return None
        edge = self._build(from_node, to_node, hop, key, scene=scene)
        if edge is not None:
            self.graph.add_edge(edge)  # verified: later hops/goals reuse free
        return edge

    def _build(self, from_node, to_node, hop, key, candidate=None, scene=None):
        started = self.budget.now()
        try:
            edge = self.edge_builder(from_node, to_node, hop.edge_class,
                                     candidate=candidate, scene=scene)
        except RoutePlanningError as err:
            self.budget.finish_edge(key, self.budget.now() - started)
            self.edge_failures.append({
                "edge": list(key), "kind": "edge_build_failed",
                "reason": err.code, "detail": err.detail})
            return None
        self.budget.finish_edge(key, self.budget.now() - started)
        if edge is not None and \
                str(edge.attributes.get("edge_class", "")) != hop.edge_class:
            self.edge_failures.append({
                "edge": list(key), "kind": "edge_build_failed",
                "reason": "EDGE_BUILD_FAILED",
                "detail": {"mismatched_class": str(
                    edge.attributes.get("edge_class", ""))}})
            return None
        return edge

    # -- chain construction ----------------------------------------------

    def _destinations(self, hop, goal_nodes, final):
        if final and goal_nodes:
            return tuple(goal_nodes)
        return tuple(node for node in self.graph.nodes_by_region(hop.to_region)
                     if str(node.load_state) == hop.load_state)

    def _chain_from(self, frontier, hops, goal_nodes, acc_edges, acc_nodes,
                     scene=None):
        if not hops:
            duration_ns = sum(
                int(edge.attributes.get("duration_ns", 0))
                for edge in acc_edges)
            travel = sum(
                float(edge.attributes.get("joint_travel_rad", 0.0))
                for edge in acc_edges)
            return RouteChain(
                node_ids=tuple(acc_nodes),
                edges=tuple(acc_edges),
                duration_ns=duration_ns,
                joint_travel_rad=travel)
        hop, rest = hops[0], hops[1:]
        final = not rest and bool(goal_nodes)
        for destination in self._destinations(hop, goal_nodes, final):
            if destination.node_id == frontier.node_id:
                continue
            edge = self._edge_for(frontier, destination, hop, scene=scene)
            if edge is None:
                continue
            # [ST09-I] chain continuity: a cartesian edge reaches the
            # requested POSE, not the destination node's IK branch — the
            # solver seeds from the start state, so the trajectory can
            # end on a different branch than the bookkept to-node.
            # Continue the chain from the ACTUAL endpoint q (live
            # symptom: entry ended on one branch, the retreat twin was
            # registered on the other, and the executor refused the
            # retreat with QP_START_STATE_MISMATCH).
            next_node = destination
            try:
                actual = tuple(
                    float(v) for v in edge.trajectory.positions[-1])
            except (AttributeError, IndexError, TypeError):
                actual = None
            if actual is not None and len(actual) == len(destination.q) \
                    and max(abs(a - b) for a, b in
                            zip(actual, destination.q)) > 1e-6:
                anchored = self.graph.node_for_q(hop.load_state, actual)
                if anchored is None:
                    from st09_contracts import LoadState

                    anchored = self.graph.register_node(
                        actual, destination.tcp_position,
                        destination.tcp_quaternion_xyzw,
                        destination.region,
                        LoadState(hop.load_state),
                        destination.ik_family)
                next_node = anchored
            result = self._chain_from(
                next_node, rest, goal_nodes,
                acc_edges + [edge], acc_nodes + [next_node.node_id],
                scene=scene)
            if result is not None:
                return result
        return None

    def entry_chain(self, start_node_id, goal_nodes, *, hops=None, scene=None):
        """CARRYING chain start -> goal; goals tried in the given order."""
        hops = hops or hops_from_table(DEFAULT_ENTRY_HOPS, "carrying")
        if not goal_nodes:
            raise RoutePlanningError("NO_FEASIBLE_ROUTE", {
                "prior_gaps": list(self.graph.prior_gaps),
                "edge_failures": list(self.edge_failures),
                "budget": self.budget.snapshot(),
            })
        self.budget.start()
        start = self.graph.node(start_node_id)
        if self.budget.entry_exhausted():
            raise RoutePlanningError("PLANNING_BUDGET_EXHAUSTED", {
                "budget": self.budget.snapshot()})
        chain = self._chain_from(start, hops, tuple(goal_nodes), [],
                                 [start.node_id], scene=scene)
        if chain is None:
            raise RoutePlanningError("NO_FEASIBLE_ROUTE", {
                "prior_gaps": list(self.graph.prior_gaps),
                "edge_failures": list(self.edge_failures),
                "budget": self.budget.snapshot(),
                "scene": _scene_note(scene),
            })
        return chain

    def retreat_chain(self, release_node_id, *, hops=None, scene=None):
        """UNLOADED chain release twin -> home with the box as obstacle."""
        hops = hops or hops_from_table(DEFAULT_RETREAT_HOPS, "unloaded")
        self.budget.start()
        release = self.graph.node(release_node_id)
        chain = self._chain_from(release, hops, (), [], [release.node_id],
                                 scene=scene)
        if chain is None:
            raise RoutePlanningError("NO_FEASIBLE_ROUTE", {
                "prior_gaps": list(self.graph.prior_gaps),
                "edge_failures": list(self.edge_failures),
                "budget": self.budget.snapshot(),
                "scene": _scene_note(scene),
            })
        return chain

    def best_full_route(self, start_node_id, goal_nodes, *, entry_hops=None,
                        retreat_hops=None, scene_carrying=None,
                        scene_unloaded=None):
        """First goal (in order) whose entry and retreat both resolve."""
        last_error = None
        for goal in goal_nodes:
            try:
                entry = self.entry_chain(
                    start_node_id, (goal,), hops=entry_hops, scene=scene_carrying)
            except RoutePlanningError as err:
                last_error = err
                if err.code == "PLANNING_BUDGET_EXHAUSTED":
                    raise
                continue
            if self.budget.exhausted():
                raise RoutePlanningError("PLANNING_BUDGET_EXHAUSTED", {
                    "budget": self.budget.snapshot(),
                    "edge_failures": list(self.edge_failures)})
            # [ST09-I] the release twin must sit where the entry chain
            # ACTUALLY ends (the descend edge's last knot — the cartesian
            # solver may land a different IK branch than the bookkept
            # goal node), or the retreat starts off-branch.
            final_q = goal.q
            try:
                final_q = tuple(
                    float(v) for v in entry.edges[-1].trajectory
                    .positions[-1])
            except (AttributeError, IndexError, TypeError):
                pass
            twin = self.graph.node_for_q("unloaded", final_q)
            if twin is None:
                from st09_contracts import LoadState

                twin = self.graph.register_node(
                    final_q, goal.tcp_position,
                    goal.tcp_quaternion_xyzw, goal.region,
                    LoadState.UNLOADED, goal.ik_family)
            try:
                retreat = self.retreat_chain(
                    twin.node_id, hops=retreat_hops, scene=scene_unloaded)
            except RoutePlanningError as err:
                last_error = err
                continue
            # [ST09-I] chain-continuity diagnostics: the executed legs must
            # chain node-to-node; a branch jump here surfaces live as a
            # QP_START_STATE_MISMATCH on the retreat leg.
            print("route chain goal=%s entry=%s twin=%s retreat=%s" % (
                [round(v, 3) for v in goal.q[:2]],
                [n[:22] for n in entry.node_ids],
                [round(v, 3) for v in twin.q[:2]],
                [n[:22] for n in retreat.node_ids]), flush=True)
            return entry, retreat, goal
        if last_error is not None:
            raise last_error
        raise RoutePlanningError("NO_FEASIBLE_ROUTE", {
            "prior_gaps": list(self.graph.prior_gaps),
            "edge_failures": list(self.edge_failures),
            "budget": self.budget.snapshot()})


def _scene_note(scene):
    """Opaque scene payload descriptor for failure evidence."""
    if scene is None:
        return None
    if isinstance(scene, dict):
        load_state = scene.get("load_state")
    else:
        load_state = getattr(scene, "load_state", None)
    return {"load_state": load_state, "type": type(scene).__name__}
