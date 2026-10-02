#!/usr/bin/env python3
"""Diverse shortest paths on a reachable-cell graph.

Three edge-cost profiles (short, high, clear) share one graph. A later
profile pays extra for edges an earlier path already used, and a path that
overlaps a kept path is dropped. String-pulling deletes a cell only when the
chord is free, joint-continuous, and no more expensive under that profile.

Section A (above the divider) is the legacy cell-graph search consumed by
``path_candidates``/``trajectory_selector``; its API and behaviour are
frozen and must stay byte-identical. Section B adds the ST09-B bounded
route search over the q-state graph: a shared budget ledger (10 s total /
2 s per new MoveIt edge / 4 supplement attempts, cache hits free), staged
entry and post-release retreat chains with per-hop alternatives, and the
edge-failure taxonomy mapping onto the frozen Reason set. st09_contracts
imports stay lazy so the legacy install chain is unaffected.
"""

from __future__ import division

import heapq
import math
import time
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Sequence, Tuple

from luggage_planning.joint_path import max_joint_step


PROFILES = ("short", "high", "clear")


class GridPath(object):
    def __init__(self, profile, nodes):
        self.profile = str(profile)
        self.nodes = list(nodes)

    @property
    def cells(self):
        return [node.index for node in self.nodes]

    @property
    def waypoints(self):
        return [node.center for node in self.nodes]

    @property
    def seeds(self):
        return [list(node.seed) for node in self.nodes]


def _dist(a, b):
    return math.sqrt(sum((float(a[i]) - float(b[i])) ** 2 for i in range(3)))


def _inside(atlas, xyz):
    origin = [float(v) for v in atlas.meta["grid"]["origin"]]
    resolution = float(atlas.resolution)
    nx, ny, nz, _nyaw = atlas.grid_size
    spans = (nx, ny, nz)
    for index in range(3):
        low = origin[index] - 1e-6
        high = origin[index] + spans[index] * resolution + 1e-6
        if xyz[index] < low or xyz[index] > high:
            return False
    return True


def _entry_nodes(graph):
    if not graph.nodes:
        return []
    side = getattr(graph, "opening_side", "negative_x")
    axis = {"x": 0, "y": 1}.get(side.split("_")[-1], 0)
    sign = -1 if side.startswith("negative") else 1
    values = [node.index[axis] for node in graph.nodes.values()]
    target = min(values) if sign < 0 else max(values)
    return [node for node in graph.nodes.values() if node.index[axis] == target]


def _nearest(nodes, xyz):
    best = None
    best_dist = None
    for node in nodes:
        distance = _dist(node.center, xyz)
        if best_dist is None or distance < best_dist:
            best = node
            best_dist = distance
    return best, best_dist


def edge_cost(profile, left, right, z_max, high_z_gain):
    length = max(_dist(left.center, right.center), 1e-6)
    if profile == "high":
        z_mid = 0.5 * (left.center[2] + right.center[2])
        return length * (1.0 + float(high_z_gain) * max(0.0, z_max - z_mid))
    if profile == "clear":
        gap = min(left.clearance, right.clearance)
        return length * (1.0 + 1.0 / max(gap, 0.02))
    return length


def _dijkstra(graph, start, goal, profile, penalized, z_max, high_z_gain,
              diversity_penalty):
    start_idx = start.index
    goal_idx = goal.index
    best = {start_idx: 0.0}
    prev = {start_idx: None}
    heap = [(0.0, start_idx)]
    while heap:
        cost, current = heapq.heappop(heap)
        if cost > best.get(current, float("inf")):
            continue
        if current == goal_idx:
            break
        for other, _length in graph.adjacency.get(current, []):
            extra = edge_cost(
                profile, graph.nodes[current], graph.nodes[other],
                z_max, high_z_gain)
            if (current, other) in penalized or (other, current) in penalized:
                extra *= 1.0 + float(diversity_penalty)
            total = cost + extra
            if total + 1e-12 < best.get(other, float("inf")):
                best[other] = total
                prev[other] = current
                heapq.heappush(heap, (total, other))
    if goal_idx not in prev:
        return None
    chain = []
    cursor = goal_idx
    seen = set()
    while cursor is not None:
        if cursor in seen:
            return None
        seen.add(cursor)
        chain.append(graph.nodes[cursor])
        cursor = prev[cursor]
    chain.reverse()
    return chain


def _jaccard(left, right):
    a = set(left)
    b = set(right)
    if not a and not b:
        return 1.0
    return float(len(a & b)) / float(len(a | b))


def _shortcut(nodes, profile, z_max, high_z_gain, continuity, chord_free):
    kept = list(nodes)
    changed = True
    while changed and len(kept) > 2:
        changed = False
        index = 0
        while index < len(kept) - 2:
            left, mid, right = kept[index], kept[index + 1], kept[index + 2]
            direct = edge_cost(profile, left, right, z_max, high_z_gain)
            via = (
                edge_cost(profile, left, mid, z_max, high_z_gain)
                + edge_cost(profile, mid, right, z_max, high_z_gain))
            joint_ok = max_joint_step(left.seed, right.seed) <= float(continuity) + 1e-9
            if joint_ok and direct <= via + 1e-9 and chord_free(left, right):
                del kept[index + 1]
                changed = True
            else:
                index += 1
    return kept


def search_paths(graph, atlas, start_xyz, goal_xyz, continuity_rad=1.2,
                 high_z_gain=8.0, diversity_penalty=3.0, overlap_max=0.7,
                 chord_free=None, snap_factor=1.5):
    """Return up to three non-overlapping ``GridPath`` values.

    ``chord_free(left_node, right_node)`` is the occupancy test for a
    shortcut. It defaults to always free.
    """
    if graph is None or graph.disabled_reason or len(graph) == 0:
        return []
    if not _inside(atlas, start_xyz):
        pool = _entry_nodes(graph)
    else:
        pool = list(graph.nodes.values())
    start, _start_dist = _nearest(pool, start_xyz)
    goal, goal_dist = _nearest(graph.nodes.values(), goal_xyz)
    if start is None or goal is None:
        return []
    limit = float(snap_factor) * float(atlas.resolution)
    if goal_dist is not None and goal_dist > limit:
        return []
    z_max = max(node.center[2] for node in graph.nodes.values())
    free = chord_free or (lambda _left, _right: True)
    penalized = set()
    kept = []
    for profile in PROFILES:
        chain = _dijkstra(
            graph, start, goal, profile, penalized, z_max, high_z_gain,
            diversity_penalty)
        if not chain:
            continue
        chain = _shortcut(
            chain, profile, z_max, high_z_gain, continuity_rad, free)
        path = GridPath(profile, chain)
        if any(_jaccard(path.cells, prior.cells) > float(overlap_max) for prior in kept):
            for left, right in zip(path.cells, path.cells[1:]):
                penalized.add((left, right))
            continue
        kept.append(path)
        for left, right in zip(path.cells, path.cells[1:]):
            penalized.add((left, right))
    return kept


# -- Section B: ST09-B bounded route search over the q-state graph -------

#: Frozen planning budgets (a B test asserts equality with
#: st09_contracts/resolved_planning_contract.json).
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
            result = self._chain_from(
                destination, rest, goal_nodes,
                acc_edges + [edge], acc_nodes + [destination.node_id],
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
            twin = self.graph.node_for_q("unloaded", goal.q)
            if twin is None:
                last_error = RoutePlanningError("EDGE_BUILD_FAILED", {
                    "missing_release_twin": goal.node_id})
                continue
            try:
                retreat = self.retreat_chain(
                    twin.node_id, hops=retreat_hops, scene=scene_unloaded)
            except RoutePlanningError as err:
                last_error = err
                continue
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
