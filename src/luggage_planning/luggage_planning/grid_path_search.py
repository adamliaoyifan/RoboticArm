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

    @property
    def search_outcome(self):
        """SEARCH_INCOMPLETE (budget/cap left work open) or UNREACHABLE."""
        stored = self.detail.get("search_outcome")
        if stored:
            return str(stored)
        if self.code == "PLANNING_BUDGET_EXHAUSTED":
            return SEARCH_INCOMPLETE
        return UNREACHABLE


#: Search-outcome labels beside the frozen Reason code. Only UNREACHABLE
#: means every alternative was tried; neither is BIN_FULL.
SEARCH_INCOMPLETE = "SEARCH_INCOMPLETE"
UNREACHABLE = "UNREACHABLE"

#: Builds of one (goal, edge class, reason) failure class before that
#: class is skipped for the goal, so one repeating failure cannot spend
#: the budget other branches and slots need.
MAX_SAME_FAILURE_PER_GOAL = 3

#: Constrained cartesian hops: a same-branch continuation of the frontier
#: is tried before any destination on another branch.
BRANCH_LOCKED_EDGE_CLASSES = ("INSERT", "DESCEND")


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


def _wrap_rad(delta):
    return (float(delta) + math.pi) % (2.0 * math.pi) - math.pi


#: Between a short same-branch descend (~0.15 rad) and a pi heading or
#: wrist flip (~3.14 rad). Pairs inside this band are the same branch.
DESCEND_BRANCH_TOL_RAD = 1.0


def descend_pair_rank(from_q, to_q):
    """Order one DESCEND pair. Lower is tried first.

    Tier 0 is same tool heading and same arm branch. Tier 1 is same
    heading on another elbow or wrist branch. Tier 2 is the same arm
    branch with joint 6 flipped by about pi. Tier 3 is both. Wrapped
    joint travel breaks ties inside a tier. Opposite-branch goals stay
    in the set; they are only deferred.
    """
    dj = [_wrap_rad(b - a) for a, b in zip(from_q, to_q)]
    if len(dj) < 6:
        return (3, float("inf"))
    same_heading = abs(dj[5]) <= DESCEND_BRANCH_TOL_RAD
    same_branch = max(abs(d) for d in dj[:5]) <= DESCEND_BRANCH_TOL_RAD
    if same_heading and same_branch:
        tier = 0
    elif same_heading:
        tier = 1
    elif same_branch:
        tier = 2
    else:
        tier = 3
    return (tier, sum(abs(d) for d in dj))


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

    def __init__(self, graph, edge_builder, budget=None, *,
                 continuation=None, branch_lock=True):
        self.graph = graph
        self.edge_builder = edge_builder
        self.budget = budget or RouteBudget()
        # continuation(frontier_node, hop, destinations) -> nodes solved
        # from the frontier's actual q; tried before stored destinations.
        self.continuation = continuation
        self.branch_lock = bool(branch_lock)
        self.edge_failures: List[dict] = []
        # [ST09-I P3] per-session failure memo (see _edge_for): shared
        # anchors must not be retried per candidate.
        self._edge_fail_counts: Dict[tuple, int] = {}
        self._goal_scope = ""
        self._class_failures: Dict[tuple, int] = {}
        self.capped_classes: List[dict] = []
        self.edges_built = 0

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
        key0 = (from_node.node_id, to_node.node_id, edge_class)
        # [ST09-I P3] failure memo: an edge that already failed twice
        # this session is not retried for the next candidate. Shared
        # anchors (the opening-centerline staging nodes are identical
        # across candidates) made every candidate re-attempt the same
        # doomed OMPL transit at ~2 s per burn until the whole route
        # budget expired (live: 42 attempts, zero completed routes).
        if self._edge_fail_counts.get(key0, 0) >= 2:
            return None
        candidate = self._candidate_edge(
            from_node.node_id, to_node.node_id, edge_class)
        key = (from_node.node_id, to_node.node_id, edge_class)
        if candidate is not None:
            replacement = self._build(from_node, to_node, hop, key,
                                      candidate=candidate, scene=scene)
            if replacement is not None:
                self.graph.promote_candidate(candidate.edge_id, replacement)
            else:
                self._edge_fail_counts[key] = self._edge_fail_counts.get(
                    key, 0) + 1
            return replacement
        if not self.budget.begin_edge(key):
            self.edge_failures.append({
                "edge": list(key), "kind": "budget",
                "reason": "PLANNING_BUDGET_EXHAUSTED"})
            return None
        edge = self._build(from_node, to_node, hop, key, scene=scene)
        if edge is not None:
            self.graph.add_edge(edge)  # verified: later hops/goals reuse free
        else:
            self._edge_fail_counts[key] = self._edge_fail_counts.get(
                key, 0) + 1
        return edge

    def _capped_class(self, edge_class):
        for (scope, klass, reason), count in self._class_failures.items():
            if scope == self._goal_scope and klass == edge_class and \
                    count >= MAX_SAME_FAILURE_PER_GOAL:
                return reason
        return None

    def _build(self, from_node, to_node, hop, key, candidate=None, scene=None):
        capped = self._capped_class(hop.edge_class)
        if capped is not None:
            self.edge_failures.append({
                "edge": list(key), "kind": "class_capped",
                "reason": capped, "goal": self._goal_scope})
            return None
        started = self.budget.now()
        self.edges_built += 1
        try:
            edge = self.edge_builder(from_node, to_node, hop.edge_class,
                                     candidate=candidate, scene=scene)
        except RoutePlanningError as err:
            self.budget.finish_edge(key, self.budget.now() - started)
            self.edge_failures.append({
                "edge": list(key), "kind": "edge_build_failed",
                "reason": err.code, "detail": err.detail})
            class_key = (self._goal_scope, hop.edge_class, err.code)
            self._class_failures[class_key] = \
                self._class_failures.get(class_key, 0) + 1
            if self._class_failures[class_key] == MAX_SAME_FAILURE_PER_GOAL:
                self.capped_classes.append({
                    "goal": self._goal_scope, "edge_class": hop.edge_class,
                    "reason": err.code})
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

    def _order_goals_for_descend(self, goal_nodes):
        """Try the slot goal that continues a hover branch before the rest."""
        goals = tuple(goal_nodes)
        if not goals:
            return goals
        sources = [node for node in self.graph.nodes_by_region("slot_upper")
                   if str(node.load_state) == str(goals[0].load_state)]
        if not sources:
            return goals

        def key(goal):
            return min(descend_pair_rank(source.q, goal.q) for source in sources)

        return tuple(sorted(goals, key=key))

    def _ordered_destinations(self, hop, goal_nodes, final, frontier, rest):
        dests = list(self._destinations(hop, goal_nodes, final))
        if hop.edge_class == "DESCEND":
            dests.sort(key=lambda node: descend_pair_rank(frontier.q, node.q))
        elif rest and rest[0].edge_class == "DESCEND" and goal_nodes:
            # The hover tried first is the one this goal can descend from.
            dests.sort(key=lambda node: min(
                descend_pair_rank(node.q, goal.q) for goal in goal_nodes))
        if self.continuation is not None:
            solved = [node for node in
                      (self.continuation(frontier, hop, tuple(dests)) or ())
                      if node is not None]
            seen = {node.node_id for node in solved}
            dests = solved + [node for node in dests
                              if node.node_id not in seen]
        if self.branch_lock and hop.edge_class in BRANCH_LOCKED_EDGE_CLASSES:
            # Stable: frontier-solved nodes stay first. Another branch is
            # deferred, not deleted; its preferred route is the
            # back-seeded ladder whose branch change is a joint transit.
            dests.sort(key=lambda node: 0 if descend_pair_rank(
                frontier.q, node.q)[0] == 0 else 1)
        return dests

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
        for destination in self._ordered_destinations(
                hop, goal_nodes, final, frontier, rest):
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
        """First continuable goal whose entry and retreat both resolve.

        Goal order is descend-pair rank against the hover nodes, not
        registration order. A heading-pi or cross-elbow slot goal is
        still tried, after the same-heading same-branch goal.
        """
        last_error = None
        ordered = list(self._order_goals_for_descend(goal_nodes))
        self._goals_tried = []
        self._goals_open = [goal.node_id for goal in ordered]
        for goal in ordered:
            self._goal_scope = goal.node_id
            self._goals_open.remove(goal.node_id)
            self._goals_tried.append(goal.node_id)
            try:
                entry = self.entry_chain(
                    start_node_id, (goal,), hops=entry_hops, scene=scene_carrying)
            except RoutePlanningError as err:
                last_error = err
                if err.code == "PLANNING_BUDGET_EXHAUSTED":
                    raise self._classified(err)
                continue
            if self.budget.exhausted():
                raise self._classified(RoutePlanningError(
                    "PLANNING_BUDGET_EXHAUSTED", {
                        "budget": self.budget.snapshot(),
                        "edge_failures": list(self.edge_failures)}))
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
            raise self._classified(last_error)
        raise self._classified(RoutePlanningError("NO_FEASIBLE_ROUTE", {
            "prior_gaps": list(self.graph.prior_gaps),
            "edge_failures": list(self.edge_failures),
            "budget": self.budget.snapshot()}))

    def coverage(self):
        """How much of the alternative space this search actually tried."""
        budget_refusals = sum(
            1 for row in self.edge_failures if row.get("kind") == "budget")
        return {
            "edges_built": int(self.edges_built),
            "goals_tried": list(getattr(self, "_goals_tried", [])),
            "goals_open": list(getattr(self, "_goals_open", [])),
            "capped_classes": list(self.capped_classes),
            "budget_refusals": budget_refusals,
            "budget": self.budget.snapshot(),
        }

    def _classified(self, err):
        """Attach search_outcome + coverage; never relabels the Reason."""
        cov = self.coverage()
        incomplete = (err.code == "PLANNING_BUDGET_EXHAUSTED"
                      or self.budget.exhausted()
                      or bool(cov["goals_open"])
                      or bool(cov["capped_classes"])
                      or cov["budget_refusals"] > 0)
        detail = dict(err.detail)
        detail["search_outcome"] = SEARCH_INCOMPLETE if incomplete \
            else UNREACHABLE
        detail["coverage"] = cov
        return RoutePlanningError(err.code, detail)


def _scene_note(scene):
    """Opaque scene payload descriptor for failure evidence."""
    if scene is None:
        return None
    if isinstance(scene, dict):
        load_state = scene.get("load_state")
    else:
        load_state = getattr(scene, "load_state", None)
    return {"load_state": load_state, "type": type(scene).__name__}
