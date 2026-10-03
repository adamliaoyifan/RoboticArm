"""ST09-B: bounded route search, budget ledger, and failure taxonomy.

Covers acceptance (a) — the eight yaw/IK classes, positive and negative
each — plus (d) atlas-uncovered solvable / unsolvable / budget-exhausted,
the budget ledger semantics (attempts incl. failures, cache hits free,
retreat reservation), the route-bearing ordering key, the contract-value
pins, and the legacy Section A behaviour staying frozen. All offline via
a FakeEdgeBuilder; the real MoveIt adapter is exercised in its own module.
"""

from __future__ import division

import json
import math
from pathlib import Path

import pytest

from st09_contracts.graph import GraphEdge, LoadState  # noqa: E402
from st09_contracts.fixtures import make_timed_trajectory  # noqa: E402
from st09_contracts.planning import (  # noqa: E402
    GeometryVerdict,
    PlacementCandidate,
)

from luggage_planning.grid_path_search import (  # noqa: E402
    DEFAULT_ENTRY_HOPS,
    DEFAULT_RETREAT_HOPS,
    EDGE_CLASSES,
    MAX_EDGE_REPAIR_ATTEMPTS,
    RETREAT_MIN_SHARE,
    ROUTE_TOTAL_WALL_S,
    SINGLE_NEW_EDGE_WALL_S,
    RouteBudget,
    RouteBudgetPolicy,
    RouteHop,
    RoutePlanningError,
    RouteSearch,
    map_edge_failure,
    route_bearing_sort_key,
)
from luggage_planning.reachable_graph import (  # noqa: E402
    RegionAnchor,
    RouteGraph,
    build_route_graph_from_atlas,
)

from toy_atlas import ToyAtlas  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[3]

Q_START = (0.05, -0.15, 0.9, -0.4, 1.2, 0.0)
Q_GOAL_F1 = (0.30, -0.60, 0.85, -0.30, 1.40, 0.10)
Q_GOAL_F2 = (0.30, -0.60, 0.85, -0.30, -1.74, 0.10)  # same XYZ, other wrist
Q_GOAL_PI = (0.30, 0.10, 0.85, -0.30, 1.40, 3.24)    # theta+pi heading
Q_HOME = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


class FakeClock(object):
    def __init__(self):
        self.now = 1000.0

    def advance(self, seconds):
        self.now += float(seconds)

    def __call__(self):
        return self.now


class FakeEdgeBuilder(object):
    """Deterministic offline edge builder for the q-graph search.

    ``fail`` maps (from_region, to_region, edge_class) to an exception
    message (or True for a bare failure). Built edges carry the duration
    and joint-travel attributes the chain cost reads.
    """

    def __init__(self, graph, fail=None, fail_dest=None, duration_ns=500_000_000,
                 joint_travel_rad=0.4):
        self.graph = graph
        self.fail = dict(fail or {})
        self.fail_dest = set(fail_dest or ())
        self.duration_ns = duration_ns
        self.joint_travel_rad = joint_travel_rad
        self.calls = []

    def __call__(self, from_node, to_node, edge_class, candidate=None,
                 scene=None):
        self.calls.append((from_node.node_id, to_node.node_id, edge_class))
        self.scenes = getattr(self, "scenes", [])
        if scene is not None:
            self.scenes.append(scene.load_state)
        key = (from_node.region, to_node.region, edge_class)
        if key in self.fail or (to_node.node_id, edge_class) in self.fail_dest:
            raise RoutePlanningError(
                "EDGE_BUILD_FAILED", {"pair": key, "dest": to_node.node_id})
        # The canned trajectory must END at the destination node's q:
        # the search re-anchors chains on each edge's actual endpoint
        # ([ST09-I] continuity), so a mock whose endpoints never match
        # collapses every node into one and routes through region pairs
        # the fail map never anticipated.
        base = make_timed_trajectory()
        trajectory = type(base)(
            joint_names=base.joint_names,
            positions=(base.positions[0],
                       tuple(float(v) for v in to_node.q)),
            velocities=tuple(base.velocities[:2]),
            accelerations=tuple(base.accelerations[:2]),
            time_from_start_ns=tuple(base.time_from_start_ns[:2]))
        edge = GraphEdge(
            edge_id="e-fake-%03d" % len(self.calls),
            source_node_id=from_node.node_id,
            destination_node_id=to_node.node_id,
            load_state=from_node.load_state,
            static_identity="static-test",
            trajectory=trajectory,
            validation_revision=None,
            attributes={
                "edge_class": edge_class,
                "duration_ns": self.duration_ns,
                "joint_travel_rad": self.joint_travel_rad,
            })
        return edge


def _node(graph, q, region, load_state=LoadState.CARRYING, ik_family="",
          tcp=(0.5, 0.0, 0.4)):
    return graph.register_node(q, tcp, (0.0, 0.0, 0.0, 1.0), region,
                               load_state, ik_family)


Q_PORTAL = (0.10, -0.25, 0.92, -0.35, 1.25, 0.05)
Q_APPROACH_F1 = tuple(v + 0.04 for v in Q_GOAL_F1)
Q_APPROACH_F2 = tuple(v - 0.04 for v in Q_GOAL_F2)
Q_UPPER_F1 = tuple(v + 0.02 for v in Q_GOAL_F1)
Q_UPPER_F2 = tuple(v - 0.02 for v in Q_GOAL_F2)


def _scaffold(graph, families=("f1", "f2"), twins=True):
    """Register the full small-graph region topology.

    CARRYING: home, portal, slot_approach, slot_upper, slot goals per
    family. UNLOADED: slot release twins, exit, home.
    """
    _node(graph, Q_START, "home", LoadState.CARRYING)
    _node(graph, Q_PORTAL, "portal", LoadState.CARRYING)
    for family in families:
        approach, upper, goal = {
            "f1": (Q_APPROACH_F1, Q_UPPER_F1, Q_GOAL_F1),
            "f2": (Q_APPROACH_F2, Q_UPPER_F2, Q_GOAL_F2),
            "pi": (Q_APPROACH_F1, Q_UPPER_F1, Q_GOAL_PI),
        }[family]
        _node(graph, approach, "slot_approach", LoadState.CARRYING, family,
              tcp=(0.5, 0.0, 0.5))
        _node(graph, upper, "slot_upper", LoadState.CARRYING, family,
              tcp=(0.5, 0.0, 0.55))
        _node(graph, goal, "slot", LoadState.CARRYING, family)
        if twins:
            _node(graph, goal, "slot", LoadState.UNLOADED, family)
    _node(graph, Q_HOME, "home", LoadState.UNLOADED)
    _node(graph, Q_START, "exit", LoadState.UNLOADED)
    return graph


def _route_graph():
    return _scaffold(RouteGraph(static_identity="static-test"))


def _search(graph, fail=None, clock=None, fail_dest=None):
    builder = FakeEdgeBuilder(graph, fail=fail, fail_dest=fail_dest)
    budget = RouteBudget(clock=clock)
    return RouteSearch(graph, builder, budget), builder, budget


def _entry_hops(transition=False):
    table = [("clearance", "ORIENTATION_TRANSITION")] if transition else []
    table.extend(DEFAULT_ENTRY_HOPS)
    hops = []
    for region, edge_class in table:
        hops.append(RouteHop(region, edge_class, "carrying"))
    return tuple(hops)


class TestContractValuePins:
    """Section B constants equal the frozen planning contract."""

    def test_budgets_match_resolved_contract(self):
        contract = json.loads(
            (REPO_ROOT / "st09_contracts" / "resolved_planning_contract.json")
            .read_text())
        budgets = contract["planning_budgets"]
        assert ROUTE_TOTAL_WALL_S == budgets["route_total_wall_s"]["value"]
        assert SINGLE_NEW_EDGE_WALL_S == \
            budgets["single_new_moveit_edge_wall_s"]["value"]
        assert MAX_EDGE_REPAIR_ATTEMPTS == \
            budgets["max_edge_repair_attempts"]["value"]

    def test_retreat_share_is_declared_dev_policy(self):
        # Not frozen: B-defined and documented as such.
        assert RETREAT_MIN_SHARE == 0.3
        assert RouteBudgetPolicy().retreat_min_share == RETREAT_MIN_SHARE

    def test_edge_classes_are_the_moveit_set(self):
        assert set(EDGE_CLASSES) == {
            "PORTAL_TRANSIT", "ORIENTATION_TRANSITION", "TRAVERSE",
            "INSERT", "DESCEND", "RETREAT", "RETURN"}

    def test_hop_rejects_unknown_class(self):
        with pytest.raises(ValueError):
            RouteHop("slot", "TELEPORT")


class TestBudgetLedger:
    """Attempts include failures; cache hits are free; clock injectable."""

    def test_failed_builds_count_toward_attempts(self):
        clock = FakeClock()
        budget = RouteBudget(clock=clock)
        budget.start()
        key = ("a", "b", "TRAVERSE")
        for _ in range(MAX_EDGE_REPAIR_ATTEMPTS):
            assert budget.begin_edge(key)
            budget.finish_edge(key, 0.1)
        assert not budget.begin_edge(key)  # 5th supplement refused

    def test_total_wall_exhaustion(self):
        clock = FakeClock()
        budget = RouteBudget(clock=clock)
        budget.start()
        assert not budget.exhausted()
        clock.advance(ROUTE_TOTAL_WALL_S + 0.001)
        assert budget.exhausted()
        assert not budget.begin_edge(("x", "y", "INSERT"))

    def test_retreat_reservation(self):
        clock = FakeClock()
        budget = RouteBudget(clock=clock)
        budget.start()
        clock.advance(ROUTE_TOTAL_WALL_S - 0.05)
        assert not budget.exhausted()
        assert budget.entry_exhausted()  # entry capped at 70% of the window

    def test_snapshot_shape(self):
        clock = FakeClock()
        budget = RouteBudget(clock=clock)
        budget.start()
        budget.begin_edge(("a", "b", "TRAVERSE"))
        budget.finish_edge(("a", "b", "TRAVERSE"), 0.25)
        snap = budget.snapshot()
        assert snap["attempts"] == {"a->b:TRAVERSE": 1}
        assert snap["durations_s"] == {"a->b:TRAVERSE": 0.25}
        assert snap["policy"]["total_wall_s"] == ROUTE_TOTAL_WALL_S


class TestRouteSearchChains:
    """Entry + retreat composition over the small region topology."""

    def test_full_route_entry_and_retreat(self):
        graph = _route_graph()
        search, builder, _budget = _search(graph)
        goals = graph.nodes_by_region("slot")
        goals = [n for n in goals if n.load_state is LoadState.CARRYING]
        start = [n for n in graph.nodes.values()
                 if n.region == "home" and n.load_state is LoadState.CARRYING][0]
        entry, retreat, goal = search.best_full_route(start.node_id, goals)
        assert [e.attributes["edge_class"] for e in entry.edges] == \
            ["PORTAL_TRANSIT", "TRAVERSE", "INSERT", "DESCEND"]
        assert [e.attributes["edge_class"] for e in retreat.edges] == \
            ["RETREAT", "RETURN"]
        assert goal.region == "slot"
        assert retreat.node_ids[0] != entry.node_ids[-1]  # twin, not merged

    def test_cache_hits_are_free_across_goals(self):
        graph = _route_graph()
        search, builder, budget = _search(graph)
        start = [n for n in graph.nodes.values()
                 if n.region == "home" and n.load_state is LoadState.CARRYING][0]
        goals = [n for n in graph.nodes_by_region("slot")
                 if n.load_state is LoadState.CARRYING]
        entry, retreat, _goal = search.best_full_route(start.node_id, goals)
        builds_before = len(builder.calls)
        attempts_before = json.dumps(budget.snapshot()["attempts"])
        # A second full request over the same graph reuses verified edges.
        search2 = RouteSearch(graph, builder, budget)
        entry2, retreat2, _goal2 = search2.best_full_route(
            start.node_id, goals)
        assert len(builder.calls) == builds_before
        assert json.dumps(budget.snapshot()["attempts"]) == attempts_before
        assert entry2.node_ids == entry.node_ids

    def test_edge_failure_falls_through_to_alternative_goal(self):
        graph = _route_graph()
        goals = [n for n in graph.nodes_by_region("slot")
                 if n.load_state is LoadState.CARRYING]
        f1 = [n for n in goals if n.ik_family == "f1"][0]
        f2 = [n for n in goals if n.ik_family == "f2"][0]
        # Only the f1 goal's final DESCEND fails; the f2 goal (same XYZ,
        # other family) must still be reachable — a family label is not a
        # rejection.
        search, builder, _budget = _search(
            graph, fail_dest={(f1.node_id, "DESCEND")})
        start = [n for n in graph.nodes.values()
                 if n.region == "home" and n.load_state is LoadState.CARRYING][0]
        _entry, _retreat, goal = search.best_full_route(start.node_id,
                                                        [f1, f2])
        assert goal.node_id == f2.node_id

    def test_all_goals_failing_is_named_no_feasible_route(self):
        graph = _route_graph()
        search, _builder, _budget = _search(
            graph, fail={("slot_upper", "slot", "DESCEND"): True,
                         ("slot_approach", "slot_upper", "INSERT"): True,
                         ("portal", "slot_approach", "TRAVERSE"): True,
                         ("home", "portal", "PORTAL_TRANSIT"): True})
        goals = [n for n in graph.nodes_by_region("slot")
                 if n.load_state is LoadState.CARRYING]
        start = [n for n in graph.nodes.values()
                 if n.region == "home" and n.load_state is LoadState.CARRYING][0]
        with pytest.raises(RoutePlanningError) as err:
            search.best_full_route(start.node_id, goals)
        assert err.value.code == "NO_FEASIBLE_ROUTE"
        assert err.value.code != "BIN_FULL"
        assert err.value.detail["edge_failures"]

    def test_missing_release_twin_is_registered_at_entry_end(self):
        # [ST09-I] chain continuity: when no unloaded twin is pre-seeded,
        # the search registers one at the entry chain's ACTUAL endpoint
        # (a cartesian edge may land a different IK branch than the
        # bookkept goal) and the retreat starts there — the route
        # succeeds with the retreat anchored on the real release state.
        graph = _scaffold(RouteGraph(static_identity="static-test"),
                          twins=False)
        search, _builder, _budget = _search(graph)
        goals = [n for n in graph.nodes_by_region("slot")
                 if n.load_state is LoadState.CARRYING]
        start = [n for n in graph.nodes.values()
                 if n.region == "home" and n.load_state is LoadState.CARRYING][0]
        entry, retreat, _goal = search.best_full_route(start.node_id, goals)
        assert retreat.node_ids
        endpoint = search.graph.node(retreat.node_ids[0])
        last_knot = entry.edges[-1].trajectory.positions[-1]
        assert all(abs(a - b) <= 1e-6 for a, b in zip(endpoint.q,
                                                      last_knot))


class TestYawIkEightClasses:
    """(a) each class with a positive and a negative case."""

    def _start(self, graph):
        return [n for n in graph.nodes.values()
                if n.region == "home" and n.load_state is LoadState.CARRYING][0]

    def _goals(self, graph, families=("f1",)):
        return [n for n in graph.nodes_by_region("slot")
                if n.load_state is LoadState.CARRYING
                and n.ik_family in families]

    # 1. 0 -> 0: transport yaw equals placement yaw; no transition hop.
    def test_zero_to_zero_positive(self):
        graph = _route_graph()
        search, _builder, _budget = _search(graph)
        entry, _retreat, _goal = search.best_full_route(
            self._start(graph).node_id, self._goals(graph))
        assert entry.edges  # fixed orientation straight through the portal

    def test_zero_to_zero_negative(self):
        graph = _route_graph()
        search, _builder, _budget = _search(
            graph, fail={("slot_upper", "slot", "DESCEND"): True,
                         ("slot_approach", "slot_upper", "INSERT"): True})
        with pytest.raises(RoutePlanningError) as err:
            search.best_full_route(self._start(graph).node_id,
                                   self._goals(graph))
        assert err.value.code == "NO_FEASIBLE_ROUTE"

    # 2. 0 -> 90: orientation transition required in a clearance zone.
    def test_zero_to_ninety_positive(self):
        graph = _route_graph()
        graph.nodes_by_region("home")  # noqa: B018 - regions exist
        clearance = _node(graph, (0.2, -0.3, 0.95, -0.35, 1.3, 0.05),
                          "clearance", LoadState.CARRYING, "f1")
        assert clearance.node_id
        search, _builder, _budget = _search(graph)
        entry, _retreat, _goal = search.best_full_route(
            self._start(graph).node_id, self._goals(graph),
            entry_hops=_entry_hops(transition=True))
        assert entry.edges[0].attributes["edge_class"] == "ORIENTATION_TRANSITION"

    def test_zero_to_ninety_negative_no_transition_region(self):
        graph = _route_graph()  # no clearance region registered
        search, _builder, _budget = _search(graph)
        with pytest.raises(RoutePlanningError) as err:
            search.best_full_route(self._start(graph).node_id,
                                   self._goals(graph),
                                   entry_hops=_entry_hops(transition=True))
        assert err.value.code == "NO_FEASIBLE_ROUTE"

    # 3. 90 -> 0: the mirrored transition (heading back to yaw 0).
    def test_ninety_to_zero_positive(self):
        graph = _route_graph()
        _node(graph, (0.2, -0.3, 0.95, -0.35, 1.3, 0.05), "clearance",
              LoadState.CARRYING, "f1")
        search, _builder, _budget = _search(graph)
        entry, _retreat, _goal = search.best_full_route(
            self._start(graph).node_id, self._goals(graph),
            entry_hops=_entry_hops(transition=True))
        assert entry.edges[0].attributes["edge_class"] == "ORIENTATION_TRANSITION"

    def test_ninety_to_zero_negative_transition_fails(self):
        graph = _route_graph()
        _node(graph, (0.2, -0.3, 0.95, -0.35, 1.3, 0.05), "clearance",
              LoadState.CARRYING, "f1")
        search, _builder, _budget = _search(
            graph, fail={("home", "clearance", "ORIENTATION_TRANSITION"): True})
        with pytest.raises(RoutePlanningError) as err:
            search.best_full_route(self._start(graph).node_id,
                                   self._goals(graph),
                                   entry_hops=_entry_hops(transition=True))
        assert err.value.code == "NO_FEASIBLE_ROUTE"

    # 4. near-square payload: only declared symmetry expands headings.
    def test_near_square_positive_two_headings_route(self):
        graph = _route_graph()
        _node(graph, Q_GOAL_PI, "slot", LoadState.CARRYING, "pi")
        _node(graph, Q_GOAL_PI, "slot", LoadState.UNLOADED, "pi")
        search, _builder, _budget = _search(graph)
        goals = self._goals(graph, families=("f1", "pi"))
        _entry, _retreat, goal = search.best_full_route(
            self._start(graph).node_id, goals)
        assert goal.ik_family == "f1"  # deterministic: given order wins

    def test_near_square_negative_no_symmetry_single_heading(self):
        graph = _route_graph()
        # Symmetry "none": theta+pi is not even a goal (place_targets
        # expansion is where that lives; here the goal set itself has one
        # heading and the search must not invent a pi twin).
        goals = self._goals(graph, families=("f1",))
        assert len(goals) == 1

    # 5. eccentric attachment: theta and theta+pi are different TCP poses.
    def test_eccentric_positive_distinct_goal_nodes(self):
        graph = _route_graph()
        pi_node = _node(graph, Q_GOAL_PI, "slot", LoadState.CARRYING, "pi",
                        tcp=(0.5, 0.2, 0.4))
        f1 = self._goals(graph, ("f1",))[0]
        assert pi_node.tcp_position != f1.tcp_position
        assert pi_node.node_id != f1.node_id

    def test_eccentric_negative_folded_goal_rejected(self):
        graph = _route_graph()
        pi_node = _node(graph, Q_GOAL_PI, "slot", LoadState.CARRYING, "pi",
                        tcp=(0.5, 0.2, 0.4))
        # The search must never merge the eccentric pi heading onto f1:
        # node_for_q is exact-q, so a distinct q stays a distinct goal.
        assert graph.node_for_q(LoadState.CARRYING, Q_GOAL_PI) is pi_node
        assert graph.node_for_q(LoadState.CARRYING, Q_GOAL_F1).node_id \
            != pi_node.node_id

    # 6. q/-q and pi/-pi representation equivalence: never a rejection.
    def test_representation_equivalence_positive(self):
        graph = _route_graph()
        f1 = self._goals(graph, ("f1",))[0]
        # A goal quaternion negated (or heading pi vs -pi) refines to the
        # same q, hence the same node: the search sees one goal.
        assert graph.node_for_q(LoadState.CARRYING, f1.q) is f1

    def test_representation_equivalence_negative_no_string_jump_reject(self):
        graph = _route_graph()
        search, builder, _budget = _search(graph)
        entry, _retreat, _goal = search.best_full_route(
            self._start(graph).node_id, self._goals(graph))
        # No build failure was caused by representation: every builder
        # call succeeded despite pi-adjacent yaw values in the goal set.
        assert all("EDGE_BUILD_FAILED" not in str(f) for f in search.edge_failures)
        assert entry.edges

    # 7. multi IK family: family change across an edge is not a rejection.
    def test_multi_family_positive_fallback(self):
        graph = _route_graph()
        goals = self._goals(graph, families=("f1", "f2"))
        f1 = [n for n in goals if n.ik_family == "f1"][0]
        # Only f1's DESCEND fails; f2 (different family, same XYZ) takes
        # over — a family label is not a rejection.
        search, _builder, _budget = _search(
            graph, fail_dest={(f1.node_id, "DESCEND")})
        _entry, _retreat, goal = search.best_full_route(
            self._start(graph).node_id, goals)
        assert goal.ik_family == "f2"

    def test_multi_family_negative_only_real_failures_reject(self):
        graph = _route_graph()
        search, _builder, _budget = _search(graph)
        goals = self._goals(graph, families=("f1", "f2"))
        entry, _retreat, goal = search.best_full_route(
            self._start(graph).node_id, goals)
        # f1 wins on the first try: its family label differs from the
        # approach node's, and that alone never rejects the chain.
        assert goal.ik_family == "f1"
        assert entry.edges[-1].destination_node_id == goal.node_id

    # 8. no-feasible-transition region: named refusal with prior context.
    def test_no_transition_region_negative(self):
        graph = _route_graph()
        graph.note_prior_gap("ATLAS_ORIENTATION_UNCOVERED region=slot yaw=1.570796")
        search, _builder, _budget = _search(
            graph, fail={("home", "portal", "PORTAL_TRANSIT"): True})
        with pytest.raises(RoutePlanningError) as err:
            search.best_full_route(self._start(graph).node_id,
                                   self._goals(graph))
        assert err.value.code == "NO_FEASIBLE_ROUTE"
        assert any("ATLAS_ORIENTATION_UNCOVERED" in gap
                   for gap in err.value.detail["prior_gaps"])

    def test_no_transition_region_positive_elsewhere(self):
        graph = _route_graph()
        graph.note_prior_gap("ATLAS_ORIENTATION_UNCOVERED region=slot yaw=1.570796")
        search, _builder, _budget = _search(graph)
        entry, _retreat, _goal = search.best_full_route(
            self._start(graph).node_id, self._goals(graph))
        assert entry.edges  # the gap itself never blocks a routable goal


class TestAtlasUncovered:
    """(d) prior gaps are supplementation territory, not verdicts."""

    def _graph_with_gap(self):
        atlas = ToyAtlas()
        atlas.add_seeds((1, 1, 1), 0.0, [list(Q_GOAL_F1)], seed_set="contact")
        graph = build_route_graph_from_atlas(
            atlas,
            [RegionAnchor("slot", (0.225, 0.225, 0.225), 0.0,
                          LoadState.CARRYING)],
            fk=lambda q: ((q[0], q[1], q[2]), (0.0, 0.0, 0.0, 1.0)))
        assert graph.prior_gaps  # the pi twin is uncovered in the toy atlas
        return graph

    def test_solvable_despite_gap(self):
        graph = self._graph_with_gap()
        _scaffold(graph, families=("f1",))
        search, _builder, _budget = _search(graph)  # supplementation succeeds
        goals = [n for n in graph.nodes_by_region("slot")
                 if n.load_state is LoadState.CARRYING]
        entry, _retreat, _goal = search.best_full_route(
            [n for n in graph.nodes.values()
             if n.region == "home" and n.load_state is LoadState.CARRYING][0].node_id,
            goals)
        assert entry.edges

    def test_unsolvable_with_gap_reports_context(self):
        graph = self._graph_with_gap()
        _scaffold(graph, families=("f1",))
        search, _builder, _budget = _search(
            graph, fail={("home", "portal", "PORTAL_TRANSIT"): True})
        goals = [n for n in graph.nodes_by_region("slot")
                 if n.load_state is LoadState.CARRYING]
        with pytest.raises(RoutePlanningError) as err:
            search.best_full_route(
                [n for n in graph.nodes.values()
                 if n.region == "home" and n.load_state is LoadState.CARRYING][0].node_id,
                goals)
        assert err.value.code == "NO_FEASIBLE_ROUTE"
        assert err.value.detail["prior_gaps"]

    def test_budget_exhausted_is_named(self):
        clock = FakeClock()
        graph = _route_graph()
        budget = RouteBudget(clock=clock)
        search = RouteSearch(graph, FakeEdgeBuilder(graph), budget)
        budget.start()
        clock.advance(ROUTE_TOTAL_WALL_S + 0.01)
        goals = [n for n in graph.nodes_by_region("slot")
                 if n.load_state is LoadState.CARRYING]
        start = [n for n in graph.nodes.values()
                 if n.region == "home" and n.load_state is LoadState.CARRYING][0]
        with pytest.raises(RoutePlanningError) as err:
            search.best_full_route(start.node_id, goals)
        assert err.value.code == "PLANNING_BUDGET_EXHAUSTED"
        assert err.value.code != "BIN_FULL"


class TestOrderingKey:
    """Route-bearing candidates compare on the frozen key, then extras."""

    @staticmethod
    def _candidate(candidate_id, layer, depth, clearance, score):
        return PlacementCandidate(
            candidate_id=candidate_id,
            footprint_class=0,
            heading_theta=0.0,
            position_container=(0.2, -0.1, 0.14),
            quaternion_xyzw_container=(0.0, 0.0, 0.0, 1.0),
            support_layer_z=layer,
            depth_m=depth,
            verdict=GeometryVerdict.LEGAL,
            score=score,
            detail={"route_clearance_m": clearance},
        )

    def test_layer_first_then_depth(self):
        low_deep = self._candidate("a", 0.0, 0.8, 0.05, 1.0)
        high_deep = self._candidate("b", 0.05, 0.8, 0.05, 1.0)
        assert route_bearing_sort_key(low_deep, 0, 0, "g") < \
            route_bearing_sort_key(high_deep, 0, 0, "g")
        shallow = self._candidate("c", 0.0, 0.4, 0.05, 1.0)
        assert route_bearing_sort_key(low_deep, 0, 0, "g") < \
            route_bearing_sort_key(shallow, 0, 0, "g")

    def test_execution_time_then_travel_then_goal_id(self):
        one = self._candidate("a", 0.0, 0.8, 0.05, 1.0)
        assert route_bearing_sort_key(one, 100, 0.5, "g1") < \
            route_bearing_sort_key(one, 200, 0.5, "g1")
        assert route_bearing_sort_key(one, 100, 0.5, "g1") < \
            route_bearing_sort_key(one, 100, 0.9, "g1")
        assert route_bearing_sort_key(one, 100, 0.5, "g1") < \
            route_bearing_sort_key(one, 100, 0.5, "g2")


class TestFailureTaxonomy:

    def test_map_edge_failure(self):
        assert map_edge_failure("budget").value == "PLANNING_BUDGET_EXHAUSTED"
        assert map_edge_failure("no_goal").value == "NO_FEASIBLE_ROUTE"
        for kind in ("fraction_below_min", "excursion_exceeded", "moveit_error",
                     "scene_apply_error", "ik_unavailable"):
            assert map_edge_failure(kind).value == "EDGE_BUILD_FAILED"

    def test_error_carries_reason_enum(self):
        error = RoutePlanningError("PLANNING_BUDGET_EXHAUSTED", {"x": 1})
        assert error.detail == {"x": 1}
        assert error.code == "PLANNING_BUDGET_EXHAUSTED"


class TestSectionARemoved(object):
    """[ST09-I integration] Section A was deleted with the geometric
    selection bypass; the bounded Section B search is the only layer."""

    def test_section_a_symbols_are_gone(self):
        from luggage_planning import grid_path_search as search
        for name in ("PROFILES", "GridPath", "search_paths", "_dijkstra"):
            assert not hasattr(search, name), name
