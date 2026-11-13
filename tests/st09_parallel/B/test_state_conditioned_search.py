"""State-conditioned route search: frontier continuation, backtracking,
failure-class caps, and the SEARCH_INCOMPLETE / UNREACHABLE split.

Offline, on the same fake edge builder as test_grid_path_search.
"""

import pytest

from luggage_planning.planning_contract.graph import LoadState

from luggage_planning.routing.grid_search import (
    MAX_SAME_FAILURE_PER_GOAL,
    SEARCH_INCOMPLETE,
    UNREACHABLE,
    RouteBudget,
    RouteHop,
    RoutePlanningError,
    RouteSearch,
)
from luggage_planning.atlas.graph import GraphRoutePlanner, RouteGraph

from test_grid_path_search import (
    Q_GOAL_F1,
    Q_UPPER_F1,
    FakeClock,
    FakeEdgeBuilder,
    _node,
    _scaffold,
)


def _start(graph):
    return [n for n in graph.nodes.values()
            if n.region == "home" and n.load_state is LoadState.CARRYING][0]


def _goal(graph, family):
    return [n for n in graph.nodes.values()
            if n.region == "slot" and n.load_state is LoadState.CARRYING
            and n.ik_family == family][0]


class FromFailBuilder(FakeEdgeBuilder):
    """Fails one edge class when it starts from one of the given nodes."""

    def __init__(self, graph, edge_class, from_ids):
        super(FromFailBuilder, self).__init__(graph)
        self.edge_class = edge_class
        self.from_ids = set(from_ids)

    def __call__(self, from_node, to_node, edge_class, candidate=None,
                 scene=None):
        if edge_class == self.edge_class and \
                (not self.from_ids or from_node.node_id in self.from_ids):
            self.calls.append((from_node.node_id, to_node.node_id, edge_class))
            raise RoutePlanningError("EDGE_BUILD_FAILED", {"forced": True})
        return super(FromFailBuilder, self).__call__(
            from_node, to_node, edge_class, candidate=candidate, scene=scene)


class TestFrontierContinuation:
    def test_frontier_solved_node_is_tried_first(self):
        graph = _scaffold(RouteGraph(static_identity="static-test"),
                          families=("f1",))
        goal = _goal(graph, "f1")
        solved_q = tuple(v + 0.003 for v in Q_GOAL_F1)
        seen = []

        def continuation(frontier, hop, dests):
            if hop.edge_class != "DESCEND":
                return ()
            seen.append(frontier.q)
            node = _node(graph, solved_q, "slot", ik_family="frontier",
                         tcp=goal.tcp_position)
            _node(graph, solved_q, "slot", LoadState.UNLOADED, "frontier",
                  tcp=goal.tcp_position)
            return (node,)

        builder = FakeEdgeBuilder(graph)
        search = RouteSearch(graph, builder, RouteBudget(),
                             continuation=continuation)
        entry, _retreat, _goal_node = search.best_full_route(
            _start(graph).node_id, [goal])
        first = next(c for c in builder.calls if c[2] == "DESCEND")
        assert graph.node(first[1]).ik_family == "frontier"
        # The continuation saw the hover the chain actually reached.
        assert seen and seen[0] == graph.node(first[0]).q
        assert tuple(entry.edges[-1].trajectory.positions[-1]) == solved_q


class TestBacktracking:
    def test_failed_descend_backtracks_to_another_hover(self):
        graph = _scaffold(RouteGraph(static_identity="static-test"),
                          families=("f1", "f2"))
        upper_f1 = [n for n in graph.nodes_by_region("slot_upper")
                    if n.ik_family == "f1"][0]
        builder = FromFailBuilder(graph, "DESCEND", [upper_f1.node_id])
        search = RouteSearch(graph, builder, RouteBudget())
        entry, _retreat, goal = search.best_full_route(
            _start(graph).node_id, [_goal(graph, "f1")])
        descend_from = [c[0] for c in builder.calls if c[2] == "DESCEND"]
        assert descend_from[0] == upper_f1.node_id
        assert any(src != upper_f1.node_id for src in descend_from[1:])
        assert entry.edges[-1].attributes["edge_class"] == "DESCEND"
        assert goal.ik_family == "f1"


class TestFailureClassCap:
    def test_repeating_failure_class_is_capped_per_goal(self):
        graph = _scaffold(RouteGraph(static_identity="static-test"),
                          families=("f1",))
        for i in range(6):
            _node(graph, tuple(v + 0.001 * (i + 1) for v in Q_UPPER_F1),
                  "slot_upper", ik_family="extra%d" % i, tcp=(0.5, 0.0, 0.55))
        builder = FromFailBuilder(graph, "DESCEND", [])
        search = RouteSearch(graph, builder, RouteBudget())
        with pytest.raises(RoutePlanningError) as info:
            search.best_full_route(_start(graph).node_id,
                                   [_goal(graph, "f1")])
        descends = [c for c in builder.calls if c[2] == "DESCEND"]
        assert len(descends) == MAX_SAME_FAILURE_PER_GOAL
        err = info.value
        assert err.code == "NO_FEASIBLE_ROUTE"
        assert err.search_outcome == SEARCH_INCOMPLETE
        capped = err.detail["coverage"]["capped_classes"]
        assert capped and capped[0]["edge_class"] == "DESCEND"

    def test_cap_resets_for_the_next_goal(self):
        graph = _scaffold(RouteGraph(static_identity="static-test"),
                          families=("f1", "f2"))
        for i in range(4):
            _node(graph, tuple(v + 0.001 * (i + 1) for v in Q_UPPER_F1),
                  "slot_upper", ik_family="extra%d" % i, tcp=(0.5, 0.0, 0.55))
        builder = FromFailBuilder(graph, "DESCEND", [])
        search = RouteSearch(graph, builder, RouteBudget())
        with pytest.raises(RoutePlanningError) as info:
            search.best_full_route(_start(graph).node_id,
                                   [_goal(graph, "f1"), _goal(graph, "f2")])
        dests = {graph.node(c[1]).ik_family for c in builder.calls
                 if c[2] == "DESCEND"}
        assert dests == {"f1", "f2"}
        assert len(info.value.detail["coverage"]["goals_tried"]) == 2


class TestSearchOutcome:
    def test_exhausted_alternatives_are_unreachable(self):
        graph = _scaffold(RouteGraph(static_identity="static-test"),
                          families=("f1",))
        builder = FromFailBuilder(graph, "DESCEND", [])
        search = RouteSearch(graph, builder, RouteBudget())
        with pytest.raises(RoutePlanningError) as info:
            search.best_full_route(_start(graph).node_id,
                                   [_goal(graph, "f1")])
        assert info.value.search_outcome == UNREACHABLE
        assert info.value.detail["coverage"]["goals_open"] == []

    def test_budget_exhaustion_is_search_incomplete_with_open_goals(self):
        graph = _scaffold(RouteGraph(static_identity="static-test"),
                          families=("f1", "f2"))
        clock = FakeClock()

        class SlowFail(FromFailBuilder):
            def __call__(self, from_node, to_node, edge_class, **kw):
                clock.advance(4.0)
                return super(SlowFail, self).__call__(
                    from_node, to_node, edge_class, **kw)

        builder = SlowFail(graph, "DESCEND", [])
        search = RouteSearch(graph, builder, RouteBudget(clock=clock))
        with pytest.raises(RoutePlanningError) as info:
            search.best_full_route(_start(graph).node_id,
                                   [_goal(graph, "f1"), _goal(graph, "f2")])
        assert info.value.code == "PLANNING_BUDGET_EXHAUSTED"
        assert info.value.search_outcome == SEARCH_INCOMPLETE
        assert info.value.code != "BIN_FULL"

    def test_bare_budget_error_defaults_to_incomplete(self):
        err = RoutePlanningError("PLANNING_BUDGET_EXHAUSTED", {})
        assert err.search_outcome == SEARCH_INCOMPLETE


def _shift_refine(delta):
    """Fake exact IK: returns the seed shifted by ``delta`` (records seeds)."""
    calls = []

    def refine(position, quaternion, seed):
        calls.append((tuple(position), tuple(seed)))
        return tuple(float(v) + delta for v in seed), {}

    refine.calls = calls
    return refine


class TestPlannerLadderAndContinuation:
    def _planner(self, graph, refine):
        return GraphRoutePlanner(
            graph, FakeEdgeBuilder(graph),
            home_node_id=_start(graph).node_id, refine=refine)

    def test_ladder_back_seeds_each_region_up_to_the_transit(self):
        graph = _scaffold(RouteGraph(static_identity="static-test"),
                          families=("f1",))
        refine = _shift_refine(0.2)
        planner = self._planner(graph, refine)
        goal = _goal(graph, "f1")
        assert planner._register_branch_ladders([goal]) == 3
        ladder = [n for n in graph.nodes.values() if n.ik_family == "ladder"]
        assert sorted(n.region for n in ladder) == \
            ["portal", "slot_approach", "slot_upper"]
        upper = [n for n in ladder if n.region == "slot_upper"][0]
        assert upper.q == pytest.approx(tuple(v + 0.2 for v in goal.q))
        # Seeds chain: each region is refined from the next region's q.
        assert refine.calls[0][1] == tuple(goal.q)
        assert refine.calls[1][1] == pytest.approx(upper.q)

    def test_continuation_refines_from_the_frontier_q(self):
        graph = _scaffold(RouteGraph(static_identity="static-test"),
                          families=("f1",))
        refine = _shift_refine(0.002)
        planner = self._planner(graph, refine)
        solve = planner._frontier_continuation()
        frontier = [n for n in graph.nodes_by_region("slot_upper")][0]
        goal = _goal(graph, "f1")
        nodes = solve(frontier, RouteHop("slot", "DESCEND", "carrying"),
                      (goal,))
        assert len(nodes) == 1
        assert nodes[0].q == pytest.approx(
            tuple(v + 0.002 for v in frontier.q))
        assert refine.calls[0][1] == tuple(frontier.q)
        assert graph.node_for_q("unloaded", nodes[0].q) is not None
        assert solve(frontier, RouteHop("portal", "PORTAL_TRANSIT",
                                        "carrying"), (goal,)) == ()

    def test_no_refined_goal_is_search_incomplete_not_unreachable(self):
        graph = _scaffold(RouteGraph(static_identity="static-test"),
                          families=("f1",))
        planner = GraphRoutePlanner(
            graph, FakeEdgeBuilder(graph),
            home_node_id=_start(graph).node_id,
            goal_seeds=lambda target: [], refine=_shift_refine(0.0))
        with pytest.raises(RoutePlanningError) as info:
            planner.plan_route(None, None, None, [], budget_ns=int(1e9))
        assert info.value.code == "NO_FEASIBLE_ROUTE"
        assert info.value.search_outcome == SEARCH_INCOMPLETE

    def test_ladder_uses_plan_pose_when_anchor_refine_missed(self):
        graph = RouteGraph(static_identity="static-test")
        _node(graph, (0.0,) * 6, "home")
        goal = _node(graph, Q_GOAL_F1, "slot", tcp=(0.5, 0.0, 0.4))
        refine = _shift_refine(0.2)
        planner = GraphRoutePlanner(
            graph, FakeEdgeBuilder(graph),
            home_node_id=_start(graph).node_id, refine=refine,
            region_poses={"slot_upper": (0.5, 0.0, 0.5),
                          "slot_approach": (0.5, 0.0, 0.6),
                          "portal": (0.3, 0.0, 0.6)})
        assert planner._register_branch_ladders([goal]) == 3
        upper = graph.nodes_by_region("slot_upper")[0]
        assert upper.tcp_position == (0.5, 0.0, 0.5)
        assert upper.tcp_quaternion_xyzw == goal.tcp_quaternion_xyzw

    def test_plan_pose_beats_an_earlier_candidates_anchor(self):
        graph = _scaffold(RouteGraph(static_identity="static-test"),
                          families=("f1",))  # stale anchors at z 0.5/0.55
        goal = _goal(graph, "f1")
        planner = GraphRoutePlanner(
            graph, FakeEdgeBuilder(graph),
            home_node_id=_start(graph).node_id, refine=_shift_refine(0.2),
            region_poses={"slot_upper": (0.9, 0.1, 0.8)})
        assert planner._region_pose("slot_upper", goal)[0] == (0.9, 0.1, 0.8)

    def test_retreat_continuation_uses_plan_exit_pose(self):
        graph = _scaffold(RouteGraph(static_identity="static-test"),
                          families=("f1",))
        refine = _shift_refine(0.2)
        planner = GraphRoutePlanner(
            graph, FakeEdgeBuilder(graph),
            home_node_id=_start(graph).node_id, refine=refine,
            region_poses={"exit": (0.5, 0.0, 0.7)})
        twin = [n for n in graph.nodes.values() if n.region == "slot"
                and n.load_state is LoadState.UNLOADED][0]
        # No exit destination survived anchor refine: empty destinations.
        nodes = planner._frontier_continuation()(
            twin, RouteHop("exit", "RETREAT", "unloaded"), ())
        assert len(nodes) == 1
        assert nodes[0].region == "exit"
        assert str(nodes[0].load_state) == "unloaded"
        assert nodes[0].tcp_quaternion_xyzw == twin.tcp_quaternion_xyzw
