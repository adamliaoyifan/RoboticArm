"""ST09-B acceptance (e): the complete route end to end on real MoveIt.

plan_route through GraphRoutePlanner on the mock graph: goal nodes from
pure target synthesis, the full entry chain (PORTAL_TRANSIT -> TRAVERSE
-> INSERT -> DESCEND) and the post-release retreat (RETREAT -> RETURN)
with the released box as a world obstacle — every edge a complete timed
trajectory (canonical joint order, velocities, accelerations, strictly
monotonic ns), same-XYZ different-q nodes kept distinct, and all route
digests recomputable from the graph.
"""

from __future__ import division

import dataclasses
import math

import pytest

from luggage_planning.planning_contract.attachment import PlaceTarget  # noqa: E402
from luggage_planning.planning_contract.fixtures import make_attachment, make_snapshot  # noqa: E402
from luggage_planning.planning_contract.graph import LoadState  # noqa: E402
from luggage_planning.planning_contract.planning import (  # noqa: E402
    GeometryVerdict,
    PlacementCandidate,
)

from luggage_planning.routing.moveit_edge_adapter import MoveItEdgeAdapter  # noqa: E402
from luggage_planning.place.place_targets import GoalSeed  # noqa: E402
from luggage_planning.atlas.graph import (  # noqa: E402
    GraphRoutePlanner,
    RouteGraph,
    make_offline_fk,
)

CANONICAL_JOINTS = tuple("elfin_joint%d" % i for i in range(1, 7))


@pytest.fixture(scope="module")
def chain(mock_graph):
    from route_harness import build_route_chain
    return build_route_chain(mock_graph.node)


class TestFullRoute:

    def test_plan_route_entry_and_retreat(self, chain):
        route = chain["planner"].plan_route(
            make_snapshot(1), chain["attachment"], chain["candidate"],
            (chain["target"],), budget_ns=10_000_000_000)
        graph = chain["graph"]
        entry_classes = [graph.edges[eid].attributes["edge_class"]
                         for eid in route.entry_edge_ids]
        retreat_classes = [graph.edges[eid].attributes["edge_class"]
                           for eid in route.retreat_edge_ids]
        assert entry_classes == ["PORTAL_TRANSIT", "TRAVERSE", "INSERT",
                                 "DESCEND"]
        assert retreat_classes == ["RETREAT", "RETURN"]
        assert route.planning_revision.startswith("B:")
        assert route.request_id == chain["attachment"].request_id
        assert route.candidate_id == chain["candidate"].candidate_id

    def test_every_edge_is_a_complete_timed_trajectory(self, chain):
        route = chain["planner"].plan_route(
            make_snapshot(1), chain["attachment"], chain["candidate"],
            (chain["target"],), budget_ns=10_000_000_000)
        graph = chain["graph"]
        for edge_id in route.entry_edge_ids + route.retreat_edge_ids:
            trajectory = graph.edges[edge_id].trajectory
            assert trajectory.joint_names == CANONICAL_JOINTS
            times = trajectory.time_from_start_ns
            assert len(times) >= 2 and times[0] == 0
            assert all(b > a for a, b in zip(times, times[1:]))
            rows = len(trajectory.positions)
            assert len(trajectory.velocities) == rows
            assert len(trajectory.accelerations) == rows
            assert len(trajectory.time_from_start_ns) == rows
            for row in trajectory.positions:
                assert len(row) == 6

    def test_route_digests_recompute_from_graph(self, chain):
        from luggage_planning.planning_contract._base import digest_of
        route = chain["planner"].plan_route(
            make_snapshot(1), chain["attachment"], chain["candidate"],
            (chain["target"],), budget_ns=10_000_000_000)
        graph = chain["graph"]
        edges = [graph.edges[eid] for eid in
                 route.entry_edge_ids + route.retreat_edge_ids]
        assert route.edge_digests == tuple(e.digest() for e in edges)
        assert route.trajectory_digest == digest_of(
            *(e.trajectory.digest() for e in edges))

    def test_release_twin_and_distinct_q_nodes(self, chain):
        chain["planner"].plan_route(
            make_snapshot(1), chain["attachment"], chain["candidate"],
            (chain["target"],), budget_ns=10_000_000_000)
        graph = chain["graph"]
        carrying = [n for n in graph.nodes_by_region("slot")
                    if n.load_state is LoadState.CARRYING]
        twins = [n for n in graph.nodes_by_region("slot")
                 if n.load_state is LoadState.UNLOADED]
        assert len(carrying) == 1 and 1 <= len(twins) <= 2
        # [ST09-I] the pre-seeded twin plus, when the entry's cartesian
        # descend lands a different IK branch than the bookkept goal, the
        # continuity anchor at the ACTUAL endpoint.
        assert any(carrying[0].q == twin.q for twin in twins)
        assert all(carrying[0].node_id != twin.node_id for twin in twins)
        # Same TCP XYZ with a different q registers a distinct node.
        other = graph.register_node(
            tuple(v + 0.2 for v in chain["q_goal"]), chain["goal_position"],
            (0.0, 0.0, 0.0, 1.0), "slot", LoadState.CARRYING, "f2")
        assert other.node_id != carrying[0].node_id
        assert other.tcp_position == carrying[0].tcp_position

    def test_within_budget_with_evidence(self, chain):
        chain["planner"].plan_route(
            make_snapshot(1), chain["attachment"], chain["candidate"],
            (chain["target"],), budget_ns=10_000_000_000)
        snapshot = chain["planner"].last_budget_snapshot
        assert snapshot["elapsed_s"] < 10.0
        assert snapshot["policy"]["total_wall_s"] == 10.0
        counts = chain["adapter"].moveit_call_counts()
        assert counts["move_group"] >= 2  # portal transit + return
        assert counts["cartesian"] >= 3   # traverse, insert, descend/retreat
        assert chain["adapter"].request_dumps()

    def test_second_request_reuses_verified_edges(self, chain):
        first = chain["planner"].plan_route(
            make_snapshot(1), chain["attachment"], chain["candidate"],
            (chain["target"],), budget_ns=10_000_000_000)
        before = len(chain["adapter"].request_dumps())
        second = chain["planner"].plan_route(
            make_snapshot(1), chain["attachment"], chain["candidate"],
            (chain["target"],), budget_ns=10_000_000_000)
        assert len(chain["adapter"].request_dumps()) == before  # cache hits
        assert second.edge_digests == first.edge_digests
