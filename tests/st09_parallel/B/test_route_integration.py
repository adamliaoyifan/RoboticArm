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

from st09_contracts.attachment import PlaceTarget  # noqa: E402
from st09_contracts.fixtures import make_attachment, make_snapshot  # noqa: E402
from st09_contracts.graph import LoadState  # noqa: E402
from st09_contracts.planning import (  # noqa: E402
    GeometryVerdict,
    PlacementCandidate,
)

from luggage_planning.moveit_edge_adapter import MoveItEdgeAdapter  # noqa: E402
from luggage_planning.place_targets import GoalSeed  # noqa: E402
from luggage_planning.reachable_graph import (  # noqa: E402
    GraphRoutePlanner,
    RouteGraph,
    make_offline_fk,
)

CANONICAL_JOINTS = tuple("elfin_joint%d" % i for i in range(1, 7))


@pytest.fixture(scope="module")
def chain(mock_graph):
    """Real-FK region chain: home -> portal -> approach -> upper -> goal."""
    try:
        from luggage_planning.offline_ik import load_urdf, refine_to_pose
        urdf = load_urdf()
    except Exception as err:
        pytest.skip("offline_ik unavailable: %s" % err)
    path = urdf.path_to("suction_contact_frame")
    fk = make_offline_fk(urdf, path)

    q_home = (0.586, -1.025, 1.268, -1.819, -1.567, -0.635)
    q_portal = (0.936, -1.025, 1.268, -1.819, -1.567, -0.635)

    def refined(seed_q, offset):
        position, quaternion = fk(seed_q)
        target = (position[0] + offset[0], position[1] + offset[1],
                  position[2] + offset[2])
        joints, info = refine_to_pose(urdf, path, target, quaternion, seed_q)
        if joints is None:
            pytest.skip("refine did not converge on this host: %r" % (info,))
        return tuple(joints)

    q_approach = refined(q_portal, (0.10, 0.0, 0.0))
    q_upper = refined(q_approach, (0.10, 0.0, 0.0))
    q_goal = refined(q_upper, (0.0, 0.0, -0.05))

    graph = RouteGraph(static_identity="static-integration")
    home = graph.register_node(q_home, *fk(q_home), "home",
                               LoadState.CARRYING)
    graph.register_node(q_portal, *fk(q_portal), "portal",
                        LoadState.CARRYING)
    graph.register_node(q_approach, *fk(q_approach), "slot_approach",
                        LoadState.CARRYING)
    graph.register_node(q_upper, *fk(q_upper), "slot_upper",
                        LoadState.CARRYING)
    graph.register_node(q_upper, *fk(q_upper), "exit", LoadState.UNLOADED)
    graph.register_node(q_home, *fk(q_home), "home", LoadState.UNLOADED)

    goal_position, goal_quaternion = fk(q_goal)
    attachment = dataclasses.replace(
        make_attachment("centered", symmetry="none"),
        dimensions_wdh=(0.05, 0.05, 0.05),
        t_tcp_box_translation=(0.0, 0.0, -0.025))
    candidate = PlacementCandidate(
        candidate_id="cand-b-int",
        footprint_class=0,
        heading_theta=0.0,
        position_container=(goal_position[0], goal_position[1],
                            goal_position[2] - 0.02 - 0.025),
        quaternion_xyzw_container=(0.0, 0.0, 0.0, 1.0),
        support_layer_z=0.0,
        depth_m=0.6,
        verdict=GeometryVerdict.LEGAL,
        score=1.0,
        detail={"route_clearance_m": 0.05},
    )
    target = PlaceTarget(
        candidate_id=candidate.candidate_id,
        heading_theta=0.0,
        tcp_frame=attachment.tcp_frame,
        frame_id="world",
        position=goal_position,
        quaternion_xyzw=goal_quaternion,
        attachment_digest=attachment.digest())

    adapter = MoveItEdgeAdapter(mock_graph.node)
    adapter.wait_ready(timeout_sec=60.0)
    planner = GraphRoutePlanner(
        graph, adapter,
        home_node_id=home.node_id,
        goal_seeds=lambda _t: [GoalSeed(q_goal, "f1", "integration-seed")],
        urdf=urdf, tcp_path=path)
    return {
        "graph": graph, "planner": planner, "adapter": adapter,
        "attachment": attachment, "candidate": candidate,
        "target": target, "q_goal": q_goal,
        "goal_position": goal_position,
    }


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
        assert route.candidate_id == "cand-b-int"

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
        from st09_contracts._base import digest_of
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
        assert len(carrying) == 1 and len(twins) == 1
        assert carrying[0].q == twins[0].q
        assert carrying[0].node_id != twins[0].node_id
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
