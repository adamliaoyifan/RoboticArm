"""Shared real-MoveIt route harness for the ST09-B mock-graph tests.

Builds the region chain home -> portal -> slot_approach -> slot_upper ->
goal from real offline FK + IK refinement, plus the planner/adapter pair
and the small suspended payload attachment the plan-only judge can carry
without the certifier's support-contact exemption.
"""

from __future__ import division

import dataclasses

import pytest

from luggage_planning.planning_contract.attachment import PlaceTarget
from luggage_planning.planning_contract.fixtures import make_attachment
from luggage_planning.planning_contract.graph import LoadState
from luggage_planning.planning_contract.planning import GeometryVerdict, PlacementCandidate

from luggage_planning.routing.moveit_edge_adapter import MoveItEdgeAdapter
from luggage_planning.place.place_targets import GoalSeed
from luggage_planning.atlas.graph import (
    GraphRoutePlanner,
    RouteGraph,
    make_offline_fk,
)


def build_route_chain(mock_graph_node):
    """Return the graph/planner/adapter/target bundle (skips without IK)."""
    try:
        from luggage_planning.motion.offline_ik import load_urdf, refine_to_pose
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
        candidate_id="cand-b-bench",
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

    adapter = MoveItEdgeAdapter(mock_graph_node)
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
