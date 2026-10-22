"""ST09-B: the plan-only MoveIt edge adapter.

Offline layer (no ROS): edge-class/planner mapping, orientation-aware
waypoint synthesis (hold vs slerp, scipy-checked), snapshot scene boxes,
judge math, contract pins, and module importability without ROS.

Mock-graph layer (real move_group on mock controllers, isolated domain):
per-class edge builds returning complete timed trajectories (order,
velocities, accelerations, monotonic ns preserved), carrying/unloaded
scene diffs through the production planning-scene client, and the
request/scene/duration evidence collectors.
"""

from __future__ import division

import json
import math
import os
import subprocess
import sys
from pathlib import Path

import pytest

np = pytest.importorskip("numpy")
scipy_rot = pytest.importorskip("scipy.spatial.transform").Rotation

from st09_contracts.graph import LoadState  # noqa: E402
from st09_contracts.fixtures import (  # noqa: E402
    BOX_WDH,
    make_attachment,
    make_snapshot,
)

from luggage_planning.moveit_edge_adapter import (  # noqa: E402
    CARTESIAN_EXCURSION_RAD_PER_M,
    CARTESIAN_MIN_FRACTION,
    MOVEIT_ACCELERATION_SCALING,
    MOVEIT_VELOCITY_SCALING,
    SINGLE_EDGE_WALL_S,
    WAYPOINT_SPACING_M,
    BoxSpec,
    EdgeClass,
    ORIENTATION_MODE,
    PayloadSpec,
    SceneState,
    _joint_travel,
    _path_length,
    _slerp,
    scene_boxes_from_snapshot,
    synthesize_waypoints,
)

REPO_ROOT = Path(__file__).resolve().parents[3]

#: Known-reachable sim-scene contact seed (2026-09-24 adaptive-lift
#: evidence, mirrored from test_frame_migration_atlas).
Q_SEED = (0.936, -1.025, 1.268, -1.819, -1.567, -0.635)


class TestContractPins:

    @staticmethod
    def _value(entry):
        return entry["value"] if isinstance(entry, dict) else entry

    def test_judge_values_match_resolved_contract(self):
        contract = json.loads(
            (REPO_ROOT / "st09_contracts" / "resolved_planning_contract.json")
            .read_text())
        timing = contract["time_parameterization"]
        assert CARTESIAN_MIN_FRACTION == \
            self._value(timing["cartesian_min_fraction"])
        assert CARTESIAN_EXCURSION_RAD_PER_M == self._value(
            contract["tool_orientation"]["cartesian_excursion_rad_per_m"])
        assert MOVEIT_VELOCITY_SCALING == \
            self._value(timing["moveit_velocity_scaling"])
        assert MOVEIT_ACCELERATION_SCALING == \
            self._value(timing["moveit_acceleration_scaling"])
        budgets = contract["planning_budgets"]
        assert SINGLE_EDGE_WALL_S == \
            self._value(budgets["single_new_moveit_edge_wall_s"])

    def test_module_importable_without_ros(self, tmp_path):
        env = os.environ.copy()
        env["PYTHONPATH"] = str(REPO_ROOT / "src" / "luggage_planning")
        result = subprocess.run(
            [sys.executable, "-c",
             "import luggage_planning.moveit_edge_adapter"],
            env=env, cwd=str(tmp_path), capture_output=True, text=True)
        assert result.returncode == 0, result.stderr


class TestEdgeClasses:

    def test_planner_mapping(self):
        assert EdgeClass.PORTAL_TRANSIT.planner == "joint"
        assert EdgeClass.RETURN.planner == "joint"
        for klass in (EdgeClass.TRAVERSE, EdgeClass.INSERT,
                      EdgeClass.DESCEND, EdgeClass.RETREAT,
                      EdgeClass.ORIENTATION_TRANSITION):
            assert klass.planner == "cartesian"

    def test_orientation_modes_cover_all_cartesian_classes(self):
        cartesian = {k for k in EdgeClass if k.planner == "cartesian"}
        assert cartesian == set(ORIENTATION_MODE)


class TestWaypointSynthesis:

    def test_hold_source_traverse(self):
        source_quat = scipy_rot.from_euler("z", 0.4).as_quat()
        destination_quat = scipy_rot.from_euler("z", 2.4).as_quat()
        poses, distance = synthesize_waypoints(
            (0.0, 0.0, 0.5), source_quat, (0.3, 0.0, 0.5), destination_quat,
            EdgeClass.TRAVERSE, spacing_m=0.10)
        assert distance == pytest.approx(0.3, abs=1e-9)
        assert len(poses) == 3  # 0.30 m / 0.10 m spacing
        for position, quaternion in poses:
            assert (scipy_rot.from_quat(quaternion).inv()
                    * scipy_rot.from_quat(source_quat)).magnitude() <= 1e-9

    def test_hold_destination_descend(self):
        source_quat = scipy_rot.from_euler("z", 0.0).as_quat()
        destination_quat = scipy_rot.from_euler("xyz", (0.0, 0.0, math.pi / 2))
        poses, _distance = synthesize_waypoints(
            (0.2, 0.1, 0.6), source_quat, (0.2, 0.1, 0.4),
            destination_quat.as_quat(), EdgeClass.DESCEND, spacing_m=0.05)
        for _position, quaternion in poses:
            relative = scipy_rot.from_quat(quaternion).inv() * destination_quat
            assert relative.magnitude() <= 1e-9

    def test_slerp_transition_accompanies_translation(self):
        source_quat = scipy_rot.from_euler("z", 0.0).as_quat()
        destination_quat = scipy_rot.from_euler("z", math.pi / 2).as_quat()
        poses, distance = synthesize_waypoints(
            (0.0, 0.0, 0.5), source_quat, (0.2, 0.0, 0.5), destination_quat,
            EdgeClass.ORIENTATION_TRANSITION, spacing_m=0.05)
        assert distance == pytest.approx(0.2, abs=1e-9)
        expected = scipy_rot.from_euler("z", 0.0)
        destination_rotation = scipy_rot.from_euler("z", math.pi / 2)
        for index, (_position, quaternion) in enumerate(poses):
            fraction = (index + 1) / len(poses)
            expect = (expected.inv() * scipy_rot.from_quat(quaternion)
                      ).magnitude() / (math.pi / 2)
            assert abs(expect - fraction) <= 0.02  # rotation tracks travel
        final = scipy_rot.from_quat(poses[-1][1])
        assert (final.inv() * destination_rotation).magnitude() <= 1e-9

    def test_slerp_matches_scipy(self):
        from scipy.spatial.transform import Slerp

        a = scipy_rot.from_euler("xyz", (0.1, -0.2, 0.3))
        b = scipy_rot.from_euler("xyz", (-0.4, 0.2, 1.9))
        slerp = Slerp([0.0, 1.0], scipy_rot.from_quat(
            np.array([a.as_quat(), b.as_quat()])))
        for t in (0.0, 0.25, 0.5, 0.75, 1.0):
            got = _slerp(a.as_quat(), b.as_quat(), t)
            assert (scipy_rot.from_quat(got).inv()
                    * slerp([t])).magnitude() <= 1e-9


class TestSceneHelpers:

    def test_snapshot_boxes_only_occupied(self):
        # Cells in the observed-free opening half (i < 10) cannot model
        # placed cargo; use deep cells like the revision-pair fixture.
        snapshot = make_snapshot(
            2, observed_free=[(1, 1), (2, 2)], occupied=[(14, 5), (15, 6)])
        boxes = scene_boxes_from_snapshot(snapshot)
        assert len(boxes) == 2
        assert all(box.object_id.startswith("snapshot-cell-") for box in boxes)
        sizes = {round(box.size_whd[0], 6) for box in boxes}
        assert sizes == {round(snapshot.resolution_m + 0.10, 6)}

    def test_unknown_never_becomes_free_geometry(self):
        snapshot = make_snapshot(1)  # all unknown
        assert scene_boxes_from_snapshot(snapshot) == ()

    def test_path_length_and_joint_travel(self):
        poses = [((0.0, 0.0, 0.0), (0, 0, 0, 1)),
                 ((0.3, 0.0, 0.0), (0, 0, 0, 1)),
                 ((0.3, 0.4, 0.0), (0, 0, 0, 1))]
        assert _path_length(poses) == pytest.approx(0.7, abs=1e-9)

        class _Traj(object):
            positions = ((0.0,) * 6, (0.1, 0.0, 0.0, 0.0, 0.0, 0.0),
                         (0.1, 0.0, 0.2, 0.0, 0.0, 0.0))

        # Max per-joint total variation (j1 0.1, j3 0.2), not a cross-joint sum.
        assert _joint_travel(_Traj()) == pytest.approx(0.2, abs=1e-12)

    def test_excursion_ratio_semantics(self):
        from luggage_planning.moveit_edge_adapter import _excursion

        class _Traj(object):
            positions = ((0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                         (1.0, 0.0, 0.0, 0.0, 0.0, 0.0))

        excursion = _excursion(_Traj(), path_len_m=0.5)
        assert excursion["worst_joint"] == "elfin_joint1"
        assert excursion["ratio"] == pytest.approx(2.0)
        short = _excursion(_Traj(), path_len_m=0.005)
        assert short["ratio"] == pytest.approx(1.0 / 0.05)  # floored denominator

    def test_joint_motion_report_names_a_pi_wrist_flip(self):
        from luggage_planning.joint_path import joint_motion_report

        start = (0.1, -0.2, 0.8, -0.3, 1.2, 0.4)
        # One sample moves joint 6 by pi. The other joints stay put.
        end = (0.1, -0.2, 0.8, -0.3, 1.2, 0.4 + math.pi)
        report = joint_motion_report((start, end), path_length_m=0.053)
        assert report["worst_joint_index"] == 5
        assert abs(report["end_delta_rad"][5]) == pytest.approx(math.pi, abs=1e-6)
        assert report["cumulative_rad"][5] == pytest.approx(math.pi, abs=1e-6)
        assert report["max_step_rad"][5] == pytest.approx(math.pi, abs=1e-6)
        assert report["ratio"] == pytest.approx(math.pi / 0.053, abs=1e-6)


# -- mock-graph layer ------------------------------------------------------

@pytest.fixture(scope="module")
def _fk():
    """World-rooted offline FK: the full joint path includes the patched
    world->elfin_base_link fixed joint, so node TCP poses and cartesian
    requests share move_group's planning frame."""
    try:
        from luggage_planning.offline_ik import load_urdf
        urdf = load_urdf()
    except Exception as err:
        pytest.skip("offline_ik unavailable: %s" % err)
    from luggage_planning.reachable_graph import make_offline_fk
    path = urdf.path_to("suction_contact_frame")
    return urdf, path, make_offline_fk(urdf, path)


def _refined_below(urdf, tcp_path, fk, drop_m=0.05):
    from luggage_planning.offline_ik import refine_to_pose
    position, quaternion = fk(Q_SEED)
    target = (position[0], position[1], position[2] - drop_m)
    joints, info = refine_to_pose(urdf, tcp_path, target, quaternion, Q_SEED)
    if joints is None:
        pytest.skip("IK refine did not converge on this host: %r" % (info,))
    return tuple(joints), target, quaternion


def _nodes(graph, fk, upper_q, lower_q):
    upper_pose, lower_pose = fk(upper_q), fk(lower_q)
    n_upper = graph.register_node(upper_q, upper_pose[0], upper_pose[1],
                                  "slot_upper", LoadState.CARRYING, "f1")
    n_lower = graph.register_node(lower_q, lower_pose[0], lower_pose[1],
                                  "slot", LoadState.CARRYING, "f1")
    return n_upper, n_lower


@pytest.fixture(scope="module")
def _graph(_fk):
    from luggage_planning.reachable_graph import RouteGraph
    urdf, tcp_path, fk = _fk
    lower_q, _target, _quat = _refined_below(urdf, tcp_path, fk)
    graph = RouteGraph(static_identity="static-test")
    nodes = _nodes(graph, fk, Q_SEED, lower_q)
    return graph, nodes, (urdf, tcp_path, fk), lower_q


class TestMockGraphEdges:

    def test_descend_edge_full_trajectory(self, mock_graph, _graph):
        from luggage_planning.moveit_edge_adapter import MoveItEdgeAdapter
        graph, (n_upper, n_lower), _ctx, _lower_q = _graph
        adapter = MoveItEdgeAdapter(mock_graph.node)
        adapter.wait_ready(timeout_sec=60.0)
        edge = adapter.build_edge(n_upper, n_lower, EdgeClass.DESCEND)
        trajectory = edge.trajectory
        assert trajectory.joint_names == tuple(
            "elfin_joint%d" % i for i in range(1, 7))
        times = trajectory.time_from_start_ns
        assert all(b > a for a, b in zip(times, times[1:]))  # strict ns
        assert times[0] == 0
        assert len(trajectory.positions) == len(velocities_times(trajectory))
        assert edge.attributes["edge_class"] == "DESCEND"
        assert edge.attributes["fraction"] >= CARTESIAN_MIN_FRACTION
        assert edge.destination_node_id == n_lower.node_id
        assert adapter.moveit_call_counts()["cartesian"] >= 1
        assert adapter.request_dumps()
        assert adapter.outcomes()[-1].wall_s > 0.0

    def test_joint_edge_via_rrtconnect(self, mock_graph, _graph):
        from luggage_planning.moveit_edge_adapter import MoveItEdgeAdapter
        graph, (n_upper, n_lower), _ctx, _lower_q = _graph
        far_q = tuple(v + 0.35 for v in Q_SEED)
        urdf, tcp_path, fk = _graph[2]
        far_pose = fk(far_q)
        n_far = graph.register_node(far_q, far_pose[0], far_pose[1],
                                    "portal", LoadState.CARRYING, "f1")
        adapter = MoveItEdgeAdapter(mock_graph.node)
        adapter.wait_ready(timeout_sec=60.0)
        edge = adapter.build_edge(n_upper, n_far, EdgeClass.PORTAL_TRANSIT)
        assert edge.attributes["planner"] == "joint"
        assert edge.trajectory.time_from_start_ns[-1] > 0
        assert adapter.moveit_call_counts()["move_group"] == 1

    def test_scene_diffs_carrying_and_unloaded(self, mock_graph, _graph):
        import dataclasses

        from luggage_planning.moveit_edge_adapter import MoveItEdgeAdapter
        graph, (n_upper, n_lower), _ctx, _lower_q = _graph
        # A suspended small payload exercises the attach/verify/reuse/
        # detach bookkeeping without needing the certifier's
        # support-contact exemption - a full-size box resting at this
        # deep contact pose is raw collision to a plan-only judge (see
        # test_full_size_payload_participates_in_collision).
        record = dataclasses.replace(
            make_attachment("eccentric"),
            dimensions_wdh=(0.05, 0.05, 0.05),
            t_tcp_box_translation=(0.015, 0.008, -0.025))
        payload = PayloadSpec(
            dimensions_wdh=record.dimensions_wdh,
            t_tcp_box_translation=record.t_tcp_box_translation,
            t_tcp_box_quaternion_xyzw=record.t_tcp_box_quaternion_xyzw)
        carrying = SceneState(load_state="carrying", payload=payload)
        adapter = MoveItEdgeAdapter(mock_graph.node)
        adapter.wait_ready(timeout_sec=60.0)
        edge = adapter.build_edge(n_upper, n_lower, EdgeClass.DESCEND,
                                  scene=carrying)
        assert adapter.moveit_call_counts()["scene_apply"] >= 1
        before = adapter.moveit_call_counts()["scene_apply"]
        # Same scene marker: a second carrying edge reuses the applied diff
        # without another scene round-trip.
        graph2_edge = adapter.build_edge(n_upper, n_lower, EdgeClass.INSERT,
                                         scene=carrying)
        assert graph2_edge.attributes["edge_class"] == "INSERT"
        assert adapter.moveit_call_counts()["scene_apply"] == before
        # Released box rests below the retreat start with a release gap,
        # at the slot's actual world position - otherwise the unloaded
        # start state sits inside its own obstacle.
        slot_tcp = n_lower.tcp_position
        release_gap_m = 0.02
        released = BoxSpec(
            "released-1",
            (slot_tcp[0], slot_tcp[1],
             slot_tcp[2] - release_gap_m - 0.5 * BOX_WDH[2]),
            (BOX_WDH[0], BOX_WDH[1], BOX_WDH[2]),
            (0.0, 0.0, 0.0, 1.0))
        unloaded = SceneState(load_state="unloaded", released_box=released)
        n_twin_upper = graph.register_node(
            n_upper.q, n_upper.tcp_position, n_upper.tcp_quaternion_xyzw,
            "slot_upper", LoadState.UNLOADED, "f1")
        n_twin_lower = graph.register_node(
            n_lower.q, n_lower.tcp_position, n_lower.tcp_quaternion_xyzw,
            "slot", LoadState.UNLOADED, "f1")
        retreat = adapter.build_edge(n_twin_lower, n_twin_upper,
                                     EdgeClass.RETREAT, scene=unloaded)
        assert retreat.attributes["edge_class"] == "RETREAT"
        after = adapter.moveit_call_counts()["scene_apply"]
        assert after > before  # detach + released box applied
        assert adapter.scene_dumps()


    def test_full_size_payload_participates_in_collision(self, mock_graph,
                                                         _graph):
        """Positive control: the attached payload is real oriented
        geometry. The same DESCEND that succeeds with a suspended small
        payload must be a NAMED EDGE_BUILD_FAILED with the full-size
        eccentric box at this deep contact pose (support-contact
        exemption belongs to the certifier, not this plan-only judge)."""
        from luggage_planning.grid_path_search import RoutePlanningError
        from luggage_planning.moveit_edge_adapter import MoveItEdgeAdapter

        graph, (n_upper, n_lower), _ctx, _lower_q = _graph
        record = make_attachment("eccentric")
        carrying = SceneState(load_state="carrying", payload=PayloadSpec(
            dimensions_wdh=record.dimensions_wdh,
            t_tcp_box_translation=record.t_tcp_box_translation,
            t_tcp_box_quaternion_xyzw=record.t_tcp_box_quaternion_xyzw))
        adapter = MoveItEdgeAdapter(mock_graph.node)
        adapter.wait_ready(timeout_sec=60.0)
        with pytest.raises(RoutePlanningError) as err:
            adapter.build_edge(n_upper, n_lower, EdgeClass.DESCEND,
                               scene=carrying)
        assert err.value.code == "EDGE_BUILD_FAILED"
        assert err.value.detail["kind"] in ("fraction_below_min",
                                            "excursion_exceeded")
        assert adapter.edge_failure_log()


def velocities_times(trajectory):
    return trajectory.velocities


def test_graph_carrying_and_unloaded_scenes_share_occupied_snapshot():
    from types import SimpleNamespace
    from luggage_planning.reachable_graph import GraphRoutePlanner
    snapshot = make_snapshot(2, observed_free=[(1, 1)], occupied=[(14, 5)])
    planner = object.__new__(GraphRoutePlanner)
    planner.scene_states = None
    # The graph receives measured metric columns from the producer; the
    # snapshot's occupied-cell bits alone no longer synthesize box heights.
    from luggage_planning.moveit_edge_adapter import BoxSpec
    planner.occupancy_boxes = (BoxSpec('snapshot-cell-14-5', (.7,.25,.1), (.05,.05,.2)),)
    released = object()
    planner._released_box = lambda attachment, candidate: released
    attachment = SimpleNamespace(dimensions_wdh=(.4, .3, .2),
                                 t_tcp_box_translation=(.02, .03, .1),
                                 t_tcp_box_quaternion_xyzw=(0, 0, 0, 1))
    carrying, unloaded = planner._scenes(attachment, None, snapshot)
    assert carrying.placed_boxes == unloaded.placed_boxes
    assert len(carrying.placed_boxes) == 1
    assert carrying.placed_boxes[0].object_id.startswith('snapshot-cell-')
    assert carrying.payload is not None
    assert unloaded.payload is None
    assert unloaded.released_box is released
