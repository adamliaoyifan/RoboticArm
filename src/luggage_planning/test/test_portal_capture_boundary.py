"""Portal hook and Select-request capture: behaviour, verified separately."""

import pytest
import json
import os
from pathlib import Path

from luggage_msgs.srv import SelectPlacePathV2
from moveit_msgs.msg import AllowedCollisionMatrix, PlanningScene

from luggage_planning.boundary_capture import (
    capture_service_request,
    load_service_request,
)
from luggage_planning.moveit_edge_adapter import MoveItEdgeAdapter
from luggage_planning.planning_scene_client import set_acm_pair
from luggage_planning.portal_goal_classifier import (
    classify_portal_record,
    urdf_joint_limits,
)

URDF = """
<joint name="elfin_joint1" type="revolute"><limit lower="-6.28" upper="6.28"/></joint>
<joint name="elfin_joint2" type="revolute"><limit lower="-3.3158" upper="0.1745"/></joint>
<joint name="elfin_joint3" type="revolute"><limit lower="-2.9319" upper="2.9319"/></joint>
<joint name="elfin_joint4" type="revolute"><limit lower="-6.28" upper="6.28"/></joint>
<joint name="elfin_joint5" type="revolute"><limit lower="-6.28" upper="6.28"/></joint>
<joint name="elfin_joint6" type="revolute"><limit lower="-6.28" upper="6.28"/></joint>
"""
NAMES = ["elfin_joint%d" % i for i in range(1, 7)]


class _Node(object):
    def __init__(self, q):
        self.node_id = "n"
        self.q = tuple(q)


class _Constraint(object):
    def __init__(self, name, value, tol=1e-4):
        self.joint_name = name
        self.position = value
        self.tolerance_above = tol
        self.tolerance_below = tol


class _Host(object):
    """Mutates the ACM in place, like the real _goal_on_submission_scene."""

    def _goal_on_submission_scene(self, scene_msg, goal_q):
        scene_msg.allowed_collision_matrix = AllowedCollisionMatrix()
        scene_msg.is_diff = False
        return {"status": "queried", "valid": False, "reason": "COLLISION",
                "contacts": [{"body_1": "a", "body_2": "b"}]}


def _constraints(goal):
    class C(object):
        joint_constraints = [_Constraint(n, v) for n, v in zip(NAMES, goal)]
    return C()


def _scene():
    scene = PlanningScene()
    set_acm_pair(scene.allowed_collision_matrix, "elfin_base", "pedestal", True)
    return scene


def test_hook_separates_goals_that_round_to_the_same_3dp(tmp_path):
    os.environ["ST09_PORTAL_CAPTURE_DIR"] = str(tmp_path)
    host = _Host()
    start = _Node([0.0] * 6)
    near_a = _Node([1.0001, -1.0, 0.2, -0.3, 0.4, 0.5])
    near_b = _Node([1.00014, -1.0, 0.2, -0.3, 0.4, 0.5])  # same 3dp as near_a
    for goal in (near_a, near_b, near_a):
        MoveItEdgeAdapter._keep_portal_failure(
            host, start, goal, -27, _scene(), _constraints(goal.q))
    files = sorted(Path(tmp_path).glob("portal_[0-9][0-9].json"))
    assert len(files) == 2  # not merged by rounding
    index = json.loads((Path(tmp_path) / "capture_index.json").read_text())
    assert sorted(e["occurrences"] for e in index["entries"]) == [1, 2]
    assert index["entries"][0]["signature_3dp"] == \
        index["entries"][1]["signature_3dp"]


def test_hook_records_pre_call_acm_and_lossless_scene(tmp_path):
    from rclpy.serialization import deserialize_message

    os.environ["ST09_PORTAL_CAPTURE_DIR"] = str(tmp_path)
    goal = _Node([0.1, -1.0, 0.2, -0.3, 0.4, 0.5])
    MoveItEdgeAdapter._keep_portal_failure(
        _Host(), _Node([0.0] * 6), goal, -27, _scene(),
        _constraints(goal.q))
    record = json.loads((Path(tmp_path) / "portal_00.json").read_text())
    assert record["acm"]["entry_names"]  # host emptied it AFTER capture
    assert any(any(row) for row in record["acm"]["matrix"])
    scene = deserialize_message(
        (Path(tmp_path) / record["scene_bin"]).read_bytes(), PlanningScene)
    assert len(scene.allowed_collision_matrix.entry_names) > 0


def test_select_request_capture_round_trips_exactly(tmp_path):
    request = SelectPlacePathV2.Request()
    request.request_id = "req-1"
    entry = capture_service_request(request, str(tmp_path), "select")
    assert entry["request_id"] == "req-1"
    back = load_service_request(
        str(tmp_path / entry["bin"]), SelectPlacePathV2.Request)
    assert back == request
    assert capture_service_request(request, "", "select") is None


def _record(goal, validity, tol=1e-4):
    return {"goal_q": goal, "error_code": -27, "exact_sha256": "x",
            "goal_constraints": [
                {"joint_name": n, "position": v, "tolerance_above": tol,
                 "tolerance_below": tol} for n, v in zip(NAMES, goal)],
            "validity_on_submission_scene": validity}


def test_classifier_distinguishes_limit_collision_constraint_and_residual():
    limits = urdf_joint_limits(URDF)
    ok_q = [0.0, -1.0, 0.0, 0.0, 0.0, 0.0]
    collide = {"status": "queried", "valid": False, "reason": "COLLISION",
               "contacts": [{"body_1": "a", "body_2": "b"}]}
    valid = {"status": "queried", "valid": True, "reason": ""}
    assert classify_portal_record(_record(ok_q, collide), limits)[
        "category"] == "GOAL_IN_COLLISION"
    assert classify_portal_record(_record(ok_q, valid), limits)[
        "category"] == "CERTIFIER_VALID_MOVEIT_INVALID"
    assert classify_portal_record(
        _record([0.0, 0.5, 0.0, 0.0, 0.0, 0.0], collide), limits)[
        "category"] == "GOAL_OUTSIDE_URDF_LIMITS"
    assert classify_portal_record(_record(ok_q, collide, tol=0.0), limits)[
        "category"] == "GOAL_CONSTRAINT_DEFECT"
    assert classify_portal_record(
        _record(ok_q, {"status": "not_evaluated"}), limits)[
        "category"] == "NOT_EVALUATED"


def test_same_goal_under_changed_scene_is_captured_separately(tmp_path):
    from geometry_msgs.msg import Point
    from moveit_msgs.msg import CollisionObject
    from shape_msgs.msg import Mesh, MeshTriangle

    os.environ["ST09_PORTAL_CAPTURE_DIR"] = str(tmp_path)
    goal = _Node([0.1, -1.0, 0.2, -0.3, 0.4, 0.5])

    def scene_with(vertex_x):
        scene = _scene()
        obj = CollisionObject()
        obj.id = "wall"
        mesh = Mesh()
        mesh.vertices = [Point(x=vertex_x), Point(x=1.0), Point(y=1.0)]
        triangle = MeshTriangle()
        triangle.vertex_indices = [0, 1, 2]
        mesh.triangles = [triangle]
        obj.meshes = [mesh]
        scene.world.collision_objects = [obj]
        return scene

    host = _Host()
    for scene in (scene_with(0.0), scene_with(0.1), scene_with(0.0)):
        MoveItEdgeAdapter._keep_portal_failure(
            host, _Node([0.0] * 6), goal, -27, scene,
            _constraints(goal.q))
    assert len(list(Path(tmp_path).glob("portal_[0-9][0-9].json"))) == 2
    index = json.loads((Path(tmp_path) / "capture_index.json").read_text())
    assert sorted(e["occurrences"] for e in index["entries"]) == [1, 2]


def _rec(goal, validity=None):
    return {
        "goal_q": goal,
        "goal_constraints": [
            {"joint_name": n, "position": v, "tolerance_above": 1e-4,
             "tolerance_below": 1e-4} for n, v in zip(NAMES, goal)],
        "validity_on_submission_scene": validity or {
            "status": "queried", "valid": True},
        "exact_sha256": "x", "error_code": -27}


def test_missing_or_invalid_limit_is_not_evaluated_not_outside():
    from luggage_planning.portal_goal_classifier import (
        classify_portal_record, urdf_joint_limits)

    goal = [0.0] * 6
    for urdf in ("", "<robot><joint name='elfin_joint1'/></robot>",
                 "<robot><joint name='elfin_joint1'><limit lower='a' "
                 "upper='1'/></joint></robot>", "not xml <<"):
        limits = urdf_joint_limits(urdf)
        assert classify_portal_record(
            _rec(goal), limits)["category"] == "NOT_EVALUATED"


def test_xml_attribute_order_and_quote_variants_parse_alike():
    from luggage_planning.portal_goal_classifier import urdf_joint_limits

    variants = [
        '<robot><joint name="elfin_joint1" type="revolute">'
        '<limit lower="-1" upper="2"/></joint></robot>',
        "<robot><joint type='revolute' name='elfin_joint1'>"
        "<limit upper='2' effort='1' lower='-1'/></joint></robot>",
    ]
    assert [urdf_joint_limits(v) for v in variants] == [
        {"elfin_joint1": (-1.0, 2.0)}] * 2


def test_real_over_limit_goal_is_classified_outside():
    from luggage_planning.portal_goal_classifier import (
        classify_portal_record, urdf_joint_limits)

    goal = [0.0, 1.0, 0.0, 0.0, 0.0, 0.0]  # joint2 upper is 0.1745
    verdict = classify_portal_record(_rec(goal), urdf_joint_limits(
        "<robot>" + URDF + "</robot>"))
    assert verdict["category"] == "GOAL_OUTSIDE_URDF_LIMITS"


# --- joint-edge interpolation gate and knot densification -------------------

def test_densify_keeps_knots_and_bounds_the_step():
    from luggage_planning.moveit_edge_adapter import _Knot, densify_knots

    points = [_Knot([0.0] * 6), _Knot([0.3, 0, 0, 0, 0, -0.12])]
    dense = densify_knots(points, 0.05)
    assert dense[0].positions == [0.0] * 6
    assert dense[-1].positions == pytest.approx([0.3, 0, 0, 0, 0, -0.12])
    steps = [max(abs(a - b) for a, b in zip(p.positions, q.positions))
             for p, q in zip(dense, dense[1:])]
    assert max(steps) <= 0.05 + 1e-12
    assert len(dense) == 7  # 0.3 / 0.05 = 6 pieces


class _GateHost(object):
    def __init__(self, verdict):
        self.verdict = verdict

    def _portal_submission_scene(self):
        return object()

    def _states_on_scene(self, scene, states, edge_id):
        return self.verdict


def _timed():
    from st09_contracts.graph import TimedTrajectory

    return TimedTrajectory(
        joint_names=tuple(NAMES),
        positions=((0.0,) * 6, (0.1,) * 6),
        velocities=((0.0,) * 6, (0.0,) * 6),
        accelerations=((0.0,) * 6, (0.0,) * 6),
        time_from_start_ns=(0, 1_000_000_000))


def _refuse(verdict):
    import types
    from luggage_planning.moveit_edge_adapter import EdgeClass

    return MoveItEdgeAdapter._refuse_interpolated_collision(
        _GateHost(verdict), types.SimpleNamespace(node_id="a"),
        types.SimpleNamespace(node_id="b"), EdgeClass.PORTAL_TRANSIT,
        _timed())


def test_interpolated_collision_refuses_edge_but_checker_failure_does_not():
    from luggage_planning.grid_path_search import RoutePlanningError

    with pytest.raises(RoutePlanningError) as caught:
        _refuse({"status": "queried", "valid": False, "reason": "COLLISION",
                 "first_invalid_sample": 5, "contacts": []})
    assert caught.value.detail["kind"] == "interpolated_collision"
    assert caught.value.detail["first_invalid_sample"] == 5
    _refuse({"status": "queried", "valid": True})
    _refuse({"status": "not_evaluated", "reason": "timeout"})
