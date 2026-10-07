"""PORTAL_TRANSIT submission capture keeps the pre-call scene."""

import json
import os
from pathlib import Path

from moveit_msgs.msg import AllowedCollisionMatrix, PlanningScene

from luggage_planning.moveit_edge_adapter import MoveItEdgeAdapter
from luggage_planning.moveit_state_validity import (
    DECLARED_STRUCTURE_PAIRS,
    FrozenSceneComposer,
)
from luggage_planning.planning_scene_client import (
    acm_pair_allowed,
    set_acm_pair,
)


class _Node(object):
    def __init__(self, name, q):
        self.node_id = name
        self.q = tuple(q)


class _Constraint(object):
    def __init__(self):
        self.joint_name = "elfin_joint1"
        self.position = 0.0
        self.tolerance_above = 1e-4
        self.tolerance_below = 1e-4


class _Constraints(object):
    joint_constraints = [_Constraint()]


class _Host(object):
    def _goal_on_submission_scene(self, scene_msg, goal_q):
        return {
            "status": "queried",
            "valid": False,
            "reason": "COLLISION",
            "detail": "recorded",
            "contacts": [],
            "scene_name": str(scene_msg.name),
        }


def test_capture_keeps_the_scene_passed_in(tmp_path):
    os.environ["ST09_PORTAL_CAPTURE_DIR"] = str(tmp_path)
    scene = PlanningScene()
    scene.name = "before-call"
    host = _Host()
    start = _Node("start", [0.0] * 6)
    goal = _Node("goal", [0.1, -1.0, 0.2, -0.3, 0.4, 0.5])
    MoveItEdgeAdapter._keep_portal_failure(
        host, start, goal, -27, scene, _Constraints())
    scene.name = "after-call"
    MoveItEdgeAdapter._keep_portal_failure(
        host, start, goal, -27, scene, _Constraints())
    other = _Node("goal-b", [0.2, -1.0, 0.2, -0.3, 0.4, 0.5])
    fresh = PlanningScene()
    fresh.name = "second-signature"
    MoveItEdgeAdapter._keep_portal_failure(
        host, start, other, -27, fresh, _Constraints())
    files = sorted(Path(tmp_path).glob("portal_*.json"))
    assert [path.name for path in files] == [
        "portal_00.json", "portal_01.json"]
    first = json.loads(files[0].read_text())
    assert first["when"] == "before_move_group_call"
    assert first["error_code"] == -27
    assert first["validity_on_submission_scene"]["scene_name"] == "before-call"
    assert first["goal_constraints"][0]["tolerance_above"] == 1e-4


def test_composed_acm_allows_only_the_bolted_base_pair():
    live = AllowedCollisionMatrix()
    set_acm_pair(live, "elfin_base", "elfin_link1", True)
    composed = FrozenSceneComposer._composed_acm(
        FrozenSceneComposer, live)
    assert acm_pair_allowed(composed, "elfin_base", "elfin_link1")
    first, second = DECLARED_STRUCTURE_PAIRS[0]
    assert acm_pair_allowed(composed, first, second)
    assert not acm_pair_allowed(composed, "elfin_link1", second)
    assert DECLARED_STRUCTURE_PAIRS == (("elfin_base", "robot_pedestal"),)
