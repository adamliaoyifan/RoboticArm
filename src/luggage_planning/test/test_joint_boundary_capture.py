import json
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import PlanningScene
from rclpy.serialization import deserialize_message
from luggage_planning.joint_boundary_capture import capture_boundary


def test_boundary_roundtrip_and_limit(tmp_path):
    scene = PlanningScene()
    scene.name = 'exact-scene'
    goal = MoveGroup.Goal()
    goal.request.start_state.joint_state.position = [0.12345678901234567]
    result = MoveGroup.Result()
    result.error_code.val = 1
    assert capture_boundary(str(tmp_path), 0, scene, goal, result)
    folder = tmp_path / 'carrying_joint_00'
    recovered = deserialize_message((folder / 'goal.cdr').read_bytes(), MoveGroup.Goal)
    assert recovered == goal
    assert deserialize_message((folder / 'scene.cdr').read_bytes(), PlanningScene) == scene
    assert deserialize_message((folder / 'result.cdr').read_bytes(), MoveGroup.Result) == result
    assert json.loads((folder / 'manifest.json').read_text())['capture_complete']
    assert not capture_boundary(str(tmp_path), 4, scene, goal, result)
    assert len(list(tmp_path.iterdir())) == 1


def test_missing_scene_is_incomplete_evidence(tmp_path):
    capture_boundary(str(tmp_path), 0, None, MoveGroup.Goal(), None)
    manifest = json.loads((tmp_path / 'carrying_joint_00/manifest.json').read_text())
    assert not manifest['capture_complete']
