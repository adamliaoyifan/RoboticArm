import json
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import PlanningScene
from rclpy.serialization import deserialize_message
from luggage_planning.capture.joint_boundary_capture import capture_boundary


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


def test_failed_insert_roundtrip_includes_exact_waypoints_and_fraction(tmp_path):
    from geometry_msgs.msg import Pose
    from moveit_msgs.srv import GetCartesianPath
    from luggage_planning.capture.joint_boundary_capture import capture_insert_boundary
    scene = PlanningScene()
    request = GetCartesianPath.Request()
    request.start_state.joint_state.position = [0.12345678901234567] * 6
    pose = Pose()
    pose.position.z = 0.91372123456789
    pose.orientation.w = 1.0
    request.waypoints = [pose]
    response = GetCartesianPath.Response()
    response.fraction = 5.0 / 9.0
    assert capture_insert_boundary(str(tmp_path), 0, scene, request, response)
    folder = tmp_path / 'carrying_insert_00'
    assert deserialize_message((folder / 'request.cdr').read_bytes(), GetCartesianPath.Request) == request
    assert deserialize_message((folder / 'response.cdr').read_bytes(), GetCartesianPath.Response) == response
    assert not capture_insert_boundary(str(tmp_path), 4, scene, request, response)
    assert len(list(tmp_path.iterdir())) == 1
