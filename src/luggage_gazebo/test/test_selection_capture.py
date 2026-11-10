import json
from pathlib import Path

from rclpy.serialization import deserialize_message
from luggage_msgs.srv import SelectPlacePathV2
from luggage_gazebo.selection_capture import capture_selection


def test_selection_boundary_retains_original_response(tmp_path):
    request = dict(request_id="case-1", box_wdh=[.51, .33, .22],
                   grasp=dict(position=[0., 0., .11], quaternion_xyzw=[0., 1., 0., 0.]))
    response = SelectPlacePathV2.Response()
    response.success = True
    response.reason_code = "OK"
    directory = Path(capture_selection(tmp_path, request, response))
    restored = deserialize_message((directory/"response.cdr").read_bytes(),
                                   SelectPlacePathV2.Response)
    assert restored.success and restored.reason_code == "OK"
    assert json.loads((directory/"request.json").read_text()) == request


def test_capture_disabled_without_root():
    assert capture_selection(None, None) is None


def test_failed_select_requires_scene_but_not_nonexistent_trajectory(tmp_path):
    from luggage_gazebo.selection_capture import preserve_failed_selection
    directory = Path(capture_selection(tmp_path, dict(request_id='fail'),
        SelectPlacePathV2.Response(success=False, reason_code='PLANNING_BUDGET_EXHAUSTED')))
    artifacts, missing = preserve_failed_selection(directory, tmp_path/'dump')
    assert missing == ['selection/planning_scene.cdr']
    from moveit_msgs.srv import GetPlanningScene
    from rclpy.serialization import serialize_message
    scene = GetPlanningScene.Response()
    (directory/'planning_scene.cdr').write_bytes(serialize_message(scene))
    artifacts, missing = preserve_failed_selection(directory, tmp_path/'dump')
    assert not missing and len(artifacts) == 3
    restored = deserialize_message((tmp_path/'dump/selection/planning_scene.cdr').read_bytes(),
                                   GetPlanningScene.Response)
    assert restored == scene
