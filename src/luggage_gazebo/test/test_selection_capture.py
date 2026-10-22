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
