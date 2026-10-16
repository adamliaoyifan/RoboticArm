"""Private scene reuse changes decoding cost, never scene contents or identity."""
import time
from types import SimpleNamespace
from unittest.mock import patch

from moveit_msgs.msg import PlanningScene
from rclpy.serialization import serialize_message, deserialize_message
from luggage_msgs.srv import CheckPlacementToolV2
from luggage_planning.placement_prior_collision_port import NativeToolCollisionPort
from luggage_planning.placement_prior_contracts import Transform


def test_reuses_exact_bytes_and_invalidates_changed_scene_with_same_labels():
    sent = []

    def call(request):
        sent.append(request)
        response = CheckPlacementToolV2.Response()
        response.request_id = request.request_id
        response.snapshot_id = request.snapshot_id
        response.model_sha256 = request.model_sha256
        response.complete = True
        response.valid = [True]
        return SimpleNamespace(done=lambda:True, result=lambda:response,
                               add_done_callback=lambda callback:callback(None))

    client = SimpleNamespace(service_is_ready=lambda:True, call_async=call)
    port = NativeToolCollisionPort(SimpleNamespace(create_client=lambda *args:client))
    scene = PlanningScene()
    scene.name = 'first'
    snapshot = SimpleNamespace(identity=('unchanged-label',), model_hash='model',
                               scene_cdr=serialize_message(scene))
    pose = Transform((0.,0.,0.), (0.,0.,0.,1.))
    def check():
        return port.check(snapshot,pose,pose,(0.5,0.3,0.2),
                          deadline=time.monotonic()+5,cancelled=lambda:False)

    with patch('rclpy.serialization.deserialize_message', wraps=deserialize_message) as decode:
        assert check().valid and check().valid
        assert decode.call_count == 1
        assert sent[0].scene == scene and sent[1].scene == scene
        scene.name = 'changed'
        snapshot.scene_cdr = serialize_message(scene)
        assert check().valid
        assert decode.call_count == 2
        assert sent[2].scene == scene
        assert sent[0].scene.name == 'first'
        assert sent[0].snapshot_id == sent[2].snapshot_id
