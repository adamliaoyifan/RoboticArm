import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from luggage_msgs.srv import VacuumCommandV2
from luggage_planning.attachment import sim_physical_attachment as physical
from test_attachment_revision_propagation import _request, LIVE


def test_fresh_pose_parser_refuses_absence_nonfinite_and_wrong_model():
    good = '\nModel: [4]\n  - Name: box\n  - Pose [ XYZ (m) ] [ RPY (rad) ]:\n    [1 2 3]\n    [0 0 0]\n'
    def run(text):
        return lambda *a, **k: SimpleNamespace(returncode=0, stdout=text)
    assert physical.observe_model_pose('box', run=run(good)) == ([1.,2.,3.], [0.,0.,0.,1.])
    for text in [good.replace('Name: box','Name: other'), good.replace('1 2 3','nan 2 3'),
                 'No model named <box> was found']:
        with pytest.raises(ValueError):
            physical.observe_model_pose('box', run=run(text))


def test_epoch_change_during_observation_refuses_relation():
    epochs = iter([('box',2),('box',3)])
    with pytest.raises(ValueError, match='changed'):
        physical.freeze_physical_relation('box',2,{},read_identity=lambda:next(epochs),
                                          observe=lambda _:([1,2,3],[0,0,0,1]))


@pytest.fixture
def controller(monkeypatch):
    path = Path(__file__).parents[1]/'scripts/vacuum_controller_node.py'
    spec = importlib.util.spec_from_file_location('physical_vacuum_node_test',path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module.tf2_ros,'TransformListener',lambda *a,**k:None)
    rclpy.init()
    node = module.VacuumControllerNode()
    node._identity = ('pickup_box_0001_carryon',2)
    node._frame_runtime = SimpleNamespace(identity={'scene_revision':LIVE},
        admit_request=lambda *a,**k:None, authorize_effect=lambda *a,**k:None)
    transform = TransformStamped()
    transform.header.stamp.nanosec = 2000
    transform.transform.translation.x = -1.
    transform.transform.translation.z = 1.107
    transform.transform.rotation.x = 1.
    transform.transform.rotation.w = 0.
    node._tf_buffer = SimpleNamespace(lookup_transform=lambda *a:transform)
    node._publish_state = lambda:None
    node._publish_event = lambda *a:None
    scene_calls, follow_calls = [], []
    node._backend._scene = SimpleNamespace(attach_pickup_box=lambda *a:(scene_calls.append(a) or True,''))
    node._backend._carry_filter = lambda _: (True,'')
    node._backend._gz = SimpleNamespace(set_model_pose=lambda *a:(follow_calls.append(a) or True,''))
    try:
        yield node, scene_calls, follow_calls
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_real_node_v2_keeps_mesh_axes_and_measured_planning_separate(controller,monkeypatch):
    node, scene, follow = controller
    # Measured +X is long axis, physical model +Y is long axis.
    original_xyz = [-1.007,.004,.985]
    physical_q = [0.,0.,math.sin(math.pi/4),math.cos(math.pi/4)]
    monkeypatch.setattr(physical,'observe_model_pose',lambda _: (original_xyz,physical_q))
    request = _request(LIVE)
    result = node._handle_command_v2(request,VacuumCommandV2.Response())
    assert result.success, result.message
    assert list(result.attached_box_wdh) == [.516,.390,.247]
    from luggage_planning.attachment.attachment_geometry_transport import geometry_from_message
    geometry = geometry_from_message(result.conservative_geometry)
    expected_position, expected_quaternion = geometry.conservative_pose_at_box(
        (-1.,0.,.9835), (0.,0.,0.,1.))
    assert scene[0][1] == pytest.approx(expected_position)
    assert scene[0][2] == pytest.approx(expected_quaternion)
    assert scene[0][3] == pytest.approx(geometry.conservative.dimensions_wdh)
    # Contact face stays fixed while the conservative bottom moves down.
    assert scene[0][1][2] + scene[0][3][2]/2 == pytest.approx(1.107)
    assert scene[0][1][2] - scene[0][3][2]/2 < .9835 - .247/2
    assert node._backend.follow_step([-1.,0.,1.107],[1.,0.,0.,0.])[0]
    assert follow[-1][1] == pytest.approx(original_xyz)
    assert follow[-1][2] == pytest.approx(physical_q)
    planning = node._backend.planning_box_pose([-1.,0.,1.107],[1.,0.,0.,0.])
    assert planning[0] == pytest.approx([-1.,0.,.9835])
    assert planning[1] == pytest.approx([0.,0.,0.,1.])
    # Move TCP: preserve the physical attachment, do not resample GT per tick.
    monkeypatch.setattr(physical,'observe_model_pose',lambda _: (_ for _ in ()).throw(AssertionError('resampled')))
    assert node._backend.follow_step([0.,0.,1.107],[1.,0.,0.,0.])[0]
    assert follow[-1][1] == pytest.approx([-.007,.004,.985])


def test_unavailable_physics_stops_before_scene_attach(controller,monkeypatch):
    node, scene, follow = controller
    monkeypatch.setattr(physical,'observe_model_pose',lambda _: (_ for _ in ()).throw(ValueError('absent')))
    response = node._handle_command_v2(_request(LIVE),VacuumCommandV2.Response())
    assert not response.success
    assert 'PHYSICAL_ATTACHMENT_UNAVAILABLE' in response.message
    assert not response.has_measured_attachment
    assert not scene and not follow and not node._backend.is_attached()


def test_observed_task_id_resolves_exact_actuator_instance(controller, monkeypatch):
    from luggage_msgs.msg import SuctionCandidate
    node, scene, follow = controller
    observed = []
    monkeypatch.setattr(physical, 'observe_model_pose',
                        lambda model: (observed.append(model) or [-1.,0.,.9835], [0.,0.,0.,1.]))
    request = _request(LIVE)
    request.box_id = request.measured_box.id = 'detected_box'
    request.measured_box.suction_candidates = [SuctionCandidate(
        instance_id='pickup_box_0001_carryon', generation=2)]
    result = node._handle_command_v2(request, VacuumCommandV2.Response())
    assert result.success and result.has_measured_attachment, result.message
    assert observed == ['pickup_box_0001_carryon']
    assert node._backend.last_attach_record['box_id'] == 'detected_box'
    assert node._backend.follow_step([-1.,0.,1.107],[1.,0.,0.,0.])[0]
    assert follow[-1][0] == 'pickup_box_0001_carryon'
    assert list(result.attached_box_wdh) == [.516,.390,.247]


@pytest.mark.parametrize('damage', ['wrong_epoch', 'ambiguous', 'unregistered'])
def test_observed_instance_refuses_before_effects(controller, monkeypatch, damage):
    from luggage_msgs.msg import SuctionCandidate
    node, scene, follow = controller
    monkeypatch.setattr(physical, 'observe_model_pose',
                        lambda _: ([-1.,0.,.9835], [0.,0.,0.,1.]))
    request = _request(LIVE)
    request.box_id = request.measured_box.id = 'detected_box'
    request.measured_box.suction_candidates = [SuctionCandidate(
        instance_id='pickup_box_0001_carryon', generation=2)]
    if damage == 'wrong_epoch': request.measured_box.suction_candidates[0].generation = 3
    elif damage == 'ambiguous': request.measured_box.suction_candidates.append(
        SuctionCandidate(instance_id='other', generation=2))
    else: request.measured_box.suction_candidates[0].instance_id = 'unregistered'
    result = node._handle_command_v2(request, VacuumCommandV2.Response())
    assert not result.success and not result.has_measured_attachment
    assert not result.diagnostic.accepted
    assert not scene and not follow and not node._backend.is_attached()
