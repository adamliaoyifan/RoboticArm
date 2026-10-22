"""Controller attachment -> hardware planning -> real typed Select encoding."""
from copy import deepcopy
from types import SimpleNamespace

import pytest
from luggage_msgs.msg import PlacementPriorChoiceV2, SurfaceContractV2
from luggage_msgs.srv import VacuumCommandV2, PreparePlacementPriorV2, SelectPlacePathV2
from luggage_planning.attachment_frame_adapters import handle_vacuum_command_v2
from luggage_planning.hardware_place_planning import attachment_from_response, plan_attached_box
from luggage_planning.place_service import pose_message
from luggage_planning.typed_driver_adapter import TypedDriverAdapter, slot_record, tcp_record
from test_attachment_revision_propagation import _request, _tool, LIVE


def attachment():
    response = handle_vacuum_command_v2(_request(LIVE), VacuumCommandV2.Response(),
        scene_revision=LIVE, generation=2,
        resolve_tool=lambda stamp: _tool(LIVE, position=[-.995,0.,1.107]),
        backend=SimpleNamespace(attach=lambda payload: (True,'ok')))
    return attachment_from_response(response, request_id='attach',
        box_id='pickup_box_0001_carryon', generation=2, scene_revision=LIVE)


def inputs():
    measured = attachment()
    surface = SurfaceContractV2()
    surface.identity.scene_revision = LIVE
    surface.geometry_hash = 'hull'
    surface.map_revision = 9
    prior = PreparePlacementPriorV2.Response()
    prior.success = prior.measured = True
    prior.map_revision = 9
    prior.geometry_hash = 'hull'
    prior.surface = deepcopy(surface)
    prior.box_wdh = list(measured.box_wdh)
    choice = PlacementPriorChoiceV2()
    choice.slot = pose_message(slot_record('prior', LIVE, [.3,.2,.5], 0.))
    choice.grasp.pose.position.x = .02  # predicted offset differs from actual
    prior.choices = [choice]
    return measured, dict(request_id='place', surface=surface, inner_size=[1.,1.,1.],
        start_tcp=tcp_record('place',LIVE,[.1,.2,.9],[1.,0.,0.,0.]),
        start_joints=[.1]*6, container_transform=('container_link','world',[1.,2.,3.],[0.,0.,0.,1.]),
        prior=prior, selected_choice=choice)


def test_full_select_uses_controller_offset_and_keeps_target_fixed():
    measured, kwargs = inputs()
    calls = []
    adapter = object.__new__(TypedDriverAdapter)
    adapter.compute_placement = lambda **values: SimpleNamespace(
        success=True, candidates=[deepcopy(kwargs['selected_choice'].slot)])
    adapter._select = object()
    adapter._call = lambda client, request, timeout: (
        calls.append(deepcopy(request)) or SelectPlacePathV2.Response(success=True,slot_index=0))
    target_before = deepcopy(kwargs['selected_choice'].slot.pose)
    response = plan_attached_box(adapter, measured, **kwargs)
    assert response.success and len(calls) == 1
    sent = calls[0]
    assert sent.slots[0].pose == target_before
    assert sent.grasp.pose.position.x == pytest.approx(.005)
    assert sent.grasp.pose.position.z == pytest.approx(.1235)
    assert sent.grasp.identity.stamp_ns == 2000
    assert sent.grasp.identity.request_id == sent.slots[0].identity.request_id == 'place'
    assert sent.box_id == measured.box_id and sent.generation == 2
    assert list(sent.start_joints) == [.1]*6


@pytest.mark.parametrize('change', ['map','dimensions','missing_joints','scene'])
def test_inconsistent_inputs_stop_before_any_select(change):
    measured, kwargs = inputs()
    if change == 'map': kwargs['surface'].map_revision += 1
    if change == 'dimensions': kwargs['prior'].box_wdh = [.1,.1,.1]
    if change == 'missing_joints': kwargs['start_joints'] = []
    if change == 'scene': kwargs['surface'].identity.scene_revision = 'other'
    adapter = SimpleNamespace(select_place_path=lambda **values: pytest.fail('must not select'))
    with pytest.raises(ValueError):
        plan_attached_box(adapter, measured, **kwargs)


def test_no_prior_calls_ordinary_placement_then_full_select():
    measured, kwargs = inputs()
    kwargs.update(prior=None,selected_choice=None)
    calls=[]
    slot=pose_message(slot_record('generated',LIVE,[.1,.2,.3],0.))
    adapter=SimpleNamespace(
        compute_placement=lambda **values: (calls.append(('compute',values)) or
            SimpleNamespace(success=True,candidates=[slot],candidate_scores=[1.])),
        select_place_path=lambda **values: calls.append(('select',values)))
    plan_attached_box(adapter,measured,**kwargs)
    assert [kind for kind, _ in calls] == ['compute','select']
    assert calls[-1][1]['grasp']['request_id'] == 'generated'
    assert calls[-1][1]['grasp']['position'][0] == pytest.approx(.005)


def test_bound_nominal_target_cannot_bypass_conservative_capacity():
    measured, kwargs = inputs()
    seen = []
    adapter = SimpleNamespace(
        compute_placement=lambda **values: (seen.append(values) or
            SimpleNamespace(success=False)),
        select_place_path=lambda **values: pytest.fail('must refuse before Select'))
    with pytest.raises(ValueError, match='BOUND_TARGET_NOT_IN_CONSERVATIVE_CANDIDATES'):
        plan_attached_box(adapter, measured, **kwargs)
    assert seen[0]['conservative_geometry'] is measured.geometry
