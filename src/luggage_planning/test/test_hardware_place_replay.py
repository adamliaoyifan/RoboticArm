"""Captured attachment and planning context must agree before ROS dispatch."""
from copy import deepcopy
from types import SimpleNamespace
import pytest
from luggage_msgs.srv import VacuumCommandV2, SelectPlacePathV2
from luggage_planning.attachment_frame_adapters import handle_vacuum_command_v2
from luggage_planning.hardware_place_planning import plan_attached_box
from luggage_planning.hardware_place_replay import replay_place
from luggage_planning.typed_driver_adapter import TypedDriverAdapter
from test_hardware_place_planning import inputs
from test_attachment_revision_propagation import _request, _tool, LIVE


def capture():
    attachment,kwargs=inputs()
    calls=[]
    adapter=object.__new__(TypedDriverAdapter)
    adapter.compute_placement=lambda **values: SimpleNamespace(
        success=True,candidates=[deepcopy(kwargs['selected_choice'].slot)])
    adapter._select=object()
    adapter._call=lambda client,request,timeout: calls.append(deepcopy(request))
    plan_attached_box(adapter,attachment,**kwargs)
    response=handle_vacuum_command_v2(_request(LIVE),VacuumCommandV2.Response(),
        scene_revision=LIVE,generation=2,
        resolve_tool=lambda stamp:_tool(LIVE,position=[-.995,0.,1.107]),
        backend=SimpleNamespace(attach=lambda payload:(True,'ok')))
    return calls[0],response,kwargs['prior']


def test_replay_uses_actual_attachment_and_preserves_bound_target():
    request,response,prior=capture()
    calls=[]
    adapter=object.__new__(TypedDriverAdapter);adapter._select=object()
    adapter.compute_placement=lambda **values: SimpleNamespace(
        success=True,candidates=[deepcopy(prior.choices[0].slot)])
    adapter._call=lambda client,req,timeout:(calls.append(deepcopy(req)) or
        SelectPlacePathV2.Response(success=True))
    result=replay_place(adapter,request,response,attachment_request=_request(LIVE),scene_revision=LIVE,
                        prior=prior,choice_index=0)
    assert result.success
    assert calls[0]==request


@pytest.mark.parametrize('change',['scene','start_frame','attachment','stamp','size','missing_measurement','choice'])
def test_inconsistent_capture_rejected_before_dispatch(change):
    request,response,prior=capture()
    choice=0
    if change=='scene':request.surface.identity.scene_revision='other'
    if change=='start_frame':request.start_tcp.identity.frame_id='world'
    if change=='attachment':response.measured_grasp.pose.position.x+=.01
    if change=='stamp':response.measured_grasp.identity.stamp_ns+=1
    if change=='size':response.attached_box_wdh=[.1,.2,.3]
    if change=='missing_measurement':response.has_measured_attachment=False
    if change=='choice':choice=10
    adapter=SimpleNamespace(select_place_path=lambda **kw:pytest.fail('must not dispatch'))
    with pytest.raises(ValueError):
        replay_place(adapter,request,response,attachment_request=_request(LIVE),scene_revision=LIVE,prior=prior,choice_index=choice)


def test_attachment_response_cannot_be_rebound_to_another_request():
    request,response,prior=capture()
    attach_request=_request(LIVE)
    attach_request.request_id='other'
    with pytest.raises(ValueError,match='identity mismatch'):
        replay_place(None,request,response,attachment_request=attach_request,
                     scene_revision=LIVE,prior=prior,choice_index=0)
