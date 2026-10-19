from copy import deepcopy
from luggage_msgs.srv import SelectPlacePathV2, PreparePlacementPriorV2
from luggage_msgs.msg import PlacementPriorChoiceV2
from luggage_planning.place_service import pose_message
from luggage_planning.placement_prior_adoption import order_select_candidates
from test_placement_prior_requests import slot_record


def inputs():
    request=SelectPlacePathV2.Request();request.geometry_hash='hull';request.map_revision=3
    request.box_wdh=[.4,.3,.2];request.grasp=pose_message(dict(slot_record(),reference='suction_contact_frame',orientation_semantics='box'))
    request.slots=[pose_message(dict(slot_record(),position=[float(i),2.,.5])) for i in range(3)]
    request.slot_scores=[3.,2.,1.];request.start_joints=[1.,2.,3.,4.,5.,6.]
    prior=PreparePlacementPriorV2.Response();prior.success=True;prior.measured=True
    prior.geometry_hash='hull';prior.map_revision=3;prior.box_wdh=[.4,.3,.2]
    prior.surface=deepcopy(request.surface)
    choice=PlacementPriorChoiceV2();choice.grasp=deepcopy(request.grasp);choice.slot=deepcopy(request.slots[2])
    prior.choices=[choice]
    return request,prior


def test_only_orders_candidates_and_preserves_actual_planning_inputs():
    request,prior=inputs();before=deepcopy(request)
    order=order_select_candidates(request,prior)
    assert order==(2,0,1)
    assert request.slots==[before.slots[2],before.slots[0],before.slots[1]]
    assert list(request.slot_scores)==[1.,3.,2.]
    assert request.grasp==before.grasp and request.start_joints==before.start_joints
    assert request.surface==before.surface


def test_changed_attachment_or_snapshot_falls_back_without_dropping_candidates():
    request,prior=inputs();before=deepcopy(request)
    request.grasp.pose.position.x+=.001
    assert order_select_candidates(request,prior)==(0,1,2)
    assert request.slots==before.slots
    request,prior=inputs();prior.map_revision=2
    assert order_select_candidates(request,prior)==(0,1,2)
    request,prior=inputs();prior.measured=False
    assert order_select_candidates(request,prior)==(0,1,2)


def test_real_typed_adapter_uses_bound_prior_and_calls_full_selector_once():
    from luggage_planning.typed_driver_adapter import TypedDriverAdapter
    from test_placement_prior_requests import slot_record
    request,prior=inputs()
    adapter=object.__new__(TypedDriverAdapter)
    adapter._select=object();calls=[]
    def call(client,wire,timeout):
        calls.append(deepcopy(wire))
        response=SelectPlacePathV2.Response();response.success=True;response.slot_index=0
        return response
    adapter._call=call;adapter.set_placement_prior(prior)
    # Mutating the caller's hint after binding must not change the adopted hint.
    prior.choices.clear()
    grasp=dict(slot_record(),reference='suction_contact_frame',orientation_semantics='box')
    response=adapter.select_place_path(request_id='prior',box_wdh=request.box_wdh,
        slots=request.slots,slot_scores=request.slot_scores,surface_message=request.surface,
        inner_size=[3.,3.,3.],geometry_hash=request.geometry_hash,map_revision=request.map_revision,
        start_tcp=dict(slot_record(),frame_id='world',reference='suction_contact_frame',orientation_semantics='tool_quaternion'),
        start_joints=[1.,2.,3.,4.,5.,6.],grasp=grasp,
        container_transform=('container_link','world',[0.,0.,0.],[0.,0.,0.,1.]),
        box_id='box',generation=2)
    assert len(calls)==1
    assert calls[0].slots[0]==request.slots[2]
    assert list(calls[0].start_joints)==[1.,2.,3.,4.,5.,6.]
    assert calls[0].generation==2
    assert response.slot_index==2  # mapped back into the original caller list
    adapter.set_placement_prior(None)
    assert adapter._placement_prior is None
