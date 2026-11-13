"""Real KDL/FCL/OMPL opening passage on an isolated full-pose fixture.

Not a Gazebo, S20, sensor or deployment baseline. Native boundary CDR is retained.
"""
import json
import math
import os
import time
from dataclasses import replace

import pytest
from rclpy.serialization import deserialize_message,serialize_message
from moveit_msgs.msg import PlanningScene

from test_joint_operation_endpoints_native import native_stack,candidate_for
from luggage_planning.joint_operation.aperture_geometry import ApertureGeometry
from luggage_planning.joint_operation.aperture import ApertureRouter
from luggage_planning.joint_operation.ik import NativeSnapshotIKPort
from luggage_planning.placement_prior.contracts import Transform

pytestmark=pytest.mark.skipif(not os.environ.get('JOINT_NATIVE_ENDPOINT_EXECUTABLE'),
                             reason='explicit native opening routing fixture')


def passage(stack):
    start=(-.25,-.6,1.2,0.,-.6,math.pi)
    goal=(.25,-.6,1.2,0.,-.6,math.pi)
    identity,planning_start,base=candidate_for(stack.model,pick_q=start,place_q=goal)
    # A measured-source-independent geometric fixture, facing +Y, with no
    # production coordinates. Four obstacles bound a finite opening.
    from luggage_planning.joint_operation.scenes import _box
    scene=deserialize_message(base.scenes.carry.scene_cdr,PlanningScene)
    for name,centre,size in (
        ('left',(-.35,0.,.85),(1.3,.04,.7)),
        ('right',(1.525,0.,.85),(.95,.04,.7)),
        ('bottom',(.675,0.,0.),(.75,.04,1.)),
        ('top',(.675,0.,1.6),(.75,.04,.8))):
        scene.world.collision_objects.append(_box(name,size,Transform(centre,(0.,0.,0.,1.)),'world'))
    raw=serialize_message(scene)
    import hashlib
    carrying=replace(base.scenes.carry,scene_cdr=raw,scene_content_hash=hashlib.sha256(raw).hexdigest())
    candidate=replace(base,scenes=replace(base.scenes,carry=carrying))
    opening=ApertureGeometry(((.3,0.,.5),(1.05,0.,.5),(1.05,0.,1.2),(.3,0.,1.2)),(0.,1.,0.))
    ik=NativeSnapshotIKPort(stack.node,stack.model,
        recorder=lambda kind,value:stack.record('opening_ik',kind,serialize_message(value)))
    assert ik.client.wait_for_service(timeout_sec=5.)
    router=ApertureRouter(opening,stack.model,ik,stack.connections,
        max_points=3,max_seeds=2)
    return identity,planning_start,candidate,router,start,goal


def test_native_grasp_bound_ordered_passage_has_collision_checked_plane_witness(native_stack):
    stack=native_stack
    _,_,candidate,router,start,goal=passage(stack)
    trace=[]
    began=time.monotonic()
    result=router.connect(candidate,start,goal,request_id='native-opening',
        deadline=began+15.,cancelled=lambda:False,trace=trace)
    output=dict(scope=__doc__,success=result.success,reason=result.reason,
        elapsed_s=time.monotonic()-began,trace=trace,checked_states=result.checked_states)
    (stack.directory/'opening_result.json').write_text(json.dumps(output,indent=2)+'\n')
    assert result.success and result.complete,(result.reason,trace)
    assert result.positions[0]==start and result.positions[-1]==goal
    selected=next(row for row in trace if row[0]=='aperture_selected')
    targets=selected[2]
    assert len(targets)==3
    assert targets[1]['position_world'][1]==pytest.approx(0.,abs=1e-12)
    for item in targets:
        assert tuple(item['joints']) in result.positions
        assert stack.model.matches(item['joints'],Transform(item['position_world'],item['quaternion_xyzw']))


def test_native_complete_six_leg_plan_certifies_the_ordered_opening(native_stack):
    from luggage_planning.joint_operation.planner import JointOperationPlanner
    stack=native_stack
    identity,start,candidate,router,_,_=passage(stack)
    planner=JointOperationPlanner(stack.endpoints,stack.connections,stack.certifier,
        velocity_scale=.2,acceleration_scale=.1,certification_reserve_s=2.5,
        aperture_router=router)
    previous=stack.certifier.aperture_router
    stack.certifier.aperture_router=router
    try:
        result=planner.prepare(identity,start,(candidate,),budget_s=15.)
    finally:
        stack.certifier.aperture_router=previous
    output=dict(status=result.status,elapsed_s=result.elapsed_s,trace=result.trace)
    (stack.directory/'opening_whole_plan.json').write_text(json.dumps(output,indent=2)+'\n')
    assert result.status=='PLAN_READY',result.trace
    assert len(result.plan.legs)==6 and len(result.plan.barriers)==2
    assert router.has_plane_witness(result.plan,candidate)
    assert any(row[0]=='aperture_selected' for row in result.trace)


def test_native_current_obstacle_refuses_all_proposed_plane_states(native_stack):
    import hashlib
    from luggage_planning.joint_operation.scenes import _box
    stack=native_stack
    _,_,candidate,router,start,goal=passage(stack)
    scene=deserialize_message(candidate.scenes.carry.scene_cdr,PlanningScene)
    scene.world.collision_objects.append(_box('blocked_opening',(1.5,.1,2.),
        Transform((.675,0.,.85),(0.,0.,0.,1.)),'world'))
    raw=serialize_message(scene)
    candidate=replace(candidate,scenes=replace(candidate.scenes,carry=replace(
        candidate.scenes.carry,scene_cdr=raw,scene_content_hash=hashlib.sha256(raw).hexdigest())))
    router.max_points=1;router.max_seeds=1
    trace=[]
    result=router.connect(candidate,start,goal,request_id='blocked-opening',
        deadline=time.monotonic()+5.,cancelled=lambda:False,trace=trace)
    (stack.directory/'blocked_opening_result.json').write_text(json.dumps(dict(
        success=result.success,reason=result.reason,trace=trace),indent=2)+'\n')
    assert not result.success and not result.positions
    assert result.reason=='APERTURE_SEARCH_INCOMPLETE'
    assert not any(row[0]=='aperture_selected' for row in trace)


def test_native_ik_chooses_seed_nearest_legal_full_turn_equivalent(native_stack):
    stack=native_stack
    target_q=(-.25,-.6,1.2,0.,-.6,3.3)
    seed=target_q[:-1]+(3.25,)
    _,_,candidate=candidate_for(stack.model,pick_q=target_q,place_q=target_q)
    target=Transform.from_matrix(stack.model.urdf.tcp(target_q,stack.model.path))
    port=NativeSnapshotIKPort(stack.node,stack.model,recorder=lambda kind,value:
        stack.record('nearest_winding',kind,serialize_message(value)))
    assert port.client.wait_for_service(timeout_sec=5.)
    result=port.refine(candidate.scenes.carry,target,seed,request_id='nearest-winding',
                       deadline=time.monotonic()+2.,cancelled=lambda:False)
    assert result.complete and result.success,result.reason
    assert result.joints[-1]>math.pi
    assert max(abs(a-b) for a,b in zip(result.joints,seed))<.1
    assert stack.model.matches(result.joints,target)
    port.client.destroy()
