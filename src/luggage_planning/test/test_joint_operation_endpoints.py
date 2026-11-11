"""Current-envelope endpoint inputs and bound untimed branch admission."""
from dataclasses import replace
import time
from types import SimpleNamespace as NS

import pytest
from luggage_msgs.msg import GraspLocalBranch
from luggage_msgs.srv import CheckGraspLocalPaths
from moveit_msgs.msg import PlanningScene
from rclpy.serialization import deserialize_message,serialize_message
from trajectory_msgs.msg import JointTrajectoryPoint

from luggage_planning.joint_operation_contracts import OPERATION_JOINTS
from luggage_planning.joint_operation_endpoints import (
    build_local_query, endpoint_evidence, NativeOperationEndpointPort,
)
from luggage_planning.joint_operation_planner import JointSeedHint
from luggage_planning.placement_prior_contracts import Transform
from test_joint_operation_planner import fixture


def transform(pose):
    return Transform((pose.position.x,pose.position.y,pose.position.z),
        (pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w))


def query(candidate):
    return build_local_query(candidate,service=CheckGraspLocalPaths,request_id='request',budget_s=1.)


def response(request):
    result=CheckGraspLocalPaths.Response()
    result.request_id,result.snapshot_id,result.model_sha256=request.request_id,request.snapshot_id,request.model_sha256
    result.complete,result.reason=True,'LOCAL_PATHS_VALID'
    def branch(seed):
        value=GraspLocalBranch()
        value.seed_index=seed
        for name,rows in [('incoming',[(.01,)*6,(.02,)*6]),('outgoing',[(.02,)*6,(.03,)*6])]:
            path=getattr(value,name)
            path.joint_names=list(OPERATION_JOINTS)
            path.points=[JointTrajectoryPoint(positions=list(q)) for q in rows]
        return value
    result.pick_branches=[branch(0),branch(1)]
    result.place_branches=[branch(0)]
    return result


def test_private_request_removes_exact_target_only_and_never_double_inflates():
    from st09_contracts.attachment_geometry import freeze_geometry
    from st09_contracts.payload_envelope import EnvelopeLimits
    from luggage_planning.joint_operation_scenes import build_operation_scenes
    from test_joint_operation_scenes import baseline
    _,candidate=fixture()
    old=candidate.predicted_attachment
    grown=freeze_geometry(old.measured_dimensions_wdh,old.t_tool_box_translation,
        old.t_tool_box_quaternion_xyzw,EnvelopeLimits(5.,(.1,.1,.1),(0.,0.),.005))
    scenes=build_operation_scenes(baseline(),pickup_object_id='pickup_box',geometry=grown,
        world_from_pick_box=candidate.world_from_pick_box,world_from_settled_box=candidate.world_from_settled_box)
    candidate=replace(candidate,predicted_attachment=grown,scenes=scenes)
    before=candidate.scenes.pick.scene_cdr
    request=query(candidate)
    assert candidate.scenes.pick.scene_cdr==before
    objects={obj.id for obj in request.scene.world.collision_objects}
    assert objects=={'pickup_box_neighbor','container','robot_pedestal'}
    assert tuple(request.box_wdh)==grown.conservative.dimensions_wdh
    assert tuple(request.box_wdh)!=grown.measured_dimensions_wdh
    for box,tool in [(request.pick_box_world,candidate.world_from_pick_tool),
                     (request.place_box_world,candidate.world_from_release_tool)]:
        actual=transform(box).compose(transform(request.box_from_tcp))
        assert actual.translation==pytest.approx(tool.translation,abs=1e-10)
        assert abs(sum(a*b for a,b in zip(actual.quaternion_xyzw,tool.quaternion_xyzw)))==pytest.approx(1.)
    matrix=request.scene.allowed_collision_matrix
    index=matrix.entry_names.index('prior-local-box')
    assert {name for name,allowed in zip(matrix.entry_names,matrix.entry_values[index].enabled) if allowed}=={'suction_panel','suction_contact_frame'}
    assert not matrix.default_entry_values[matrix.default_entry_names.index('prior-local-box')]


def test_seed_hints_preserve_zero_values_order_and_branch_provenance():
    _,candidate=fixture()
    candidate=replace(candidate,pick_entry_seeds=(JointSeedHint('atlas:zero',(0.,)*6),
        JointSeedHint('atlas:other',(.1,)*6)),place_entry_seeds=(JointSeedHint('atlas:place',(.2,)*6),))
    request=query(candidate)
    assert tuple(request.pick_entry_seed_joints_flat)==(0.,)*6+(.1,)*6
    assert tuple(request.place_entry_seed_joints_flat)==(.2,)*6
    request=deserialize_message(serialize_message(request),CheckGraspLocalPaths.Request)
    result=endpoint_evidence(response(request),request,candidate)
    assert result.complete and len(result.pick_branches)==2
    assert result.pick_branches[0].branch_id=='pick:hint:atlas:zero'
    assert result.pick_branches[1].branch_id=='pick:hint:atlas:other'
    assert result.place_branches[0].branch_id=='place:hint:atlas:place'


@pytest.mark.parametrize('field',['request_id','snapshot_id','model_sha256'])
def test_foreign_endpoint_receipt_refuses(field):
    _,candidate=fixture();request=query(candidate);result=response(request)
    setattr(result,field,'other')
    evidence=endpoint_evidence(result,request,candidate)
    assert not evidence.complete and not evidence.pick_branches


@pytest.mark.parametrize('kind',['timed','nan','step','names','disconnect','empty','duplicate','seed_index'])
def test_malformed_untimed_local_witness_never_admitted(kind):
    _,candidate=fixture();request=query(candidate);result=response(request)
    branch=result.pick_branches[0]
    if kind=='timed': branch.incoming.points[1].time_from_start.nanosec=1
    elif kind=='nan': branch.incoming.points[0].positions=[float('nan')]*6
    elif kind=='step': branch.incoming.points[0].positions=[1.]*6
    elif kind=='names': branch.outgoing.joint_names=list(reversed(OPERATION_JOINTS))
    elif kind=='disconnect': branch.outgoing.points[0].positions=[.03]*6
    elif kind=='empty': result.place_branches=[]
    elif kind=='duplicate': result.pick_branches[1].seed_index=0
    else: branch.seed_index=8
    evidence=endpoint_evidence(result,request,candidate)
    assert not evidence.complete and not evidence.pick_branches and not evidence.place_branches


def test_hint_changes_identity_but_is_never_treated_as_a_trajectory():
    _,candidate=fixture()
    plain=query(candidate)
    hints=(JointSeedHint('generic-0',(0.,)*6),)
    hinted=replace(candidate,pick_entry_seeds=hints)
    request=query(hinted)
    assert plain.snapshot_id!=request.snapshot_id
    result=endpoint_evidence(response(request),request,hinted)
    assert [b.branch_id for b in result.pick_branches]==['pick:hint:generic-0','pick:generic-0']
    assert result.pick_branches[0].incoming[0]!=(0.,)*6


def test_busy_and_cancelled_ports_never_submit_another_request():
    _,candidate=fixture()
    port=object.__new__(NativeOperationEndpointPort)
    port.inflight=NS(done=lambda:False)
    assert port.solve(candidate,request_id='req',deadline=time.monotonic()+1.,cancelled=lambda:False).reason=='LOCAL_PATH_BACKEND_BUSY'
    assert port.solve(candidate,request_id='req',deadline=time.monotonic()+1.,cancelled=lambda:True).reason=='CANCELLED_OR_DEADLINE'


def test_reserved_local_payload_identity_does_not_remove_an_obstacle():
    from dataclasses import replace
    _,candidate=fixture()
    scene=deserialize_message(candidate.scenes.pick.scene_cdr,PlanningScene)
    scene.world.collision_objects[0].id='prior-local-box'
    phases=replace(candidate.scenes,pick=replace(candidate.scenes.pick,scene_cdr=serialize_message(scene)))
    with pytest.raises(ValueError,match='conflicts'):
        query(replace(candidate,scenes=phases))
