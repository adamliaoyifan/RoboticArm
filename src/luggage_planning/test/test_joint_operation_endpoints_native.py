"""Real KDL endpoints + OMPL connections + complete native certification.

Isolated six-axis fixture, no Gazebo or execution. The fixture has full pose
kinematics, tool collision and frozen measured box geometry. It is not S20 or
deploy_update qualification. Every request/response and node log is preserved.
"""
from dataclasses import replace
import hashlib
import json
import os
from pathlib import Path
import signal
import shutil
import subprocess
import threading
import time
from types import SimpleNamespace

import pytest

EXECUTABLE = os.environ.get('JOINT_NATIVE_ENDPOINT_EXECUTABLE','')
pytestmark = pytest.mark.skipif(not EXECUTABLE, reason='explicit native joint endpoint gate')


def robot_fixture():
    links = '<link name="world"/><link name="base"/>'
    joints = '<joint name="world_base" type="fixed"><parent link="world"/><child link="base"/><origin xyz="0 0 0.4"/></joint>'
    origins = ('0 0 0.3','0 0 0.15','0.35 0 0','0.25 0 0','0.1 0 0','0.08 0 0')
    axes = ('0 0 1','0 1 0','0 1 0','1 0 0','0 1 0','1 0 0')
    parent = 'base'
    for i,(origin,axis) in enumerate(zip(origins,axes),1):
        child = 'arm%d'%i
        links += '<link name="%s"/>'%child
        joints += ('<joint name="elfin_joint%d" type="revolute"><parent link="%s"/>'
            '<child link="%s"/><origin xyz="%s"/><axis xyz="%s"/>'
            '<limit lower="-3.5" upper="3.5" effort="100" velocity="1.57"/></joint>'
            %(i,parent,child,origin,axis))
        parent = child
    links += '<link name="suction_contact_frame"><collision><geometry><sphere radius="0.03"/></geometry></collision></link>'
    joints += '<joint name="tool" type="fixed"><parent link="arm6"/><child link="suction_contact_frame"/><origin xyz="0.1 0 0"/></joint>'
    xml = '<robot name="endpoint_fixture">'+links+joints+'</robot>'
    srdf = '<robot name="endpoint_fixture"><group name="elfin_arm"><chain base_link="base" tip_link="suction_contact_frame"/></group></robot>'
    return xml,srdf


@pytest.fixture(scope='module')
def native_stack(request):
    import rclpy
    import yaml
    from luggage_planning.joint_operation.endpoints import NativeOperationEndpointPort
    from luggage_planning.joint_operation.connection import NativeSnapshotConnectionPort
    from luggage_planning.joint_operation.certification import EndpointModel,NativeOperationCertificationPort
    directory = Path(os.environ['JOINT_ENDPOINT_EVIDENCE_DIR'])
    directory.mkdir(parents=True,exist_ok=False)
    xml,srdf = getattr(request,'param',None) or robot_fixture()
    model_parameters = {'robot_description':xml,'robot_description_semantic':srdf}
    kinematics = {'elfin_arm':{'kinematics_solver':'kdl_kinematics_plugin/KDLKinematicsPlugin',
        'kinematics_solver_search_resolution':.005,'kinematics_solver_timeout':.05}}
    config = {'grasp_local_check':{'ros__parameters':dict(model_parameters,robot_description_kinematics=kinematics)},
        'snapshot_joint_path':{'ros__parameters':dict(model_parameters,**{
            'ompl.planner_configs.RRTConnectkConfigDefault.type':'geometric::RRTConnect',
            'ompl.elfin_arm.planner_configs':['RRTConnectkConfigDefault']})},
        'validate_route_v2':{'ros__parameters':model_parameters}}
    parameters = directory/'parameters.yaml'
    parameters.write_text(yaml.safe_dump(config))
    executables = (('local',EXECUTABLE),('connection',os.environ['JOINT_NATIVE_CONNECTOR_EXECUTABLE']),
                   ('certification',os.environ['JOINT_NATIVE_CERTIFIER_EXECUTABLE']))
    executable_hashes = {}
    for name, executable in executables:
        frozen = directory/(name+'_executable')
        shutil.copy2(executable,frozen)
        executable_hashes[name] = hashlib.sha256(frozen.read_bytes()).hexdigest()
    processes,logs,records = [],[],[]
    for name,executable in executables:
        log = (directory/(name+'.log')).open('wb')
        logs.append(log)
        processes.append(subprocess.Popen([executable,'--ros-args','--params-file',str(parameters)],
            env=dict(os.environ,ROS_DOMAIN_ID='175',ROS_LOCALHOST_ONLY='1'),stdout=log,stderr=subprocess.STDOUT))
    context = rclpy.context.Context()
    old_localhost = os.environ.get('ROS_LOCALHOST_ONLY')
    os.environ['ROS_LOCALHOST_ONLY']='1'
    rclpy.init(context=context,domain_id=175)
    node = rclpy.create_node('joint_endpoint_gate',context=context)
    if old_localhost is None: os.environ.pop('ROS_LOCALHOST_ONLY',None)
    else: os.environ['ROS_LOCALHOST_ONLY']=old_localhost
    executor = rclpy.executors.SingleThreadedExecutor(context=context)
    executor.add_node(node)
    worker = threading.Thread(target=executor.spin,daemon=True)
    worker.start()
    def record(port,kind,raw):
        name = '%02d_%s_%s.cdr'%(len(records),port,kind)
        (directory/name).write_bytes(raw)
        records.append({'port':port,'kind':kind,'file':name,'sha256':hashlib.sha256(raw).hexdigest()})
    from rclpy.serialization import serialize_message
    endpoints = NativeOperationEndpointPort(node,max_query_s=3.,
        recorder=lambda kind,raw:record('local',kind,raw))
    connections = NativeSnapshotConnectionPort(node,
        recorder=lambda kind,message:record('connection',kind,serialize_message(message)))
    model = EndpointModel(xml,srdf,position_tolerance_m=.002,orientation_tolerance_rad=.02)
    certifier = NativeOperationCertificationPort(node,model,velocity_limits=(.314,)*6,
        acceleration_limits=(.314,)*6,release_drop_m=.05,
        recorder=lambda kind,raw:record('certification',kind,raw))
    try:
        for port in (endpoints,connections,certifier):
            assert port.client.wait_for_service(timeout_sec=10.),'native port unavailable; see preserved node logs'
        assert all(p.poll() is None for p in processes)
        yield SimpleNamespace(endpoints=endpoints,connections=connections,certifier=certifier,
            model=model,robot_description=xml,robot_description_semantic=srdf,
            records=records,directory=directory,node=node,record=record)
    finally:
        executor.shutdown(timeout_sec=2.);worker.join(timeout=2.)
        node.destroy_node();context.shutdown()
        for process in processes:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try: process.wait(timeout=5.)
                except subprocess.TimeoutExpired: process.kill();process.wait(timeout=5.)
        for log in logs: log.close()
        (directory/'manifest.json').write_text(json.dumps({'model_sha256':model.model_sha256,
            'scope':'isolated full-pose KDL+OMPL+native certification; not deployment qualification',
            'commands':[(name,path) for name,path in executables],'records':records,
            'executable_sha256':executable_hashes,'worker_joined':not worker.is_alive(),
            'teardown_exit_codes':[p.returncode for p in processes],'capture_complete':True},indent=2))
        assert not worker.is_alive(), 'endpoint fixture executor did not stop'
        assert all(p.returncode==0 for p in processes), 'native endpoint backend teardown failed'


def candidate_for(model, *, pick_q=None, place_q=None):
    import math
    from scipy.spatial.transform import Rotation
    from moveit_msgs.msg import PlanningScene
    from rclpy.serialization import serialize_message
    from luggage_planning.joint_operation.contracts import OperationIdentity,OPERATION_JOINTS
    from luggage_planning.joint_operation.planner import OperationCandidate,JointSeedHint
    from luggage_planning.joint_operation.scenes import build_operation_scenes
    from luggage_planning.placement_prior.contracts import Snapshot,Transform
    from luggage_planning.planning_contract.attachment_geometry import freeze_geometry
    from luggage_planning.planning_contract.payload_envelope import EnvelopeLimits
    pick_q=pick_q or (0.,-.6,1.2,0.,-.6,math.pi)
    place_q=place_q or (0.,-.4,1.0,0.,-.6,math.pi)
    def pose(q):
        matrix=model.urdf.tcp(q,model.path)
        return Transform(tuple(matrix[:3,3]),tuple(Rotation.from_matrix(matrix[:3,:3]).as_quat()))
    def raised(p,height):
        return Transform((p.translation[0],p.translation[1],p.translation[2]+height),p.quaternion_xyzw)
    pick_tool,release_tool=pose(pick_q),pose(place_q)
    geometry=freeze_geometry((.2,.15,.1),(0.,0.,.05),(1.,0.,0.,0.),
        EnvelopeLimits(0.,(0.,0.,0.),(0.,0.),0.))
    grasp=Transform(geometry.t_tool_box_translation,geometry.t_tool_box_quaternion_xyzw)
    pick_box,release_box=pick_tool.compose(grasp),release_tool.compose(grasp)
    settled_box=raised(release_box,-.05)
    scene=PlanningScene();scene.robot_model_name='endpoint_fixture'
    start_q=(pick_q[0]-.03,)+pick_q[1:]
    scene.robot_state.joint_state.name=list(OPERATION_JOINTS)
    scene.robot_state.joint_state.position=list(start_q)
    snapshot=Snapshot('scene',7,'hull',model.model_sha256,Transform((0.,0.,0.),(0.,0.,0.,1.)),
        b'fixture-measured-observation',serialize_message(scene))
    scenes=build_operation_scenes(snapshot,pickup_object_id='pickup_box',geometry=geometry,
        world_from_pick_box=pick_box,world_from_settled_box=settled_box)
    identity=OperationIdentity('native-operation','scene',hashlib.sha256(repr(snapshot.identity).encode()).hexdigest(),
        'hull',model.model_sha256,7,'observed-box',12,15,'box',0,'contact')
    candidate=OperationCandidate('joint-native',pick_box,pick_tool,release_box,release_tool,settled_box,
        geometry,scenes,raised(pick_tool,.04),raised(pick_tool,.06),raised(release_tool,.08),raised(release_tool,.08),
        (JointSeedHint('out-of-bounds',(100.,)*6),JointSeedHint('pick-prior',pick_q)),
        (JointSeedHint('out-of-bounds',(100.,)*6),JointSeedHint('place-prior',place_q)))
    return identity,start_q,candidate


def test_real_native_ik_skips_bad_hint_and_retains_alternative_branch_provenance(native_stack):
    _,_,candidate=candidate_for(native_stack.model)
    result=native_stack.endpoints.solve(candidate,request_id='endpoints-only',
        deadline=time.monotonic()+6.,cancelled=lambda:False)
    assert result.complete,result.reason
    assert any(b.branch_id=='pick:hint:pick-prior' for b in result.pick_branches)
    assert any(b.branch_id=='place:hint:place-prior' for b in result.place_branches)
    assert all('out-of-bounds' not in b.branch_id for b in result.pick_branches+result.place_branches)


def test_complete_plan_uses_real_native_endpoints_connections_and_certificate(native_stack):
    from luggage_planning.joint_operation.planner import JointOperationPlanner
    from luggage_planning.joint_operation.contracts import admit_operation_plan
    identity,start,candidate=candidate_for(native_stack.model)
    planner=JointOperationPlanner(native_stack.endpoints,native_stack.connections,native_stack.certifier,
        velocity_scale=.2,acceleration_scale=.1,certification_reserve_s=2.5)
    result=planner.prepare(identity,start,(candidate,),budget_s=12.)
    assert result.status=='PLAN_READY',result.trace
    plan=result.plan
    admit_operation_plan(plan,identity,plan.plan_digest)
    assert len(plan.legs)==6
    assert plan.legs[0].knots[0].positions==start
    assert native_stack.model.matches(plan.legs[1].knots[-1].positions,candidate.world_from_pick_tool)
    assert native_stack.model.matches(plan.legs[4].knots[-1].positions,candidate.world_from_release_tool)
    (native_stack.directory/'plan.json').write_text(json.dumps({'plan_digest':plan.plan_digest,
        'certificate_id':plan.certificate_id,'trace':result.trace,'elapsed_s':result.elapsed_s,
        'start_joints':plan.start_joints,'knots_per_leg':[len(leg.knots) for leg in plan.legs]},indent=2))


def test_equivalent_whole_place_branch_passes_native_six_leg_certificate(native_stack):
    from dataclasses import asdict
    from luggage_planning.joint_operation.local_branch_search import LocalBranchSearch
    from luggage_planning.joint_operation.planner import JointOperationPlanner
    from luggage_planning.joint_operation.contracts import admit_operation_plan
    identity, start, candidate = candidate_for(native_stack.model,
        pick_q=(0., -.6, 1.2, 0., -.6, -3.3),
        place_q=(0., -.4, 1., 0., -.6, 3.3))
    receipts = []

    class RecordedEndpoints:
        def solve(self, *args, **kwargs):
            receipt = native_stack.endpoints.solve(*args, **kwargs)
            receipts.append(receipt)
            return receipt

    planner = JointOperationPlanner(RecordedEndpoints(), native_stack.connections, native_stack.certifier,
        velocity_scale=.2, acceleration_scale=.1, certification_reserve_s=2.5,
        branch_search=LocalBranchSearch(native_stack.model))
    result = planner.prepare(identity, start, (candidate,), budget_s=12.)
    evidence = dict(result_status=result.status, trace=result.trace,
        local_receipts=[asdict(receipt) for receipt in receipts],
        plan=asdict(result.plan) if result.plan else None)
    (native_stack.directory/'equivalent_branch_case.json').write_text(json.dumps(evidence, indent=2))
    assert result.status == 'PLAN_READY', result.trace
    assert any(row[0] == 'equivalent_local_branch' for row in result.trace)
    assert any(branch.incoming[0][-1] > 3. for branch in receipts[0].place_branches)
    plan = result.plan
    admit_operation_plan(plan, identity, plan.plan_digest)
    assert len(plan.legs) == 6
    assert plan.legs[4].knots[0].positions[-1] < -2.
    assert native_stack.model.matches(plan.legs[4].knots[-1].positions, candidate.world_from_release_tool)
    assert plan.world_from_pick_tool == candidate.world_from_pick_tool
    assert plan.world_from_place_box == candidate.world_from_release_box


def test_legacy_empty_hints_still_produce_native_local_paths(native_stack):
    _,_,candidate=candidate_for(native_stack.model)
    candidate=replace(candidate,pick_entry_seeds=(),place_entry_seeds=())
    result=native_stack.endpoints.solve(candidate,request_id='legacy-no-hints',
        deadline=time.monotonic()+6.,cancelled=lambda:False)
    assert result.complete,result.reason
    assert all(':generic-' in b.branch_id for b in result.pick_branches+result.place_branches)
    assert all(b.incoming[-1]==b.outgoing[0] for b in result.pick_branches+result.place_branches)


@pytest.mark.parametrize('hints',[[0.]*5,[0.]*24,[float('nan')]*6])
def test_native_rejects_malformed_entry_seed_arrays(native_stack,hints):
    from luggage_planning.joint_operation.endpoints import build_local_query
    from luggage_msgs.srv import CheckGraspLocalPaths
    from rclpy.serialization import serialize_message
    _,_,candidate=candidate_for(native_stack.model)
    query=build_local_query(candidate,service=CheckGraspLocalPaths,request_id='bad-hints',budget_s=1.)
    query.pick_entry_seed_joints_flat=hints
    native_stack.record('local','request',serialize_message(query))
    ready=threading.Event()
    future=native_stack.endpoints.client.call_async(query)
    future.add_done_callback(lambda _:ready.set())
    assert ready.wait(5.)
    response=future.result()
    native_stack.record('local','response',serialize_message(response))
    assert not response.complete and not response.pick_branches and not response.place_branches
    assert response.reason.startswith('INVALID_QUERY: at most three finite entry seed rows')


def test_current_world_obstacle_rejects_even_with_valid_hint(native_stack):
    from luggage_planning.joint_operation.endpoints import build_local_query,endpoint_evidence
    from luggage_msgs.srv import CheckGraspLocalPaths
    from moveit_msgs.msg import CollisionObject
    from shape_msgs.msg import SolidPrimitive
    from geometry_msgs.msg import Pose
    from rclpy.serialization import serialize_message
    _,_,candidate=candidate_for(native_stack.model)
    query=build_local_query(candidate,service=CheckGraspLocalPaths,request_id='current-obstacle',budget_s=2.)
    obj=CollisionObject()
    obj.id,obj.header.frame_id,obj.operation='current-pick-obstacle','world',CollisionObject.ADD
    pose=Pose()
    pose.position.x,pose.position.y,pose.position.z=candidate.pick_approach.translation
    pose.orientation.w=1.
    obj.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[.4]*3)]
    obj.primitive_poses=[pose]
    query.scene.world.collision_objects.append(obj)
    native_stack.record('local','request',serialize_message(query))
    ready=threading.Event()
    future=native_stack.endpoints.client.call_async(query)
    future.add_done_callback(lambda _:ready.set())
    assert ready.wait(5.)
    response=future.result()
    native_stack.record('local','response',serialize_message(response))
    result=endpoint_evidence(response,query,candidate)
    assert not result.complete and not result.pick_branches
    assert response.reason=='NO_PICK_LOCAL_BRANCH'
    assert json.loads(response.diagnostics_json)['collision_rejections']>0
