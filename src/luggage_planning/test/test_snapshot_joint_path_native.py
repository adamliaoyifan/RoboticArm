"""Real MoveIt/OMPL service on an isolated six-joint collision fixture.

No Gazebo, controllers, cameras or physical robot. Set the executable and a new
evidence directory to run this explicit native gate; default pure suites skip.
"""
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time
from types import SimpleNamespace

import pytest

EXECUTABLE = os.environ.get("JOINT_NATIVE_CONNECTOR_EXECUTABLE", "")
pytestmark = pytest.mark.skipif(not EXECUTABLE, reason="explicit native connector gate")


def robot_fixture():
    links = '<link name="world"/><link name="base"/>'
    joints = '<joint name="world_base" type="fixed"><parent link="world"/><child link="base"/></joint>'
    parent = "base"
    for i in range(1, 7):
        child = "arm%d" % i
        links += '<link name="%s"/>' % child
        joints += ('<joint name="elfin_joint%d" type="revolute"><parent link="%s"/>'
                   '<child link="%s"/><origin xyz="0.2 0 0"/><axis xyz="0 0 1"/>'
                   '<limit lower="-2.5" upper="2.5" effort="100" velocity="1.57"/></joint>'
                   % (i, parent, child))
        parent = child
    links += ('<link name="suction_contact_frame"><collision><geometry>'
              '<sphere radius="0.03"/></geometry></collision></link>')
    joints += ('<joint name="tool" type="fixed"><parent link="arm6"/>'
               '<child link="suction_contact_frame"/></joint>')
    xml = '<robot name="connection_fixture">'+links+joints+'</robot>'
    semantic = ('<robot name="connection_fixture"><group name="elfin_arm">'
                '<chain base_link="base" tip_link="suction_contact_frame"/>'
                '</group></robot>')
    return xml, semantic


@pytest.fixture(scope="module")
def backend():
    import rclpy
    import yaml
    from luggage_msgs.srv import PlanSnapshotJointPathV2
    from rclpy.serialization import serialize_message
    from moveit_msgs.msg import PlanningScene
    from luggage_planning.joint_operation_contracts import OPERATION_JOINTS

    directory = Path(os.environ["JOINT_NATIVE_EVIDENCE_DIR"])
    directory.mkdir(parents=True, exist_ok=False)
    xml, semantic = robot_fixture()
    params = {"snapshot_joint_path": {"ros__parameters": {
        "robot_description": xml, "robot_description_semantic": semantic,
        "joint_check_step_rad": .01, "max_checked_states": 50000,
        "ompl.planner_configs.RRTConnectkConfigDefault.type": "geometric::RRTConnect",
        "ompl.elfin_arm.planner_configs": ["RRTConnectkConfigDefault"],
    }}}
    parameter_file = directory / "parameters.yaml"
    parameter_file.write_text(yaml.safe_dump(params))
    env = dict(os.environ, ROS_DOMAIN_ID="173", ROS_LOCALHOST_ONLY="1")
    log = (directory / "node.log").open("wb")
    process = subprocess.Popen([EXECUTABLE, "--ros-args", "--params-file", str(parameter_file)],
                               env=env, stdout=log, stderr=subprocess.STDOUT)
    # rclpy context uses the same isolated domain; it has no global default node.
    context = rclpy.context.Context()
    old_localhost = os.environ.get("ROS_LOCALHOST_ONLY")
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    rclpy.init(context=context, domain_id=173)
    node = rclpy.create_node("native_connection_gate", context=context)
    client = node.create_client(PlanSnapshotJointPathV2, "/joint_operation/plan_connection")
    executor = rclpy.executors.SingleThreadedExecutor(context=context)
    executor.add_node(node)
    if old_localhost is None:
        os.environ.pop("ROS_LOCALHOST_ONLY", None)
    else:
        os.environ["ROS_LOCALHOST_ONLY"] = old_localhost
    calls = []
    manifest = {"scope": "native planner fixture; no deployment or whole-chain qualification",
                "model_sha256": hashlib.sha256((xml+semantic).encode()).hexdigest(),
                "ros_domain_id": 173, "command": [EXECUTABLE, "--ros-args", "--params-file", str(parameter_file)],
                "calls": calls, "capture_complete": False, "replay_possible": True}

    def request():
        req = PlanSnapshotJointPathV2.Request()
        req.request_id, req.snapshot_id = "native-%d" % len(calls), "fixture-snapshot"
        req.model_sha256 = manifest["model_sha256"]
        scene = PlanningScene()
        scene.name = "fixture"
        scene.robot_model_name = "connection_fixture"
        scene.robot_state.joint_state.name = list(OPERATION_JOINTS)
        scene.robot_state.joint_state.position = [0.]*6
        set_scene(req, scene)
        req.start_joints = [0.]*6
        req.goal_joints = [.4]+[0.]*5
        req.budget_s = 2.
        return req

    def set_scene(req, scene):
        raw = serialize_message(scene)
        req.scene_cdr = list(raw)
        req.scene_sha256 = hashlib.sha256(raw).hexdigest()

    def call(req):
        prefix = "%02d" % len(calls)
        (directory / (prefix+"_request.cdr")).write_bytes(serialize_message(req))
        entry = {"request_id": req.request_id, "request": prefix+"_request.cdr"}
        calls.append(entry)
        future = client.call_async(req)
        executor.spin_until_future_complete(future, timeout_sec=req.budget_s+5.)
        assert future.done(), "native query timeout; see preserved node.log and request"
        response = future.result()
        (directory / (prefix+"_response.cdr")).write_bytes(serialize_message(response))
        entry.update(response=prefix+"_response.cdr", success=response.success,
                     complete=response.complete, reason=response.reason,
                     checked_states=response.checked_states, elapsed_s=response.elapsed_s,
                     diagnostics=json.loads(response.diagnostics_json))
        return response

    try:
        ready = client.wait_for_service(timeout_sec=10.)
        assert ready and process.poll() is None, "native node not ready; see node.log"
        yield SimpleNamespace(request=request, call=call, set_scene=set_scene,
                              node=node, executor=executor, directory=directory,
                              model_hash=manifest["model_sha256"])
    finally:
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5.)
            except subprocess.TimeoutExpired:
                process.terminate()
                process.wait(timeout=5.)
        log.close()
        manifest.update(teardown="native child stopped", exit_code=process.returncode,
                        capture_complete=all("response" in c for c in calls))
        (directory / "manifest.json").write_text(json.dumps(manifest, indent=2)+"\n")


def test_free_scene_uses_native_planner_and_checks_preserved_endpoints(backend):
    from luggage_planning.joint_operation_connection import validate_connection_response
    req = backend.request()
    response = backend.call(req)
    assert response.success and response.complete, response.reason
    result = validate_connection_response(response, request_id=req.request_id,
        snapshot_id=req.snapshot_id, model_sha256=req.model_sha256,
        scene_sha256=req.scene_sha256, start=tuple(req.start_joints), goal=tuple(req.goal_joints))
    assert result.success and len(result.positions) >= 40
    assert result.positions[0] == tuple(req.start_joints)
    assert result.positions[-1] == tuple(req.goal_joints)
    assert all(not p.velocities and not p.time_from_start.sec and not p.time_from_start.nanosec
               for p in response.geometric_path.points)


@pytest.mark.parametrize('allowed',[True,False])
def test_native_position_path_constraint_is_enforced_during_connection(backend,allowed):
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import PositionConstraint
    from shape_msgs.msg import SolidPrimitive
    req=backend.request()
    constraint=PositionConstraint()
    constraint.header.frame_id='world'
    constraint.link_name='suction_contact_frame'
    constraint.weight=1.
    pose=Pose();pose.orientation.w=1.
    pose.position.x=1.1 if allowed else -5.
    constraint.constraint_region.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,
        dimensions=[1.,1.5,.2])]
    constraint.constraint_region.primitive_poses=[pose]
    req.path_constraints.position_constraints=[constraint]
    response=backend.call(req)
    if allowed:
        assert response.success and response.complete,response.reason
    else:
        assert not response.success and response.reason=='PATH_CONSTRAINT'


def test_disconnected_constraint_regions_do_not_return_a_path_between_valid_endpoints(backend):
    import numpy as np
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import PositionConstraint
    from shape_msgs.msg import SolidPrimitive
    from luggage_planning.offline_ik import Urdf
    req=backend.request()
    model=Urdf(robot_fixture()[0]);path=model.path_to('suction_contact_frame')
    constraint=PositionConstraint()
    constraint.header.frame_id='world';constraint.link_name='suction_contact_frame';constraint.weight=1.
    for q in (req.start_joints,req.goal_joints):
        xyz=model.tcp(q,path)[:3,3]
        pose=Pose();pose.orientation.w=1.
        pose.position.x,pose.position.y,pose.position.z=map(float,xyz)
        constraint.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX,
            dimensions=[.05,.05,.05]))
        constraint.constraint_region.primitive_poses.append(pose)
    req.path_constraints.position_constraints=[constraint]
    response=backend.call(req)
    assert not response.success and not response.geometric_path.points
    # Both endpoints were accepted; bounded planning exhausted the disconnected
    # region. This is not an unreachability proof for arbitrary task constraints.
    assert response.reason in ('NO_CONNECTION','DEADLINE')
    assert response.checked_states==2


def test_start_collision_refuses_then_fresh_private_scene_still_plans(backend):
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import CollisionObject, PlanningScene
    from shape_msgs.msg import SolidPrimitive
    from rclpy.serialization import deserialize_message
    req = backend.request()
    obstacle = CollisionObject()
    obstacle.id, obstacle.header.frame_id = "start-blocker", "world"
    primitive = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[.1, .1, .1])
    position = Pose()
    position.position.x, position.orientation.w = 1.2, 1.
    obstacle.primitives, obstacle.primitive_poses = [primitive], [position]
    obstacle.operation = CollisionObject.ADD
    scene = deserialize_message(bytes(req.scene_cdr), PlanningScene)
    scene.world.collision_objects = [obstacle]
    backend.set_scene(req, scene)
    before = bytes(req.scene_cdr)
    response = backend.call(req)
    assert response.complete and not response.success
    assert response.reason == "STATE_COLLISION" and not response.geometric_path.points
    assert bytes(req.scene_cdr) == before
    assert backend.call(backend.request()).success  # blocker did not leak to next request


def test_model_identity_and_joint_limits_refuse_without_path(backend):
    req = backend.request()
    req.model_sha256 = "wrong"
    response = backend.call(req)
    assert not response.success and not response.complete and not response.geometric_path.points
    assert response.model_sha256 == backend.model_hash
    req = backend.request()
    req.goal_joints = [3.]+[0.]*5
    response = backend.call(req)
    assert response.reason == "JOINT_BOUNDS" and response.complete and not response.success
    assert not response.geometric_path.points


def test_original_scene_bytes_and_deadline_are_checked(backend):
    req = backend.request()
    req.scene_sha256 = "wrong"
    response = backend.call(req)
    assert not response.complete and not response.success and not response.geometric_path.points
    req = backend.request()
    req.budget_s = 1e-9
    response = backend.call(req)
    assert response.reason == "DEADLINE" and not response.complete and not response.success
    assert not response.geometric_path.points


def test_scene_diffs_refused_and_joint6_winding_preserved(backend):
    from moveit_msgs.msg import PlanningScene
    from rclpy.serialization import deserialize_message
    req = backend.request()
    scene = deserialize_message(bytes(req.scene_cdr), PlanningScene)
    scene.is_diff = True
    backend.set_scene(req, scene)
    response = backend.call(req)
    assert not response.success and not response.complete and not response.geometric_path.points
    req = backend.request()
    req.start_joints = [0.]*5+[2.4]
    req.goal_joints = [0.]*5+[-2.4]
    response = backend.call(req)
    assert response.success and response.complete, response.reason
    path = response.geometric_path.points
    assert path[0].positions[-1] == 2.4 and path[-1].positions[-1] == -2.4
    assert all(max(abs(a-b) for a, b in zip(left.positions, right.positions)) <= .01+1e-10
               for left, right in zip(path, path[1:]))


def test_attached_payload_in_snapshot_participates_in_native_collision_check(backend):
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
    from shape_msgs.msg import SolidPrimitive
    from rclpy.serialization import deserialize_message
    req = backend.request()
    scene = deserialize_message(bytes(req.scene_cdr), PlanningScene)
    attachment = AttachedCollisionObject()
    attachment.link_name = "suction_contact_frame"
    attachment.touch_links = ["suction_contact_frame"]
    attachment.object.id, attachment.object.header.frame_id = "predicted-box", "suction_contact_frame"
    attachment.object.operation = CollisionObject.ADD
    offset = Pose()
    offset.orientation.w, offset.position.z = 1., .2
    attachment.object.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[.1]*3)]
    attachment.object.primitive_poses = [offset]
    scene.robot_state.attached_collision_objects = [attachment]
    backend.set_scene(req, scene)
    assert backend.call(req).success  # attachment itself is feasible
    blocker = CollisionObject()
    blocker.id, blocker.header.frame_id, blocker.operation = "payload-only-blocker", "world", CollisionObject.ADD
    world_pose = Pose()
    world_pose.orientation.w, world_pose.position.x, world_pose.position.z = 1., 1.2, .2
    blocker.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[.1]*3)]
    blocker.primitive_poses = [world_pose]
    scene.world.collision_objects = [blocker]
    backend.set_scene(req, scene)
    response = backend.call(req)
    assert response.reason == "STATE_COLLISION" and not response.success
    assert "predicted-box" in response.diagnostics_json
    scene.robot_state.attached_collision_objects = []
    backend.set_scene(req, scene)
    assert backend.call(req).success  # bare arm alone does not collide


def test_production_python_port_consumes_native_connection(backend):
    import threading
    from luggage_planning.joint_operation_connection import NativeSnapshotConnectionPort
    req = backend.request()
    snapshot = SimpleNamespace(identity=("fixture", 1), model_hash=backend.model_hash,
                               scene_cdr=bytes(req.scene_cdr))
    records = []
    def record(kind, message):
        from rclpy.serialization import serialize_message
        name = "port_%s.cdr" % kind
        (backend.directory / name).write_bytes(serialize_message(message))
        records.append(kind)
    port = NativeSnapshotConnectionPort(backend.node, recorder=record)
    assert port.client.wait_for_service(timeout_sec=2.)
    stopped = threading.Event()
    def spin():
        while not stopped.is_set():
            backend.executor.spin_once(timeout_sec=.01)
    thread = threading.Thread(target=spin)
    thread.start()
    try:
        result = port.connect(snapshot, (0.,)*6, (.4,)+(0.,)*5,
                              request_id="port", deadline=time.monotonic()+3.,
                              cancelled=lambda: False)
        assert result.success and result.complete
        assert result.positions[0] == (0.,)*6
        assert result.positions[-1] == (.4,)+(0.,)*5
        assert records == ["request", "response"]
    finally:
        stopped.set()
        thread.join(timeout=2.)
        backend.node.destroy_client(port.client)
    assert not thread.is_alive()
