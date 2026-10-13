"""Production collision check rejects an occupancy column, and only that.

The same joint state and attached payload are submitted to
``/validate_route_v2`` twice. With the column in the frozen scene the
leg is ``COLLISION``. With the column removed the leg is valid. The
column is the only world object, so the refusal is not the container
or ``placed_0_0_0``.
"""

from __future__ import division

import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
HERE = ROOT / "scripts" / "st09_i"
SOURCE = ROOT / "src" / "luggage_planning"
for path in (str(HERE), str(SOURCE), str(ROOT)):
    if path not in sys.path:
        sys.path.insert(0, path)

import corridor_lattice_score as score  # noqa: E402
from luggage_planning.moveit_state_validity import (  # noqa: E402
    SUCTION_INTERFACE_TOUCH_LINKS,
)
from luggage_planning.offline_ik import JOINTS, Urdf  # noqa: E402
from luggage_planning.planning_scene_client import (  # noqa: E402
    build_attach_scene,
    build_collision_object,
)

SCENE_TF = str(
    ROOT / "src" / "luggage_description" / "config" / "scene_tf_deploy.yaml")
# A carried start that the second-box request already planned from.
START_Q = (
    -1.3909481677798137,
    -1.7365991346176266,
    -1.6476149388915682,
    -1.3271240705895746,
    1.576986724025758,
    -1.402174174444041,
)
DOMAIN = 44
GZ_LAUNCH = {
    "use_gz_sim": "true",
    "hardware_plugin": "gz_ros2_control/GazeboSimSystem",
    "xacro_extra": (
        "initial_joint1:=-1.250000,initial_joint2:=-1.746600,"
        "initial_joint3:=-0.997400,initial_joint4:=-1.915300,"
        "initial_joint5:=1.771400,initial_joint6:=1.861000"),
}


def _robot_xml(node):
    from rcl_interfaces.srv import GetParameters
    import threading

    client = node.create_client(
        GetParameters, "/robot_state_publisher/get_parameters")
    if not client.wait_for_service(timeout_sec=20.0):
        raise RuntimeError("robot_description unavailable")
    request = GetParameters.Request()
    request.names = ["robot_description"]
    done = threading.Event()
    future = client.call_async(request)
    future.add_done_callback(lambda _f: done.set())
    if not done.wait(15.0):
        raise RuntimeError("robot_description timeout")
    result = future.result()
    return str(result.values[0].string_value)


def _acm(node):
    from moveit_msgs.msg import PlanningSceneComponents
    from luggage_planning.planning_scene_client import PlanningSceneClient

    client = PlanningSceneClient(node)
    if not client.wait_ready(timeout_sec=30.0):
        raise RuntimeError("get_planning_scene unavailable")
    scene = client.get_scene(
        PlanningSceneComponents.ALLOWED_COLLISION_MATRIX, timeout=8.0)
    if scene is None or not scene.allowed_collision_matrix.entry_names:
        raise RuntimeError("production ACM is empty")
    return scene.allowed_collision_matrix


def _scene(acm, tcp_xyz, with_column):
    from moveit_msgs.msg import CollisionObject, PlanningScene

    scene = PlanningScene()
    scene.name = "occupancy-negative"
    scene.is_diff = False
    scene.allowed_collision_matrix = acm
    attach = build_attach_scene(
        "cert-payload", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0),
        (0.04, 0.04, 0.04), "suction_contact_frame",
        touch_links=SUCTION_INTERFACE_TOUCH_LINKS)
    scene.robot_state = attach.robot_state
    scene.robot_state.is_diff = False
    scene.robot_state.joint_state.name = list(JOINTS)
    scene.robot_state.joint_state.position = [float(v) for v in START_Q]
    if with_column:
        scene.world.collision_objects = [build_collision_object(
            "snapshot-cell-0-0", list(tcp_xyz), (0.0, 0.0, 0.0, 1.0),
            (0.08, 0.08, 0.08), "world", CollisionObject.ADD)]
    return scene


def _request(scene):
    from luggage_msgs.srv import ValidateRouteV2
    from luggage_msgs.msg import RouteLegV2
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    request = ValidateRouteV2.Request()
    request.request_id = "occupancy-negative"
    request.route_digest = "occupancy-negative"
    request.scene_digest = "occupancy-negative"
    request.load_state_scenes = [scene]
    request.max_sample_dt_ns = 0.0
    request.max_sample_step_m = 0.0
    request.distance_stride = 1
    request.budget_s = 2.0
    request.joint_velocity_limits = [0.0] * 6
    leg = RouteLegV2()
    leg.edge_id = "carry-sample"
    leg.edge_class = "TRAVERSE"
    leg.carrying = True
    leg.load_state_index = 0
    leg.joint_names = list(JOINTS)
    leg.n_samples = 1
    leg.sample_times_ns = [0.0]
    leg.sample_positions_flat = [float(v) for v in START_Q]
    knots = JointTrajectory()
    knots.joint_names = list(JOINTS)
    point = JointTrajectoryPoint()
    point.positions = [float(v) for v in START_Q]
    point.time_from_start.sec = 0
    point.time_from_start.nanosec = 0
    knots.points = [point]
    leg.knots = knots
    request.legs = [leg]
    return request


def _call(node, scene):
    from luggage_msgs.srv import ValidateRouteV2
    import threading

    client = node.create_client(ValidateRouteV2, "/validate_route_v2")
    if not client.wait_for_service(timeout_sec=5.0):
        raise RuntimeError("validate_route_v2 unavailable")
    done = threading.Event()
    future = client.call_async(_request(scene))
    future.add_done_callback(lambda _f: done.set())
    if not done.wait(15.0):
        raise RuntimeError("validate_route_v2 timeout")
    response = future.result()
    leg = response.leg_results[0] if response.leg_results else None
    return {
        "result": int(response.result),
        "detail": str(response.detail),
        "valid": bool(leg.valid) if leg is not None else False,
        "reason": str(leg.reason) if leg is not None else "",
        "leg_detail": str(leg.detail) if leg is not None else "",
        "contacts": [
            (str(item.body_1), str(item.body_2))
            for item in (leg.contacts if leg is not None else [])
        ],
    }


def test_occupancy_column_rejects_and_clears():
    import rclpy
    from rclpy.node import Node
    from luggage_perception.eval.isolated_domain import isolated_replay_env

    residuals = score._residuals()  # noqa: SLF001
    assert not residuals, residuals
    env = isolated_replay_env(DOMAIN)
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    out = Path("/tmp/occupancy_collision_negative")
    out.mkdir(parents=True, exist_ok=True)
    proc = score._start_moveit(  # noqa: SLF001
        env, SCENE_TF, str(out / "moveit.log"), GZ_LAUNCH)
    rclpy.init()
    node = Node("occupancy_collision_negative")
    stop, thread = score._spin(node)  # noqa: SLF001
    try:
        from luggage_msgs.srv import ValidateRouteV2

        client = node.create_client(ValidateRouteV2, "/validate_route_v2")
        assert client.wait_for_service(timeout_sec=90.0)
        xml = _robot_xml(node)
        urdf = Urdf(xml)
        pose = urdf.tcp(START_Q, urdf.path_to("suction_contact_frame"))
        tcp = [float(v) for v in pose[:3, 3]]
        acm = _acm(node)
        blocked = _call(node, _scene(acm, tcp, True))
        clear = _call(node, _scene(acm, tcp, False))
    finally:
        stop.set()
        thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        score._stop_moveit(proc)  # noqa: SLF001
    assert blocked["reason"] == "COLLISION", blocked
    bodies = {name for pair in blocked["contacts"] for name in pair}
    assert "snapshot-cell-0-0" in bodies, blocked
    assert clear["valid"] is True, clear
    assert clear["reason"] == "", clear
