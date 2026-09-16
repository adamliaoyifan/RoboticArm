#!/usr/bin/env python3
"""Isolated-domain MoveIt plan-only worker. Do not run on ROS_DOMAIN_ID=7.

Launch: robot_state_publisher + move_group (no Gazebo, no execution).
Reads a JSON request (start joints + attach XYZ) and writes planned joints.
"""
from __future__ import division

import argparse
import json
import os
import signal
import subprocess
import sys
import time

from luggage_perception.eval.isolated_domain import (
    IsolatedDomainError,
    assert_isolated_domain,
    isolated_replay_env,
)

JOINTS = [
    "elfin_joint1", "elfin_joint2", "elfin_joint3",
    "elfin_joint4", "elfin_joint5", "elfin_joint6",
]
TOOL_DOWN = (1.0, 0.0, 0.0, 0.0)


def _write(path, payload):
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle)
        handle.write("\n")


def _stop_launch(proc):
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGINT)
    except (ProcessLookupError, PermissionError, OSError):
        try:
            proc.send_signal(signal.SIGINT)
        except (ProcessLookupError, OSError):
            pass
    try:
        proc.wait(timeout=8)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(proc.pid, signal.SIGTERM)
        except (ProcessLookupError, PermissionError, OSError):
            proc.kill()
        proc.wait(timeout=5)


def _start_launch(env, timeout_sec):
    cmd = [
        "ros2", "launch", "luggage_planning", "isolated_moveit.launch.py",
        "use_rviz:=false",
    ]
    proc = subprocess.Popen(
        cmd, env=env, stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL, start_new_session=True)
    return proc


def _plan(request):
    import rclpy
    from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import (
        BoundingVolume,
        Constraints,
        MoveItErrorCodes,
        OrientationConstraint,
        PositionConstraint,
    )
    from moveit_msgs.srv import GetPositionIK
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from shape_msgs.msg import SolidPrimitive

    start = [float(v) for v in request["start_joints"][:6]]
    xyz = [float(v) for v in request["xyz"]]
    timeout_sec = float(request.get("timeout_sec", 45.0))

    rclpy.init()
    node = Node("site_pick_replay_moveit")
    pub = node.create_publisher(JointState, "/joint_states", 10)

    def _publish_js():
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.name = list(JOINTS)
        msg.position = start
        pub.publish(msg)

    timer = node.create_timer(0.05, _publish_js)
    ik = node.create_client(GetPositionIK, "/compute_ik")
    move = ActionClient(node, MoveGroup, "move_action")
    # Humble move_group action is typically /move_action; some overlays use
    # /move_group. Probe both.
    move_alt = ActionClient(node, MoveGroup, "/move_group")

    deadline = time.monotonic() + min(25.0, timeout_sec)
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if ik.service_is_ready() and (
                move.server_is_ready() or move_alt.server_is_ready()):
            break
    else:
        timer.cancel()
        node.destroy_node()
        rclpy.shutdown()
        return {"message": "MoveIt skipped: move_group not ready on isolated domain",
                "t_sec": [], "q": [], "xyz": []}

    client = move if move.server_is_ready() else move_alt
    pose = Pose()
    pose.position = Point(x=xyz[0], y=xyz[1], z=xyz[2])
    pose.orientation = Quaternion(
        x=TOOL_DOWN[0], y=TOOL_DOWN[1], z=TOOL_DOWN[2], w=TOOL_DOWN[3])

    ik_req = GetPositionIK.Request()
    ik_req.ik_request.group_name = "elfin_arm"
    ik_req.ik_request.ik_link_name = "suction_contact_frame"
    ik_req.ik_request.avoid_collisions = True
    ik_req.ik_request.robot_state.joint_state.name = list(JOINTS)
    ik_req.ik_request.robot_state.joint_state.position = start
    stamped = PoseStamped()
    stamped.header.frame_id = "world"
    stamped.pose = pose
    ik_req.ik_request.pose_stamped = stamped
    future = ik.call_async(ik_req)
    ik_deadline = time.monotonic() + 5.0
    while time.monotonic() < ik_deadline and not future.done():
        rclpy.spin_once(node, timeout_sec=0.1)
    ik_joints = None
    if future.done():
        resp = future.result()
        if resp is not None and resp.error_code.val == MoveItErrorCodes.SUCCESS:
            by_name = dict(zip(resp.solution.joint_state.name,
                               resp.solution.joint_state.position))
            if all(name in by_name for name in JOINTS):
                ik_joints = [float(by_name[n]) for n in JOINTS]

    goal = MoveGroup.Goal()
    goal.request.group_name = "elfin_arm"
    goal.request.num_planning_attempts = 8
    goal.request.allowed_planning_time = 5.0
    goal.request.planner_id = "RRTConnect"
    goal.request.max_velocity_scaling_factor = 0.3
    goal.request.max_acceleration_scaling_factor = 0.3
    goal.request.start_state.joint_state.name = list(JOINTS)
    goal.request.start_state.joint_state.position = start
    goal.planning_options.plan_only = True
    goal.planning_options.replan = False
    if ik_joints is not None:
        from moveit_msgs.msg import JointConstraint
        constraints = Constraints()
        for name, value in zip(JOINTS, ik_joints):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(value)
            jc.tolerance_above = 0.02
            jc.tolerance_below = 0.02
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        goal.request.goal_constraints = [constraints]
    else:
        constraints = Constraints()
        pos = PositionConstraint()
        pos.header.frame_id = "world"
        pos.link_name = "suction_contact_frame"
        box = BoundingVolume()
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.005]
        box.primitives = [primitive]
        box.primitive_poses = [pose]
        pos.constraint_region = box
        pos.weight = 1.0
        constraints.position_constraints = [pos]
        orient = OrientationConstraint()
        orient.header.frame_id = "world"
        orient.link_name = "suction_contact_frame"
        orient.orientation = pose.orientation
        orient.absolute_x_axis_tolerance = 0.08
        orient.absolute_y_axis_tolerance = 0.08
        orient.absolute_z_axis_tolerance = 0.08
        orient.weight = 1.0
        constraints.orientation_constraints = [orient]
        goal.request.goal_constraints = [constraints]

    send = client.send_goal_async(goal)
    send_deadline = time.monotonic() + 8.0
    while time.monotonic() < send_deadline and not send.done():
        rclpy.spin_once(node, timeout_sec=0.1)
    if not send.done() or send.result() is None or not send.result().accepted:
        timer.cancel()
        node.destroy_node()
        rclpy.shutdown()
        return {"message": "MoveIt skipped: plan goal rejected",
                "t_sec": [], "q": [], "xyz": []}
    result_fut = send.result().get_result_async()
    res_deadline = time.monotonic() + timeout_sec
    while time.monotonic() < res_deadline and not result_fut.done():
        rclpy.spin_once(node, timeout_sec=0.1)
    timer.cancel()
    payload = {"message": "MoveIt plan failed", "t_sec": [], "q": [], "xyz": []}
    if result_fut.done() and result_fut.result() is not None:
        wrapped = result_fut.result()
        result = getattr(wrapped, "result", wrapped)
        code = int(result.error_code.val)
        traj = result.planned_trajectory.joint_trajectory
        names = list(traj.joint_names)
        t_sec, q = [], []
        for pt in traj.points:
            by_name = dict(zip(names, pt.positions))
            if all(n in by_name for n in JOINTS):
                t_sec.append(float(pt.time_from_start.sec)
                             + 1e-9 * float(pt.time_from_start.nanosec))
                q.append([float(by_name[n]) for n in JOINTS])
        if q and code == MoveItErrorCodes.SUCCESS:
            payload = {
                "message": "MoveIt plan_only ok (%d points, domain %s)"
                % (len(q), os.environ.get("ROS_DOMAIN_ID")),
                "t_sec": t_sec,
                "q": q,
                "xyz": [],
            }
        else:
            payload["message"] = "MoveIt error_code=%s" % code
    node.destroy_node()
    rclpy.shutdown()
    return payload


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--request", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])
    try:
        domain = assert_isolated_domain(os.environ.get("ROS_DOMAIN_ID"))
    except IsolatedDomainError as exc:
        _write(args.out, {"message": str(exc), "t_sec": [], "q": [], "xyz": []})
        return 2
    env = isolated_replay_env(domain)
    os.environ.update({
        "ROS_DOMAIN_ID": env["ROS_DOMAIN_ID"],
        "ROS_LOCALHOST_ONLY": env["ROS_LOCALHOST_ONLY"],
    })
    with open(args.request, encoding="utf-8") as handle:
        request = json.load(handle)
    launch = None
    try:
        launch = _start_launch(env, request.get("timeout_sec", 45.0))
        time.sleep(3.0)
        payload = _plan(request)
    except Exception as exc:  # noqa: BLE001 worker must always write JSON
        payload = {"message": "MoveIt skipped: %s" % exc,
                   "t_sec": [], "q": [], "xyz": []}
    finally:
        _stop_launch(launch)
    _write(args.out, payload)
    return 0


if __name__ == "__main__":
    sys.exit(main())
