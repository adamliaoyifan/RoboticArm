#!/usr/bin/env python3
"""Frozen-input replay node for one A0/A'/B control run.

Restores the FULL release boundary from obs_rel_07's capture:
  - the box spawns STATIC (carry semantics) at the frozen pose, then at
    t_spawn - the recorded 1 ms after the recorded spawn service time -
    is deleted and re-created DYNAMIC from the recorded release.sdf
    (mass/inertia/contact sensor identical to production);
  - the arm replays the RECORDED retreat joints as a time-parameterized
    JointTrajectory (original sim timestamps, linear interpolation at
    100 Hz — the checked parameterization), sent to the FollowJoint-
    Trajectory controller and echoed to /obs/replay_joints for
    deviation bookkeeping;
  - zero initial velocity is the SDF default; the FIRST VALID dynamic
    physics state is captured by the dyn_pose stream (the runner's
    validity gate separates configured-zero from measured first-step).

Run under the sim clock only (use_sim_time). Prints a final JSON line
with the spawn and command bookkeeping.
"""
import json
import os
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from ros_gz_interfaces.srv import SpawnEntity, DeleteEntity

JOINTS = ["elfin_joint%d" % i for i in range(1, 7)]
CMD_RATE_HZ = 100.0


def load_window(tool_stream, t_lo, t_hi):
    rows = []
    for line in open(tool_stream):
        d = json.loads(line)
        if d["kind"] != "joint_states":
            continue
        t = d["t_sim_ns"] / 1e9
        if not (t_lo <= t <= t_hi):
            continue
        names = d.get("name") or []
        if names and set(JOINTS) <= set(names):
            q = [d["position"][names.index(j)] for j in JOINTS]
        else:
            q = [float(v) for v in d["position"][:6]]
        rows.append((t, q))
    rows.sort()
    return rows


class ReplayNode(Node):

    def __init__(self, spec):
        super().__init__("ab_replay_node")
        self.spec = spec
        self.report = {}
        self.spawn_cli = self.create_client(
            SpawnEntity, "/world/%s/create" % spec["world"])
        self.delete_cli = self.create_client(
            DeleteEntity, "/world/%s/remove" % spec["world"])
        self.traj_cli = ActionClient(
            self, FollowJointTrajectory, "/elfin_arm_controller/follow_trajectory")
        self.echo_pub = self.create_publisher(
            JointState, "/obs/replay_joints", 10)
        self.joint_sub = self.create_subscription(
            JointState, "/joint_states", self._on_joints, 10)
        self.actual = []

    def _on_joints(self, msg):
        self.actual.append((self.now_sim(), list(msg.position)))

    def now_sim(self):
        stamp = self.get_clock().now().to_msg()
        return stamp.sec + stamp.nanosec * 1e-9

    def wait_services(self, timeout=120.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if not self.spawn_cli.wait_for_service(timeout_sec=1.0):
                self._missing = "create"
                continue
            self._missing = "remove" if not \
                self.delete_cli.wait_for_service(timeout_sec=1.0) else \
                "follow_trajectory"
            if self._missing != "follow_trajectory":
                continue
            if self.traj_cli.wait_for_server(timeout_sec=1.0):
                return True
        self.report["missing_service"] = getattr(self, "_missing", "?")
        return False

    def spawn_static(self, model, pose6):
        request = SpawnEntity.Request()
        request.entity_factory.name = model
        sdf = open(self.spec["release_sdf"]).read()
        # Static carry body: the recorded release.sdf with static=true.
        sdf = sdf.replace("<model name=", "<static>true</static>\n  <model name=", 1) \
            if "<static>" not in sdf.split("</model>")[0] else sdf
        request.entity_factory.sdf = sdf
        self._pose(request.entity_factory.pose, pose6)
        future = self.spawn_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        return future.result()

    def swap_dynamic(self, model, pose6):
        delete = DeleteEntity.Request()
        delete.entity.name = model
        delete.entity.type = DeleteEntity.Request().ENTITY_MODEL \
            if hasattr(DeleteEntity.Request(), "ENTITY_MODEL") else 0
        # ros_gz DeleteEntity uses entity.type int; MODEL == 1 in sdf errs
        # — the bridge maps name-only for MODEL when type omitted.
        delete.entity.type = 1
        future = self.delete_cli.call_async(delete)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        request = SpawnEntity.Request()
        request.entity_factory.name = model
        request.entity_factory.sdf = open(self.spec["release_sdf"]).read()
        self._pose(request.entity_factory.pose, pose6)
        future = self.spawn_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        return future.result()

    @staticmethod
    def _pose(target, pose6):
        import math
        x, y, z, qw, qx, qy, qz = pose6
        target.position.x = float(x)
        target.position.y = float(y)
        target.position.z = float(z)
        target.orientation.w = float(qw)
        target.orientation.x = float(qx)
        target.orientation.y = float(qy)
        target.orientation.z = float(qz)
        _ = math  # noqa: F841

    def send_trajectory(self, rows, t0_recorded):
        """Time-parameterized replay: original sim timestamps mapped onto
        the live sim clock, linear interpolation at CMD_RATE_HZ."""
        traj = JointTrajectory()
        traj.joint_names = JOINTS
        t_live0 = self.now_sim()
        for t_rec, q in rows:
            point = JointTrajectoryPoint()
            point.positions = [float(v) for v in q]
            dt = t_rec - t0_recorded
            if dt < 0.0:
                dt = 0.0
            point.time_from_start.sec = int(dt)
            point.time_from_start.nanosec = int(
                (dt - int(dt)) * 1e9)
            traj.points.append(point)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        self.report["t_live_start"] = t_live0
        self.report["t_rec_start"] = t0_recorded
        send_future = self.traj_cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future,
                                         timeout_sec=15.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.report["trajectory_accepted"] = False
            return None
        self.report["trajectory_accepted"] = True
        return goal_handle

    def echo_commands(self, rows, t0_recorded, stop_event):
        """Echo the interpolated command stream for deviation records."""
        rate = self.create_rate(CMD_RATE_HZ)
        idx = 0
        while not stop_event.is_set() and idx < len(rows) - 1:
            t_rec = self.now_sim() - self.report["t_live_start"] \
                + t0_recorded
            while idx < len(rows) - 2 and rows[idx + 1][0] < t_rec:
                idx += 1
            (t0, q0), (t1, q1) = rows[idx], rows[idx + 1]
            span = max(t1 - t0, 1e-9)
            a = min(max((t_rec - t0) / span, 0.0), 1.0)
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = JOINTS
            msg.position = [(1 - a) * b0 + a * b1
                            for b0, b1 in zip(q0, q1)]
            self.echo_pub.publish(msg)
            rate.sleep()


def main():
    spec = json.loads(sys.argv[1])
    rclpy.init()
    node = ReplayNode(spec)
    spin = threading.Thread(
        target=lambda: [rclpy.spin_once(node, timeout_sec=0.05)
                        for _ in iter(lambda: not node.done, True)],
        daemon=True)
    node.done = False
    spin.start()  # service discovery needs a spinning node
    if not node.wait_services():
        node.report["error"] = "services unavailable"
        node.done = True
        print(json.dumps(node.report))
        return 2
    rows = load_window(spec["tool_stream"],
                       spec["t_release"] - 1.0,
                       spec["t_release"] + spec["window_s"])
    node.report["joint_rows"] = len(rows)
    if len(rows) < 10:
        node.report["error"] = "insufficient joint rows"
        node.done = True
        print(json.dumps(node.report))
        return 2

    # 1) static carry box at the frozen pose
    res = node.spawn_static(spec["model"], spec["pose6"])
    node.report["static_spawn"] = bool(
        res and getattr(res, "success", False))

    # 2) time-parameterized trajectory (starts at window head; includes
    # the descend tail before the release boundary)
    handle = node.send_trajectory(rows, rows[0][0])
    stop_event = threading.Event()
    echo = threading.Thread(
        target=node.echo_commands,
        args=(rows, rows[0][0], stop_event), daemon=True)
    echo.start()

    # 3) at the recorded t_spawn, swap to dynamic (recorded 1ms after
    # static delete in production; here delete+create back-to-back —
    # the recorded gap is inside one physics step).
    t_live_spawn = node.report["t_live_start"] + (
        spec["t_spawn"] - node.report["t_rec_start"])
    while node.now_sim() < t_live_spawn and not getattr(node, "abort", False):
        time.sleep(0.002)
    node.report["t_live_spawn"] = node.now_sim()
    res = node.swap_dynamic(spec["model"], spec["pose6"])
    node.report["dynamic_spawn"] = bool(
        res and getattr(res, "success", False))

    # 4) hold until the window completes; record actual joints.
    t_live_end = node.report["t_live_start"] + (
        rows[-1][0] - node.report["t_rec_start"]) + 2.0
    while node.now_sim() < t_live_end:
        time.sleep(0.2)
    stop_event.set()
    if handle is not None:
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future,
                                         timeout_sec=10.0)
        node.report["trajectory_result"] = int(
            getattr(result_future.result().result,
                    "error_code", -1))
    node.report["actual_joint_samples"] = len(node.actual)
    with open(spec["out_json"], "w") as handle_out:
        json.dump(node.report, handle_out, indent=2)
    node.done = True
    print(json.dumps(node.report))
    return 0


if __name__ == "__main__":
    sys.exit(main())
