#!/usr/bin/env python3
"""Three-pose world oracle: 16UC1 deprojection + TF at the image stamp.

Does not launch Gazebo. Does not subscribe to a camera PointCloud2.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration as MsgDuration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from luggage_msgs.srv import GetCurrentBox, SpawnNextBox
from luggage_perception.eval.dsim_three_pose import (
    measure_depth_world,
    quat_xyzw_to_R,
    score_three_pose,
)
from luggage_perception.eval.gate4_dump import model_pose_from_gz, parse_gz_pose_info


JOINTS = [
    "elfin_joint1", "elfin_joint2", "elfin_joint3",
    "elfin_joint4", "elfin_joint5", "elfin_joint6",
]
PICKUP = [1.8806, -1.7736, -1.0491, 4.4439, 1.7721, 1.8473]
POSES = {
    "pickup_observe": list(PICKUP),
    "pickup_j1_plus": [PICKUP[0] + 0.18, *PICKUP[1:]],
    "pickup_j1_minus": [PICKUP[0] - 0.18, *PICKUP[1:]],
}


def _stamp_key(stamp):
    return (int(stamp.sec), int(stamp.nanosec))


def _tf_rt_world_from_optical(msg):
    t = msg.transform.translation
    r = msg.transform.rotation
    rot = quat_xyzw_to_R(r.x, r.y, r.z, r.w)
    trans = np.array([t.x, t.y, t.z], dtype=np.float64)
    return rot, trans, {
        "translation": [float(t.x), float(t.y), float(t.z)],
        "rotation_xyzw": [float(r.x), float(r.y), float(r.z), float(r.w)],
        "child_frame_id": msg.child_frame_id,
        "header_frame_id": msg.header.frame_id,
        "stamp": _stamp_key(msg.header.stamp),
    }


class Oracle(Node):
    def __init__(self):
        super().__init__("dsim_three_pose_oracle")
        be = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST)
        rel = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST)
        self.depth = {}
        self.info = {}
        self.joints = None
        self.create_subscription(Image, "/camera/depth/image_raw", self._on_depth, be)
        self.create_subscription(Image, "/camera/depth/image_raw", self._on_depth, rel)
        self.create_subscription(
            CameraInfo, "/camera/depth/camera_info", self._on_info, be)
        self.create_subscription(
            CameraInfo, "/camera/depth/camera_info", self._on_info, rel)
        self.create_subscription(JointState, "/joint_states", self._on_js, rel)
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)
        self._spawn = self.create_client(SpawnNextBox, "/pickup_box_spawner/spawn_next_box")
        self._current = self.create_client(GetCurrentBox, "/pickup_box_spawner/get_current_box")
        self._fjt = ActionClient(
            self, FollowJointTrajectory,
            "/elfin_arm_controller/follow_joint_trajectory")

    def _on_depth(self, msg):
        self.depth[_stamp_key(msg.header.stamp)] = msg
        while len(self.depth) > 30:
            del self.depth[min(self.depth)]

    def _on_info(self, msg):
        self.info[_stamp_key(msg.header.stamp)] = msg
        while len(self.info) > 30:
            del self.info[min(self.info)]

    def _on_js(self, msg):
        self.joints = msg

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def go_to(self, joints, duration=4.0, timeout=20.0):
        if not self._fjt.wait_for_server(timeout_sec=15.0):
            return False, "no_fjt"
        if self.joints is None:
            return False, "no_joint_state"
        by_name = dict(zip(self.joints.name, self.joints.position))
        start = [float(by_name.get(name, joints[i])) for i, name in enumerate(JOINTS)]
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = list(JOINTS)
        p0 = JointTrajectoryPoint()
        p0.positions = start
        p0.velocities = [0.0] * 6
        p0.time_from_start = MsgDuration(sec=0, nanosec=0)
        p1 = JointTrajectoryPoint()
        p1.positions = [float(v) for v in joints]
        p1.velocities = [0.0] * 6
        sec = int(duration)
        nsec = int(round((duration - sec) * 1e9))
        p1.time_from_start = MsgDuration(sec=sec, nanosec=nsec)
        traj.points = [p0, p1]
        goal.trajectory = traj
        goal.goal_time_tolerance = MsgDuration(sec=2, nanosec=0)
        send = self._fjt.send_goal_async(goal)
        wait = time.monotonic() + 10.0
        while not send.done() and time.monotonic() < wait and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
        handle = send.result() if send.done() else None
        if handle is None or not handle.accepted:
            return False, "fjt_rejected"
        result = handle.get_result_async()
        wait = time.monotonic() + duration + timeout
        while not result.done() and time.monotonic() < wait and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
        if not result.done():
            return False, "fjt_timeout"
        return True, "ok"

    def spawn(self, timeout=25.0):
        deadline = time.monotonic() + timeout
        while not self._spawn.wait_for_service(timeout_sec=1.0):
            if time.monotonic() > deadline:
                return None, "no_spawn_service"
            rclpy.spin_once(self, timeout_sec=0.05)
        future = self._spawn.call_async(SpawnNextBox.Request())
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done() or not future.result().success:
            return None, "spawn_failed"
        return future.result().box, "ok"

    def current_box(self, timeout=5.0):
        if not self._current.wait_for_service(timeout_sec=timeout):
            return None
        future = self._current.call_async(GetCurrentBox.Request())
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            return None
        resp = future.result()
        return resp.box if resp.success else None

    def capture_gz(self, world_name, model_name):
        topic = "/world/%s/pose/info" % world_name
        try:
            proc = subprocess.run(
                ["ign", "topic", "-e", "-t", topic, "--num", "1"],
                capture_output=True, text=True, timeout=3.0, check=False)
        except (OSError, subprocess.TimeoutExpired) as exc:
            return {"error": str(exc), "model": None}
        parsed = parse_gz_pose_info(proc.stdout or "")
        return {
            "model_name": model_name,
            "model": model_pose_from_gz(parsed, model_name),
            "n_entities": len(parsed),
        }

    def measure_pose(self, pose_name):
        self.spin_for(0.4)
        common = set(self.depth).intersection(self.info)
        if not common:
            return {
                "pose_name": pose_name, "ok": False,
                "reason": "no_exact_depth_info",
                "tf_source": "stamp", "used_camera_cloud": False,
            }
        key = max(common)
        depth_msg = self.depth[key]
        info = self.info[key]
        stamp_time = Time.from_msg(depth_msg.header.stamp)
        try:
            tf_msg = self._tf.lookup_transform(
                "world", depth_msg.header.frame_id, stamp_time,
                timeout=Duration(seconds=0.5))
        except Exception as exc:  # noqa: BLE001 - fail closed, never latest TF
            return {
                "pose_name": pose_name, "ok": False,
                "reason": "tf_stamp_miss:%s" % exc,
                "tf_source": "stamp", "used_camera_cloud": False,
                "stamp": list(key),
            }
        rot, trans, tf_rec = _tf_rt_world_from_optical(tf_msg)
        box = self.current_box()
        if box is None:
            return {
                "pose_name": pose_name, "ok": False,
                "reason": "no_gt_box",
                "tf_source": "stamp", "used_camera_cloud": False,
            }
        mm = np.frombuffer(bytes(depth_msg.data), dtype="<u2").reshape(
            depth_msg.height, depth_msg.width)
        k = (float(info.k[0]), float(info.k[4]), float(info.k[2]), float(info.k[5]))
        gt_xyz = (box.pose.position.x, box.pose.position.y, box.pose.position.z)
        gt_quat = (
            box.pose.orientation.x, box.pose.orientation.y,
            box.pose.orientation.z, box.pose.orientation.w)
        gt_size = (box.width, box.depth, box.height)
        rec = measure_depth_world(
            mm, info.width, info.height, k, rot, trans,
            gt_xyz, gt_quat, gt_size, stride=1)
        rec["pose_name"] = pose_name
        rec["stamp"] = list(key)
        rec["frame_id"] = depth_msg.header.frame_id
        rec["tf"] = tf_rec
        rec["tf_source"] = "stamp"
        rec["used_camera_cloud"] = False
        rec["k"] = list(k)
        rec["gt_box"] = {
            "id": str(box.id),
            "xyz": list(gt_xyz),
            "quat_xyzw": list(gt_quat),
            "size": list(gt_size),
        }
        rec["encoding"] = depth_msg.encoding
        rec["image"] = {"width": int(info.width), "height": int(info.height)}
        return rec, depth_msg, info, tf_rec


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--world-name", default="airport_loading")
    ap.add_argument("--settle-sec", type=float, default=4.0)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    rclpy.init()
    node = Oracle()
    node.spin_for(2.0)
    box, spawn_reason = node.spawn()
    if box is None:
        payload = {"pass": False, "failures": ["spawn:%s" % spawn_reason]}
        with open(os.path.join(args.out, "three_pose.json"), "w") as handle:
            json.dump(payload, handle, indent=2)
        print(json.dumps(payload, indent=2))
        node.destroy_node()
        rclpy.shutdown()
        return 1
    node.spin_for(args.settle_sec)
    samples = []
    dumps = []
    for name, joints in POSES.items():
        ok, reason = node.go_to(joints, duration=4.0)
        node.spin_for(args.settle_sec)
        measured = node.measure_pose(name)
        if isinstance(measured, tuple):
            rec, depth_msg, info, tf_rec = measured
        else:
            rec = measured
            depth_msg = info = tf_rec = None
        rec["go_to_ok"] = ok
        rec["go_to_reason"] = reason
        rec["joint_target"] = list(joints)
        gz = node.capture_gz(args.world_name, str(box.id))
        rec["gz_pose"] = gz.get("model")
        samples.append(rec)
        if not rec.get("ok"):
            dump = {
                "pose_name": name,
                "reason": rec.get("reason"),
                "stamp": rec.get("stamp"),
                "tf": tf_rec,
                "gz_pose": gz,
                "camera_info": None if info is None else {
                    "k": list(info.k),
                    "width": int(info.width),
                    "height": int(info.height),
                    "frame_id": info.header.frame_id,
                    "stamp": _stamp_key(info.header.stamp),
                },
            }
            if depth_msg is not None:
                npy_path = os.path.join(args.out, "fail_%s_depth.npy" % name)
                mm = np.frombuffer(bytes(depth_msg.data), dtype="<u2").reshape(
                    depth_msg.height, depth_msg.width)
                np.save(npy_path, mm)
                dump["depth_npy"] = npy_path
            dumps.append(dump)
    node.go_to(POSES["pickup_observe"], duration=4.0)
    verdict = score_three_pose(samples)
    payload = {"verdict": verdict, "samples": samples, "fail_dumps": dumps}
    with open(os.path.join(args.out, "three_pose.json"), "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, default=str)
    print(json.dumps({
        "pass": verdict["pass"],
        "failures": verdict["failures"],
        "n_ok": verdict["n_ok"],
        "out": args.out,
    }, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if verdict["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
