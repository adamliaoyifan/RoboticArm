#!/usr/bin/env python3
"""PF-R6-RANSAC-RESEARCH simulation capture node (research-only, ROS).

Runs alongside the live perception stack (accepted sim profile) and
records, per detection frame, the *exact non-privileged inputs* the
support estimator sees: the same-stamp raw depth cloud transformed to
the world frame plus the detector's own top estimate. The spawner
service drives carryon/standard/large trials; spawn responses provide
eval-only GT.

Nothing is published and no production node is modified. Comparators run
offline (bench_offline.py) on the captured npz files.

The cloud decoder below is a verbatim copy of the committed
``ros_message_adapters.cloud_points_from_msg`` logic (offset-aware
FLOAT32 XYZ decode) so the capture path matches production decoding.

Usage (after `source install/setup.bash`):
  ros2 run ... or: PYTHONPATH=research/pf_r6_ransac:$PYTHONPATH \
  python3 research/pf_r6_ransac/pfr6bench/capture_sim.py \
      --out /home/adamliao/work/pf_r6_ransac_data/sim_capture \
      --trials 30 --frames-per-trial 6
"""

from __future__ import division

import argparse
import json
import math
import os
import threading
import time
from collections import OrderedDict

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from luggage_msgs.msg import DetectionFrame
from luggage_msgs.srv import SpawnNextBox

try:
    import tf2_ros
except ImportError:  # pragma: no cover - ROS env required
    tf2_ros = None


def cloud_points_from_msg(msg):
    """(N,3) float64 XYZ or None (committed adapter logic, verbatim)."""
    if msg.point_step <= 0:
        return None
    fields = {field.name: field for field in msg.fields}
    if not {"x", "y", "z"} <= set(fields):
        return None
    for name in ("x", "y", "z"):
        field = fields[name]
        if field.datatype != PointField.FLOAT32 or field.count != 1:
            return None
        if field.offset + 4 > msg.point_step:
            return None
    npoints = int(msg.width) * int(msg.height)
    if npoints <= 0:
        return np.zeros((0, 3), dtype=np.float64)
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    npoints = min(npoints, raw.size // int(msg.point_step))
    if npoints <= 0:
        return np.zeros((0, 3), dtype=np.float64)
    cloud = np.zeros((npoints, 3), dtype=np.float32)
    for i, name in enumerate(("x", "y", "z")):
        off = fields[name].offset
        cloud[:, i] = np.frombuffer(
            msg.data, dtype=np.float32, count=npoints,
            offset=off)[0:npoints] if msg.point_step == 12 else \
            raw[off::msg.point_step][:npoints].view(np.float32)
    return cloud.astype(np.float64)


class CaptureNode(Node):

    def __init__(self, args):
        super().__init__("pf_r6_ransac_research_capture")
        self.args = args
        self.frame_seq = 0
        self.rows = []
        self.raw_buffer = OrderedDict()
        self.raw_lock = threading.Lock()
        self.det_frames = []
        self.det_lock = threading.Lock()
        self.tf_buffer = tf2_ros.Buffer() if tf2_ros else None
        if tf2_ros:
            self.tf_listener = tf2_ros.TransformListener(
                self.tf_buffer, self)
        stream_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            PointCloud2, "/luggage/preprocessed/camera/depth/points",
            self._cloud_cb, stream_qos)
        self.create_subscription(
            DetectionFrame, "/luggage/perception/detection_frame",
            self._det_cb, stream_qos)
        self.spawn_client = self.create_client(
            SpawnNextBox, "/pickup_box_spawner/spawn_next_box")
        self.workspace = (
            tuple(args.workspace_center), tuple(args.workspace_half))

    def _cloud_cb(self, msg):
        key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        with self.raw_lock:
            self.raw_buffer[key] = msg
            while len(self.raw_buffer) > 24:
                self.raw_buffer.popitem(last=False)

    def _det_cb(self, msg):
        with self.det_lock:
            self.det_frames.append(msg)
            self.det_frames = self.det_frames[-40:]

    def _pop_cloud(self, key, attempts=6, period=0.05):
        for _ in range(attempts):
            with self.raw_lock:
                msg = self.raw_buffer.get(key)
            if msg is not None:
                return msg
            time.sleep(period)
        return None

    def _cloud_world(self, msg):
        pts = cloud_points_from_msg(msg)
        if pts is None or not len(pts):
            return None
        pts = pts[np.isfinite(pts).all(axis=1)]
        if not len(pts):
            return None
        if self.tf_buffer is None:
            return None
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        try:
            tf = self.tf_buffer.lookup_transform(
                "world", msg.header.frame_id, stamp,
                rclpy.duration.Duration(seconds=0.5))
            t = tf.transform.translation
            q = tf.transform.rotation
            yaw = 2.0 * math.atan2(q.z, q.w)  # optical depth frame: yaw only
            # Full 3x3 rotation from quaternion for correctness.
            xw, yw, zw, cw = q.x, q.y, q.z, q.w
            R = np.array([
                [1 - 2 * (yw * yw + zw * zw),
                 2 * (xw * yw - zw * cw),
                 2 * (xw * zw + yw * cw)],
                [2 * (xw * yw + zw * cw),
                 1 - 2 * (xw * xw + zw * zw),
                 2 * (yw * zw - xw * cw)],
                [2 * (xw * zw - yw * cw),
                 2 * (yw * zw + xw * cw),
                 1 - 2 * (xw * xw + yw * yw)]], dtype=np.float64)
            return pts @ R.T + np.array(
                [t.x, t.y, t.z], dtype=np.float64)
        except Exception:
            return None

    def _top_from_frame(self, fr):
        """Rebuild a TopSurfaceEstimate-shaped dict from DetectionFrame."""
        b = fr.box
        top_pose = b.top_surface_pose
        q = top_pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return {
            "center_xy": [top_pose.position.x, top_pose.position.y],
            "top_z": float(top_pose.position.z),
            "yaw": float(yaw),
            "width": float(b.width),
            "depth": float(b.depth),
            "pca_valid": bool(fr.pca_valid),
        }

    def capture_frame(self, fr, trial, gt):
        key = (fr.header.stamp.sec, fr.header.stamp.nanosec)
        cloud_msg = self._pop_cloud(key)
        row = {
            "frame_seq": int(fr.frame_seq),
            "trial": int(trial),
            "stamp": "%d.%09d" % key,
            "gt_box_id": gt.get("box_id"),
            "gt_height_m": gt.get("height_m"),
            "pca_valid": bool(fr.pca_valid),
            "pca_reason": fr.pca_reason,
            "production_support_valid": bool(fr.support_valid),
            "production_support_reason": fr.support_reason,
            "production_support_z": float(fr.support_z),
            "production_top_z": float(
                fr.box.top_surface_pose.position.z),
            "cloud_found": cloud_msg is not None,
        }
        top = None
        if cloud_msg is not None and fr.pca_valid:
            world = self._cloud_world(cloud_msg)
            if world is not None:
                top = self._top_from_frame(fr)
                row.update({
                    "raw_points": int(len(world)),
                    "top": top,
                })
                self.frame_seq += 1
                fid = "trial%02d_frame%04d" % (trial, self.frame_seq)
                np.savez_compressed(
                    os.path.join(self.args.out, "%s.npz" % fid),
                    raw_world=world.astype(np.float32))
                row["fid"] = fid
        self.rows.append(row)
        return row

    def run_trials(self, executor):
        while not self.spawn_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("waiting for spawner service ...")
        self.get_logger().info(
            "settling %.1f s before trial 0" % self.args.settle)
        time.sleep(self.args.settle)
        gt = {}
        for trial in range(self.args.trials):
            req = SpawnNextBox.Request()
            fut = self.spawn_client.call_async(req)
            deadline = time.time() + 10.0
            while not fut.done() and time.time() < deadline:
                time.sleep(0.05)
            resp = fut.result() if fut.done() else None
            if resp is None or not resp.success:
                self.get_logger().error(
                    "spawn failed at trial %d" % trial)
                continue
            b = resp.box
            gt = {
                "box_id": b.id,
                "height_m": float(b.height),
                "width_m": float(b.width),
                "depth_m": float(b.depth),
                "pose_z": float(b.pose.position.z),
            }
            # Drain old detection frames, then collect fresh ones.
            with self.det_lock:
                self.det_frames.clear()
            t_end = time.time() + self.args.frames_window
            while time.time() < t_end:
                time.sleep(0.2)
            with self.det_lock:
                frames = list(self.det_frames)
            n = 0
            for fr in frames[-self.args.frames_per_trial:]:
                row = self.capture_frame(fr, trial, gt)
                if row.get("cloud_found"):
                    n += 1
            self.get_logger().info(
                "trial %02d box=%s captured=%d/%d" % (
                    trial, gt.get("box_id"), n, len(frames)))
        with open(os.path.join(self.args.out, "capture_rows.json"),
                  "w") as fh:
            json.dump(self.rows, fh, indent=2)
        self.get_logger().info(
            "done: %d rows -> %s" % (len(self.rows), self.args.out))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--trials", type=int, default=30)
    ap.add_argument("--frames-per-trial", type=int, default=6)
    ap.add_argument("--frames-window", type=float, default=6.0)
    ap.add_argument("--settle", type=float, default=6.0)
    ap.add_argument("--workspace-center", nargs=2,
                    type=float, default=[0.0, 0.0])
    ap.add_argument("--workspace-half", nargs=2,
                    type=float, default=[0.5, 0.5])
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    rclpy.init()
    node = CaptureNode(args)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        node.run_trials(executor)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
