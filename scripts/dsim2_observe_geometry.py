#!/usr/bin/env python3
"""DSIM-2 pickup_observe geometry probe. Does not launch Gazebo."""
from __future__ import annotations

import argparse
import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_ros import Buffer, TransformListener

from luggage_msgs.msg import DetectionFrame
from luggage_msgs.srv import SpawnNextBox
from luggage_perception.detect_overlay import project_detection
from luggage_perception.depth_deprojection import deproject_selected
from luggage_perception.top_support_estimator import (
    TopSupportConfig,
    estimate_local_support,
    estimate_top_surface,
)


POSE_VALUES = [1.8806, -1.7736, -1.0491, 4.4439, 1.7721, 1.8473]
JOINT_NAMES = [
    "elfin_joint1", "elfin_joint2", "elfin_joint3",
    "elfin_joint4", "elfin_joint5", "elfin_joint6",
]
NEAR_MARGIN = 0.30
VALID_DEPTH_MIN = 0.95
PIXEL_MARGIN = 10
MIN_SUPPORT = 80


def _stamp_key(stamp):
    return (int(stamp.sec), int(stamp.nanosec))


class _K:
    def __init__(self, fx, fy, cx, cy):
        self.fx = float(fx)
        self.fy = float(fy)
        self.cx = float(cx)
        self.cy = float(cy)


def _tf_rt(msg: TransformStamped):
    t = msg.transform.translation
    r = msg.transform.rotation
    qx, qy, qz, qw = r.x, r.y, r.z, r.w
    rot = np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)],
    ], dtype=np.float64)
    trans = np.array([t.x, t.y, t.z], dtype=np.float64)
    return rot, trans


class Probe(Node):
    def __init__(self):
        super().__init__("dsim2_observe_geometry")
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        rel = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.depth = {}
        self.info = None
        self.joints = None
        self.frames = []
        self.create_subscription(Image, "/camera/depth/image_raw", self._on_depth, qos)
        self.create_subscription(
            CameraInfo, "/camera/depth/camera_info", self._on_info, qos)
        self.create_subscription(
            CameraInfo, "/camera/depth/camera_info", self._on_info, rel)
        self.create_subscription(JointState, "/joint_states", self._on_js, rel)
        self.create_subscription(
            DetectionFrame, "/luggage/perception/detection_frame",
            self._on_frame, qos)
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)
        self._spawn = self.create_client(SpawnNextBox, "/pickup_box_spawner/spawn_next_box")

    def _on_depth(self, msg):
        self.depth[_stamp_key(msg.header.stamp)] = msg
        if len(self.depth) > 20:
            oldest = min(self.depth)
            del self.depth[oldest]

    def _on_info(self, msg):
        self.info = msg

    def _on_js(self, msg):
        self.joints = msg

    def _on_frame(self, msg):
        self.frames.append(msg)
        if len(self.frames) > 30:
            self.frames = self.frames[-30:]


def _optical_to_world(points, rot, trans):
    shifted = np.asarray(points, dtype=np.float64) - trans.reshape(1, 3)
    return shifted @ rot


def _roi_points_world(mm, info, u0, v0, u1, v1, rot, trans, stride=1):
    height, width = int(info.height), int(info.width)
    u0 = max(0, min(width - 1, int(u0)))
    u1 = max(0, min(width - 1, int(u1)))
    v0 = max(0, min(height - 1, int(v0)))
    v1 = max(0, min(height - 1, int(v1)))
    uu, vu = np.meshgrid(
        np.arange(u0, u1 + 1, stride),
        np.arange(v0, v1 + 1, stride),
        indexing="xy",
    )
    k = _K(info.k[0], info.k[4], info.k[2], info.k[5])
    optical, n = deproject_selected(mm, uu.reshape(-1), vu.reshape(-1), k)
    if n == 0:
        return optical.astype(np.float64)
    return _optical_to_world(optical, rot, trans)


def _estimate_support(mm, info, bbox, rot, trans):
    if bbox is None:
        return None, None, "no_bbox"
    u0, v0, u1, v1 = bbox
    cargo = _roi_points_world(mm, info, u0, v0, u1, v1, rot, trans, stride=1)
    fx = max(1e-6, float(info.k[0]))
    pad = int(math.ceil(fx * 0.22 / 0.60)) + 4
    raw = _roi_points_world(
        mm, info, u0 - pad, v0 - pad, u1 + pad, v1 + pad, rot, trans, stride=1)
    cfg = TopSupportConfig(min_support_points=MIN_SUPPORT, crop_to_workspace=False)
    timing = {}
    top = estimate_top_surface(cargo, None, config=cfg, timing=timing)
    if top is None:
        return None, None, "top_unobservable:%s" % timing
    support = estimate_local_support(raw, top, None, config=cfg, timing=timing)
    return top, support, support.reason if support is not None else "no_support"


def _joint_error(js):
    if js is None:
        return None
    name_to_pos = dict(zip(js.name, js.position))
    errs = []
    for name, target in zip(JOINT_NAMES, POSE_VALUES):
        if name not in name_to_pos:
            return None
        errs.append(abs(float(name_to_pos[name]) - float(target)))
    return max(errs)


def _measure_one(probe, catalog_id, timeout=25.0):
    deadline = time.monotonic() + timeout
    while not probe._spawn.wait_for_service(timeout_sec=1.0):
        if time.monotonic() > deadline:
            return {"catalog_id": catalog_id, "ok": False, "reason": "no_spawn_service"}
    probe.frames = []
    future = probe._spawn.call_async(SpawnNextBox.Request())
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(probe, timeout_sec=0.05)
    if not future.done():
        return {"catalog_id": catalog_id, "ok": False, "reason": "spawn_timeout"}
    resp = future.result()
    if not resp.success:
        return {"catalog_id": catalog_id, "ok": False, "reason": resp.message}
    box = resp.box
    settle_end = time.monotonic() + 3.0
    while time.monotonic() < settle_end:
        rclpy.spin_once(probe, timeout_sec=0.05)
    info = probe.info
    if info is None or not probe.depth:
        return {"catalog_id": catalog_id, "ok": False, "reason": "no_camera"}
    depth_msg = probe.depth[max(probe.depth)]
    try:
        tf_msg = probe._tf.lookup_transform(
            info.header.frame_id, "world", rclpy.time.Time())
    except Exception as exc:  # noqa: BLE001
        return {"catalog_id": catalog_id, "ok": False, "reason": "tf:%s" % exc}
    rot, trans = _tf_rt(tf_msg)
    k = (float(info.k[0]), float(info.k[4]), float(info.k[2]), float(info.k[5]))
    pos = (box.pose.position.x, box.pose.position.y, box.pose.position.z)
    quat = (box.pose.orientation.x, box.pose.orientation.y,
            box.pose.orientation.z, box.pose.orientation.w)
    size = (box.width, box.depth, box.height)
    centre, corners, corner_valid, _ = project_detection(
        pos, quat, size, rot, trans, k)
    valid_uv = corners[corner_valid]
    if valid_uv.size == 0:
        margin = -1
        bbox = None
    else:
        u0, v0 = valid_uv.min(axis=0)
        u1, v1 = valid_uv.max(axis=0)
        bbox = [float(u0), float(v0), float(u1), float(v1)]
        margin = min(u0, v0, info.width - 1 - u1, info.height - 1 - v1)
    mm = np.frombuffer(bytes(depth_msg.data), dtype="<u2").reshape(
        depth_msg.height, depth_msg.width)
    if bbox is None:
        valid_ratio = 0.0
        zmin = zmax = None
    else:
        u0, v0, u1, v1 = [int(round(v)) for v in bbox]
        u0 = max(0, min(info.width - 1, u0))
        u1 = max(0, min(info.width - 1, u1))
        v0 = max(0, min(info.height - 1, v0))
        v1 = max(0, min(info.height - 1, v1))
        roi = mm[v0:v1 + 1, u0:u1 + 1]
        valid = roi > 0
        valid_ratio = float(valid.mean()) if roi.size else 0.0
        if valid.any():
            zmin = float(roi[valid].min()) * 0.001
            zmax = float(roi[valid].max()) * 0.001
        else:
            zmin = zmax = None
    detector_support = None
    top_valid = None
    for frame in reversed(probe.frames):
        if frame.box.id:
            detector_support = int(frame.support_inliers)
            top_valid = bool(frame.box.top_surface_valid)
            break
    top_est, support_est, support_reason = _estimate_support(
        mm, info, bbox, rot, trans)
    support = None
    if support_est is not None and support_est.reason == "ok":
        support = int(support_est.inlier_count)
    elif detector_support is not None:
        support = detector_support
    cam_xyz = None
    try:
        cam_tf = probe._tf.lookup_transform(
            "world", info.header.frame_id, rclpy.time.Time())
        t = cam_tf.transform.translation
        cam_xyz = [float(t.x), float(t.y), float(t.z)]
    except Exception:  # noqa: BLE001
        cam_xyz = None
    joint_err = _joint_error(probe.joints)
    rec = {
        "catalog_id": catalog_id,
        "gt_id": box.id,
        "gt_size": list(size),
        "gt_xyz": list(pos),
        "camera_optical_xyz_world": cam_xyz,
        "image": {"width": int(info.width), "height": int(info.height)},
        "optical_z_min": zmin,
        "optical_z_max": zmax,
        "valid_depth_ratio": valid_ratio,
        "pixel_margin": float(margin),
        "bbox": bbox,
        "support_inliers": support,
        "support_reason": support_reason,
        "detector_support_inliers": detector_support,
        "top_surface_valid": top_valid if top_valid is not None else (
            top_est is not None),
        "top_z": None if top_est is None else float(top_est.top_z),
        "support_z": None if support_est is None else float(support_est.support_z),
        "joint_abs_err_max": joint_err,
        "ideal_range_gap": None if zmin is None else abs(zmin - 0.6),
        "ik_collision_note": (
            "pickup_observe is the stored collision-aware IK; "
            "spawn_at_observe holds it. Settle is max joint abs error."
        ),
        "gates": {
            "optical_z": bool(zmin is not None and zmin >= NEAR_MARGIN),
            "valid_depth": bool(valid_ratio >= VALID_DEPTH_MIN),
            "margin": bool(margin >= PIXEL_MARGIN),
            "support": bool(support is not None and support >= MIN_SUPPORT),
            "settle": bool(joint_err is not None and joint_err <= 0.15),
        },
    }
    rec["ok"] = bool(all(rec["gates"].values()))
    rec["reason"] = "pass" if rec["ok"] else "gate_fail"
    return rec


def expected_lying_optical_z(cam_z=1.9, platform_z=0.86, height=0.32):
    return cam_z - (platform_z + height)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--ids", default="carryon,standard,large")
    args = ap.parse_args()
    rclpy.init()
    probe = Probe()
    warmup = time.monotonic() + 8.0
    while time.monotonic() < warmup:
        rclpy.spin_once(probe, timeout_sec=0.05)
    results = []
    for catalog_id in [s.strip() for s in args.ids.split(",") if s.strip()]:
        results.append(_measure_one(probe, catalog_id))
    payload = {
        "pose_values": POSE_VALUES,
        "expected_0p24_m_note": (
            "0.24 m is 1.90 - 0.86 - 0.80 for a standing 0.80 m case. "
            "Catalog spawn is lying; large height 0.32 m => optical Z "
            "about %.3f m." % expected_lying_optical_z()
        ),
        "results": results,
        "all_ok": all(r.get("ok") for r in results),
    }
    with open(args.out, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, default=str)
    print(json.dumps(payload, indent=2))
    probe.destroy_node()
    rclpy.shutdown()
    return 0 if payload["all_ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
