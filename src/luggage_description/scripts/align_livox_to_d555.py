#!/usr/bin/env python3
"""One-shot: ICP Livox onto D555 in the camera optical frame, freeze mount xacro.

Camera Layer 3 stays the reference. Do not edit Layers 1-2 or livox_optical.

  ros2 run luggage_description align_livox_to_d555.py
  ros2 run luggage_description align_livox_to_d555.py --write
"""

from __future__ import division

import argparse
import json
import os
import shutil
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, PointCloud2
from tf2_ros import Buffer, TransformListener

_PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _PKG not in sys.path:
    sys.path.insert(0, _PKG)

from luggage_description.handeye_layer3 import T_t_q, parse_xacro_xyz_rpy  # noqa: E402
from luggage_description.livox_d555_align import (  # noqa: E402
    apply_cloud_correction,
    conjugate_se3,
    fit_horizontal_plane,
    frustum_mask,
    height_above_plane,
    icp_point_to_plane,
    invert_T,
    mount_from_adapter_livox,
    nn_rmse,
    replace_mid360_mount,
    rotation_deg,
    set_origin_height,
    transform_points,
    translation_m,
    voxel_downsample,
)


def _qos():
    return QoSProfile(
        depth=5,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
    )


def _xyz_from_cloud(msg):
    from sensor_msgs_py import point_cloud2

    gen = point_cloud2.read_points(
        msg, field_names=("x", "y", "z"), skip_nans=False
    )
    pts = np.array(list(gen))
    if pts.size == 0:
        return np.zeros((0, 3), dtype=np.float64)
    if pts.dtype.names:
        return np.stack(
            [pts["x"], pts["y"], pts["z"]], axis=1
        ).astype(np.float64)
    pts = np.asarray(pts, dtype=np.float64)
    return pts.reshape((-1, 3))


def _T_from_tf(msg):
    t = msg.transform.translation
    q = msg.transform.rotation
    return T_t_q((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))


class AlignNode(Node):
    def __init__(self, args):
        super().__init__("align_livox_to_d555")
        self._args = args
        self._cam_info = None
        self._d555 = None
        self._livox = []
        self._tf = Buffer()
        self._listener = TransformListener(self._tf, self)
        self.create_subscription(
            CameraInfo, args.camera_info_topic, self._on_info, _qos()
        )
        self.create_subscription(
            PointCloud2, args.d555_topic, self._on_d555, _qos()
        )
        self.create_subscription(
            PointCloud2, args.livox_topic, self._on_livox, _qos()
        )

    def _on_info(self, msg):
        self._cam_info = msg

    def _on_d555(self, msg):
        self._d555 = msg

    def _on_livox(self, msg):
        self._livox.append(msg)
        if len(self._livox) > 40:
            self._livox = self._livox[-40:]

    def lookup(self, target, source):
        tfm = self._tf.lookup_transform(target, source, rclpy.time.Time())
        return _T_from_tf(tfm)

    def wait_data(self, timeout_s):
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (
                self._d555 is not None
                and self._cam_info is not None
                and len(self._livox) >= int(self._args.livox_frames)
            ):
                return
        missing = []
        if self._d555 is None:
            missing.append(self._args.d555_topic)
        if self._cam_info is None:
            missing.append(self._args.camera_info_topic)
        if len(self._livox) < int(self._args.livox_frames):
            missing.append(
                "%s (%d/%d frames)"
                % (self._args.livox_topic, len(self._livox), self._args.livox_frames)
            )
        raise RuntimeError("timeout waiting for " + ", ".join(missing))


def align(args):
    rclpy.init()
    node = AlignNode(args)
    try:
        node.wait_data(args.timeout)
        info = node._cam_info
        fx, fy = float(info.k[0]), float(info.k[4])
        cx, cy = float(info.k[2]), float(info.k[5])
        d555 = _xyz_from_cloud(node._d555)
        livox = np.concatenate(
            [_xyz_from_cloud(msg) for msg in node._livox[-args.livox_frames :]],
            axis=0,
        )
        cam_frame = node._d555.header.frame_id
        T_cam_livox = node.lookup(cam_frame, "livox_frame")
        T_adapter_cam = node.lookup("eef_mount_adapter", cam_frame)
        T_base_cam = node.lookup("elfin_base_link", cam_frame)
        T_adapter_base = node.lookup("eef_mount_adapter", "elfin_base_link")
        T_al_old = node.lookup("eef_mount_adapter", "livox_frame")
        livox_in_cam = transform_points(T_cam_livox, livox)
        keep = frustum_mask(
            livox_in_cam,
            fx,
            fy,
            cx,
            cy,
            info.width,
            info.height,
            args.z_min,
            args.z_max,
        )
        src = voxel_downsample(livox_in_cam[keep], args.voxel)
        dst = voxel_downsample(d555, args.voxel)
        dst = dst[(dst[:, 2] > args.z_min) & (dst[:, 2] < args.z_max)]
        T_base_liv_old = invert_T(T_adapter_base).dot(T_al_old)
        livox_base = transform_points(T_base_liv_old, livox)
        floor_n, floor_d = fit_horizontal_plane(
            voxel_downsample(livox_base, 0.04),
            expected_origin_height=args.pedestal_height_m,
            height_tol=0.12,
        )
        h_before = height_above_plane(T_base_liv_old[:3, 3], floor_n, floor_d)
        T_h_base = np.eye(4)
        if args.livox_height_m is not None:
            T_h_base[:3, 3] = floor_n * (float(args.livox_height_m) - h_before)
        T_h_cam = conjugate_se3(invert_T(T_base_cam), T_h_base)
        src_h = transform_points(T_h_cam, src)
        before, n_before = nn_rmse(src, dst, max_dist=0.30)
        T_icp, stats = icp_point_to_plane(
            src_h, dst, max_iter=args.max_iter, max_dist=args.max_dist, voxel=args.voxel
        )
        icp_ok = (
            rotation_deg(T_icp) <= args.max_rot_deg
            and translation_m(T_icp) <= args.max_trans_m
        )
        aligned_icp = transform_points(T_icp, src_h)
        after_icp, n_after_icp = nn_rmse(aligned_icp, dst, max_dist=0.15)
        after_h, n_after_h = nn_rmse(src_h, dst, max_dist=0.15)
        if icp_ok and after_icp <= after_h * 1.05:
            T_corr_cam = T_icp.dot(T_h_cam)
            after, n_after = after_icp, n_after_icp
            icp_used = True
        else:
            T_corr_cam = T_h_cam
            after, n_after = after_h, n_after_h
            icp_used = False
        T_corr_adp = conjugate_se3(T_adapter_cam, T_corr_cam)
        T_al_new = apply_cloud_correction(T_al_old, T_corr_adp)
        if args.livox_height_m is not None:
            T_base_liv_new = invert_T(T_adapter_base).dot(T_al_new)
            T_base_liv_new, _h = set_origin_height(
                T_base_liv_new, floor_n, floor_d, args.livox_height_m
            )
            T_al_new = T_adapter_base.dot(T_base_liv_new)
        origin = os.path.join(_PKG, "config", "mid360_origin.xacro")
        _xyz_seed, rpy_seed = parse_xacro_xyz_rpy(
            origin, "mid360_mount_xyz", "mid360_mount_rpy"
        )
        xyz, rpy = mount_from_adapter_livox(T_al_new, rpy_seed=rpy_seed)
        h_after = height_above_plane(
            invert_T(T_adapter_base).dot(T_al_new)[:3, 3], floor_n, floor_d
        )
        result = {
            "camera_frame": cam_frame,
            "d555_points": int(d555.shape[0]),
            "livox_points_raw": int(livox.shape[0]),
            "livox_in_frustum": int(keep.sum()),
            "rmse_before_m": before,
            "rmse_after_m": after,
            "pairs_before": n_before,
            "pairs_after": n_after,
            "icp_used": icp_used,
            "icp": {k: stats[k] for k in ("iter", "pairs", "rmse", "median")},
            "icp_translation_m": translation_m(T_icp),
            "icp_rotation_deg": rotation_deg(T_icp),
            "corr_cam_translation_m": translation_m(T_corr_cam),
            "corr_cam_rotation_deg": rotation_deg(T_corr_cam),
            "livox_height_before_m": h_before,
            "livox_height_after_m": h_after,
            "livox_height_target_m": args.livox_height_m,
            "floor_normal_base": [float(v) for v in floor_n],
            "mid360_mount_xyz": xyz,
            "mid360_mount_rpy": rpy,
            "T_corr_cam": T_corr_cam.tolist(),
            "T_adapter_livox_new": T_al_new.tolist(),
        }
        print(json.dumps(result, indent=2))
        if args.livox_height_m is not None:
            delta_h = abs(h_after - h_before)
            if abs(h_before - args.livox_height_m) > args.max_height_delta_m:
                print(
                    "livox height %.3f m is not within %.3f m of target; "
                    "refusing to write (floor fit?)"
                    % (h_before, args.max_height_delta_m),
                    file=sys.stderr,
                )
                return 4
            if delta_h > args.max_height_delta_m:
                print(
                    "height correction %.3f m exceeds %.3f m; refusing to write"
                    % (delta_h, args.max_height_delta_m),
                    file=sys.stderr,
                )
                return 4
        if args.livox_height_m is None:
            if after > before * 0.85 and after > 0.04:
                print(
                    "ICP did not improve overlap enough; not writing xacro",
                    file=sys.stderr,
                )
                return 2
            if rotation_deg(T_corr_cam) > 20.0 or translation_m(T_corr_cam) > 0.20:
                print(
                    "correction too large; check FOV overlap / arm motion",
                    file=sys.stderr,
                )
                return 3
        elif not icp_used:
            print(
                "ICP rejected (rot/trans or RMSE); keeping height lock only",
                file=sys.stderr,
            )
        if args.write:
            origin = os.path.join(_PKG, "config", "mid360_origin.xacro")
            if args.backup_dir:
                os.makedirs(args.backup_dir, exist_ok=True)
                shutil.copy2(
                    origin, os.path.join(args.backup_dir, "mid360_origin.xacro")
                )
                with open(
                    os.path.join(args.backup_dir, "livox_d555_icp.json"),
                    "w",
                    encoding="utf-8",
                ) as handle:
                    json.dump(result, handle, indent=2)
                    handle.write("\n")
            note = (
                "mid360_mount from D555-referenced ICP %s. "
                "Camera Layer 3 is the reference. livox_optical/IMU stay handbook. "
                "CAD pad was 0.022 0.103 0.038 / 0 pi/2 pi/2."
                % time.strftime("%Y-%m-%d")
            )
            if args.livox_height_m is not None:
                note += " Livox origin height locked to %.3f m above D555 floor." % (
                    args.livox_height_m,
                )
            text = open(origin, encoding="utf-8").read()
            text = replace_mid360_mount(text, xyz, rpy, note)
            if "0.000 0.000 0.047" not in text:
                raise RuntimeError("refusing to write: handbook optical 47 mm missing")
            with open(origin, "w", encoding="utf-8") as handle:
                handle.write(text)
            print("wrote", origin)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--d555-topic", default="/camera/d555/depth/color/points")
    parser.add_argument(
        "--camera-info-topic", default="/camera/d555/depth/camera_info"
    )
    parser.add_argument("--livox-topic", default="/livox/lidar")
    parser.add_argument("--livox-frames", type=int, default=12)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--voxel", type=float, default=0.025)
    parser.add_argument("--max-dist", type=float, default=0.18)
    parser.add_argument("--max-iter", type=int, default=30)
    parser.add_argument("--max-rot-deg", type=float, default=8.0)
    parser.add_argument("--max-trans-m", type=float, default=0.08)
    parser.add_argument(
        "--livox-height-m",
        type=float,
        default=None,
        help="Lock livox_frame origin height above the fitted floor (m).",
    )
    parser.add_argument(
        "--pedestal-height-m",
        type=float,
        default=0.86,
        help="elfin_base_link origin above floor; used to pick the real ground.",
    )
    parser.add_argument(
        "--max-height-delta-m",
        type=float,
        default=0.04,
        help="Refuse height locks larger than this (metres).",
    )
    parser.add_argument("--z-min", type=float, default=0.25)
    parser.add_argument("--z-max", type=float, default=4.5)
    parser.add_argument("--write", action="store_true")
    parser.add_argument(
        "--backup-dir",
        default=os.path.join(
            _PKG, "config", "backups", time.strftime("%Y%m%d_%H%M_livox_d555_icp")
        ),
    )
    args = parser.parse_args(argv)
    return align(args)


if __name__ == "__main__":
    sys.exit(main())
