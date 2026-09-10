"""Eval-only Gate-4 failure dumps. Not imported by online nodes.

Writes stamp-joined RGB / depth / overlay / mask, cargo clouds, and the
JSON intermediates a human needs to tell a tipped box from a top-plane
miss. Scoring is unchanged: this module only serializes arrays.
"""
from __future__ import division

import collections
import json
import math
import os
import re

import numpy as np

from luggage_perception.eval.detection_gate_sampling import (
    build_aligned_dump,
    pick_joined_stamp,
    stamp_sec_from_key,
    write_png,
)


_GZ_POSE_BLOCK = re.compile(
    r"pose\s*\{([^{}]*(?:\{[^{}]*\}[^{}]*)*)\}", re.MULTILINE)
_GZ_NAME = re.compile(r'name:\s*"([^"]+)"')
_GZ_FIELD = re.compile(
    r"(x|y|z|w):\s*([-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)")


class StampBuffer(object):
    """Keep the last *maxlen* messages keyed by header stamp."""

    def __init__(self, maxlen=40):
        self._maxlen = int(maxlen)
        self._items = collections.OrderedDict()

    def push(self, msg):
        stamp = msg.header.stamp
        key = (int(stamp.sec), int(stamp.nanosec))
        self._items[key] = msg
        self._items.move_to_end(key)
        while len(self._items) > self._maxlen:
            self._items.popitem(last=False)

    def snapshot(self):
        return dict(self._items)


def trial_is_failure(recovery, settled=None):
    """True when this trial would fail C1 / placement-recovery."""
    recovery = recovery or {}
    if not recovery.get("spawn_ok", True):
        return True
    t_valid = recovery.get("t_first_valid_sec")
    t_full = recovery.get("t_first_full3d_sec")
    if t_valid is None or float(t_valid) > 1.4:
        return True
    if t_full is None or float(t_full) > 1.4:
        return True
    if int(recovery.get("n_settled") or 0) < 30:
        return True
    settled = list(settled or [])
    if settled and all(not row.get("top_surface_valid") for row in settled):
        return True
    return False


def select_dump_stamps(keys, count=3):
    """First / middle / last joined stamps, de-duplicated and ordered."""
    ordered = sorted(keys)
    if not ordered:
        return []
    count = max(1, int(count))
    if len(ordered) == 1 or count == 1:
        return [ordered[-1]]
    if count == 2 or len(ordered) == 2:
        picked = [ordered[0], ordered[-1]]
    else:
        mid = ordered[len(ordered) // 2]
        picked = [ordered[0], mid, ordered[-1]]
    out = []
    for key in picked:
        if key not in out:
            out.append(key)
    return out


def cargo_summary(points, frame_id=""):
    """Finite XYZ stats for a dumped cloud."""
    pts = np.asarray(points if points is not None else [], dtype=np.float64)
    pts = pts.reshape(-1, 3) if pts.size else np.zeros((0, 3), dtype=np.float64)
    finite = np.isfinite(pts).all(axis=1) if len(pts) else np.zeros((0,), bool)
    pts = pts[finite]
    summary = {
        "frame_id": str(frame_id or ""),
        "n": int(len(pts)),
        "n_dropped_nonfinite": int((~finite).sum()) if finite.size else 0,
    }
    if len(pts) == 0:
        return summary
    summary["min_xyz"] = [float(v) for v in pts.min(axis=0)]
    summary["max_xyz"] = [float(v) for v in pts.max(axis=0)]
    summary["mean_xyz"] = [float(v) for v in pts.mean(axis=0)]
    return summary


def crop_workspace_xy(points, center_xy, half_extents):
    pts = np.asarray(points if points is not None else [], dtype=np.float64)
    pts = pts.reshape(-1, 3) if pts.size else np.zeros((0, 3), dtype=np.float64)
    if len(pts) == 0:
        return pts
    cx, cy = float(center_xy[0]), float(center_xy[1])
    hx, hy = float(half_extents[0]), float(half_extents[1])
    mask = (
        (pts[:, 0] >= cx - hx) & (pts[:, 0] <= cx + hx)
        & (pts[:, 1] >= cy - hy) & (pts[:, 1] <= cy + hy)
    )
    return pts[mask]


def _finite_xyz(points):
    pts = np.asarray(points if points is not None else [], dtype=np.float64)
    pts = pts.reshape(-1, 3) if pts.size else np.zeros((0, 3), dtype=np.float64)
    if len(pts):
        pts = pts[np.isfinite(pts).all(axis=1)]
    return pts


def write_xyz(path, points):
    """Headerless X Y Z text. CloudCompare File > Open treats .xyz as a cloud."""
    pts = _finite_xyz(points)
    np.savetxt(path, pts, fmt="%.6f")
    return int(len(pts))


def write_ply_xyz(path, points):
    """Binary little-endian XYZ PLY that CloudCompare loads as a point cloud.

    Point-only ASCII PLY (no faces) is opened by CloudCompare's mesh filter
    and then reports "Nothing to load". A binary cloud plus an explicit
    empty face element is accepted.
    """
    pts = _finite_xyz(points)
    header = (
        "ply\n"
        "format binary_little_endian 1.0\n"
        "comment gate4 dump point cloud\n"
        "element vertex %d\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "element face 0\n"
        "property list uchar int vertex_indices\n"
        "end_header\n"
    ) % len(pts)
    with open(path, "wb") as handle:
        handle.write(header.encode("ascii"))
        if len(pts):
            handle.write(np.ascontiguousarray(pts, dtype="<f4").tobytes())
    return int(len(pts))


def rpy_from_quat(x, y, z, w):
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (w * y - z * x)
    pitch = (
        math.copysign(math.pi * 0.5, sinp)
        if abs(sinp) >= 1.0 else math.asin(sinp))
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return roll, pitch, math.atan2(siny, cosy)


def parse_gz_pose_info(text):
    """Parse ``ign topic -e`` Pose_V text into {name: pose dict}."""
    poses = {}
    for body in _GZ_POSE_BLOCK.findall(text or ""):
        named = _GZ_NAME.search(body)
        if named is None:
            continue
        pos_m = re.search(r"position\s*\{([^}]*)\}", body)
        ori_m = re.search(r"orientation\s*\{([^}]*)\}", body)
        pos = dict(_GZ_FIELD.findall(pos_m.group(1))) if pos_m else {}
        ori = dict(_GZ_FIELD.findall(ori_m.group(1))) if ori_m else {}
        qx = float(ori.get("x", 0.0))
        qy = float(ori.get("y", 0.0))
        qz = float(ori.get("z", 0.0))
        qw = float(ori.get("w", 1.0))
        roll, pitch, yaw = rpy_from_quat(qx, qy, qz, qw)
        poses[named.group(1)] = {
            "position": {
                "x": float(pos.get("x", 0.0)),
                "y": float(pos.get("y", 0.0)),
                "z": float(pos.get("z", 0.0)),
            },
            "orientation": {"x": qx, "y": qy, "z": qz, "w": qw},
            "rpy_rad": [roll, pitch, yaw],
            "tilt_rad": max(abs(roll), abs(pitch)),
        }
    return poses


def model_pose_from_gz(poses, model_name):
    if not poses or not model_name:
        return None
    name = str(model_name)
    if name in poses:
        return poses[name]
    for key, pose in poses.items():
        if key == name or key.startswith(name + "::"):
            return pose
    return None


def trial_folder_name(trial, recovery, box_id, settled=None):
    failed = trial_is_failure(recovery, settled)
    reason = "spawn_fail"
    if recovery.get("spawn_ok", True):
        t_full = recovery.get("t_first_full3d_sec")
        reasons = [
            str(row.get("pca_reason") or "")
            for row in (settled or [])
            if not row.get("top_surface_valid")]
        reason = "ok"
        if failed:
            if t_full is not None and float(t_full) > 1.4:
                reason = "slow_full3d"
            else:
                reason = reasons[-1] if reasons else "fail"
    slug = re.sub(r"[^A-Za-z0-9._-]+", "_", str(reason)).strip("_")[:60]
    ident = re.sub(r"[^A-Za-z0-9._-]+", "_", str(box_id or "unknown"))[:80]
    kind = "fail" if failed else "ok"
    return "trial_%02d_%s_%s_%s" % (int(trial), kind, slug or "na", ident)


def write_snapshot_dir(dest, images=None, arrays=None, extras=None,
                       clouds=None):
    """Write one joined camera/cloud snapshot. Returns dest."""
    os.makedirs(dest, exist_ok=True)
    for name, arr in (images or {}).items():
        if arr is None:
            continue
        write_png(os.path.join(dest, "%s.png" % name), arr)
    for name, arr in (arrays or {}).items():
        if arr is None:
            continue
        np.save(os.path.join(dest, "%s.npy" % name), np.asarray(arr))
    cloud_meta = {}
    for name, payload in (clouds or {}).items():
        if payload is None:
            continue
        points = payload.get("points")
        frame_id = payload.get("frame_id") or ""
        npy_path = os.path.join(dest, "%s.npy" % name)
        ply_path = os.path.join(dest, "%s.ply" % name)
        xyz_path = os.path.join(dest, "%s.xyz" % name)
        np.save(npy_path, np.asarray(points if points is not None else []))
        n_ply = write_ply_xyz(ply_path, points)
        write_xyz(xyz_path, points)
        cloud_meta[name] = cargo_summary(points, frame_id)
        cloud_meta[name]["ply_vertices"] = n_ply
    payload = dict(extras or {})
    payload["clouds"] = cloud_meta
    with open(os.path.join(dest, "meta.json"), "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True, default=str)
        handle.write("\n")
    return dest


def extract_top_ransac(points_world, workspace=None, config=None):
    """Replay v1 horizontal top RANSAC and return inlier/outlier clouds.

    Same crop / voxel / RANSAC as ``estimate_top_surface``. Eval-only.
    """
    from luggage_perception.luggage_box_estimator import (
        _pca_rectangle,
        _ransac_horizontal_plane,
        _refine_rectangle,
        voxel_downsample,
    )
    from luggage_perception.top_support_estimator import (
        TopSupportConfig,
        _crop_workspace,
    )

    config = config or TopSupportConfig(
        workspace_center_xy=(-1.0, 0.0),
        workspace_half_extents=(0.5, 0.5),
    )
    if workspace is None:
        workspace = (config.workspace_center_xy, config.workspace_half_extents)
    points = np.asarray(
        points_world if points_world is not None else [],
        dtype=np.float64)
    points = points.reshape(-1, 3) if points.size else np.zeros((0, 3))
    points = points[np.isfinite(points).all(axis=1)]
    record = {
        "ok": False,
        "reason": "DETECT_TOP_UNOBSERVABLE",
        "n_input": int(len(points)),
        "n_cropped": 0,
        "n_voxel": 0,
        "n_inliers": 0,
        "plane_z": None,
        "center_xy": None,
        "width": None,
        "depth": None,
        "yaw": None,
        "inliers": np.zeros((0, 3), dtype=np.float64),
        "outliers": np.zeros((0, 3), dtype=np.float64),
        "voxel": np.zeros((0, 3), dtype=np.float64),
        "workspace_center_xy": [float(v) for v in workspace[0]],
        "workspace_half_extents": [float(v) for v in workspace[1]],
        "voxel_size": float(config.voxel_size),
        "ransac_dist_thresh": float(config.ransac_dist_thresh),
    }
    cropped = _crop_workspace(points, workspace[0], workspace[1])
    record["n_cropped"] = int(len(cropped))
    if len(cropped) < int(config.min_top_points):
        record["reason"] = "too_few_cropped"
        return record
    voxel = voxel_downsample(cropped, config.voxel_size)
    record["voxel"] = voxel
    record["n_voxel"] = int(len(voxel))
    if len(voxel) < int(config.min_top_points):
        record["reason"] = "too_few_voxel"
        return record
    inlier_mask, plane_z = _ransac_horizontal_plane(
        voxel,
        max_iter=config.ransac_max_iter,
        dist_thresh=config.ransac_dist_thresh,
        min_inliers=max(3, config.min_top_points // 2),
        normal_thresh=math.radians(config.normal_tolerance_deg),
    )
    if inlier_mask is None or plane_z is None:
        record["outliers"] = voxel
        record["reason"] = "ransac_no_plane"
        return record
    inliers = voxel[inlier_mask]
    outliers = voxel[~inlier_mask]
    yaw, extent_0, extent_1, _ratio = _pca_rectangle(inliers[:, :2])
    try:
        yaw, extent_0, extent_1, center = _refine_rectangle(
            inliers[:, :2], yaw)
    except (ValueError, TypeError):
        center = inliers[:, :2].mean(axis=0)
    width, depth = (
        (extent_0, extent_1) if extent_0 >= extent_1 else (extent_1, extent_0))
    record.update({
        "ok": True,
        "reason": "ok",
        "plane_z": float(plane_z),
        "center_xy": [float(center[0]), float(center[1])],
        "width": float(width),
        "depth": float(depth),
        "yaw": float(yaw),
        "n_inliers": int(len(inliers)),
        "inliers": inliers,
        "outliers": outliers,
    })
    return record


def write_top_ransac_dir(dest, points_world, workspace=None, config=None):
    """Write top_inliers / top_outliers / top_ransac.json under dest."""
    os.makedirs(dest, exist_ok=True)
    record = extract_top_ransac(points_world, workspace=workspace, config=config)
    write_ply_xyz(os.path.join(dest, "top_inliers.ply"), record["inliers"])
    write_xyz(os.path.join(dest, "top_inliers.xyz"), record["inliers"])
    write_ply_xyz(os.path.join(dest, "top_outliers.ply"), record["outliers"])
    write_xyz(os.path.join(dest, "top_outliers.xyz"), record["outliers"])
    write_ply_xyz(os.path.join(dest, "top_voxel.ply"), record["voxel"])
    write_xyz(os.path.join(dest, "top_voxel.xyz"), record["voxel"])
    serial = {
        k: v for k, v in record.items()
        if k not in ("inliers", "outliers", "voxel")
    }
    with open(os.path.join(dest, "top_ransac.json"), "w", encoding="utf-8") as handle:
        json.dump(serial, handle, indent=2, sort_keys=True)
        handle.write("\n")
    return record


def write_index(dump_root, records, extra=None):
    """INDEX.md + index.json for a human pass over the dump set."""
    os.makedirs(dump_root, exist_ok=True)
    index = {"trials": list(records or []), "extra": extra or {}}
    with open(os.path.join(dump_root, "index.json"), "w", encoding="utf-8") as handle:
        json.dump(index, handle, indent=2, sort_keys=True, default=str)
        handle.write("\n")
    lines = [
        "# Gate-4 trial dumps",
        "",
        "Open `late/color.png`, `late/overlay.png`, and `late/cargo_camera.ply`.",
        "Failed trials also keep `early/` and `mid/`. `meta.json` has detector",
        "`stream_stats` (including `timing_ms.pipeline` crop/inlier counts),",
        "segmenter/filter stats, and GT. `gz_pose.json` is the live Gazebo pose.",
        "",
        "| trial | folder | result | box | pca_reason | n_cargo | t_valid_s | t_full3d_s |",
        "|---|---|---|---|---|---|---|---|",
    ]
    for rec in records or []:
        lines.append(
            "| %s | `%s` | %s | %s | %s | %s | %s | %s |" % (
                rec.get("trial"),
                rec.get("folder") or "",
                "FAIL" if rec.get("failed") else "ok",
                rec.get("box_id") or "",
                rec.get("pca_reason") or "",
                rec.get("n_cargo_points"),
                rec.get("t_first_valid_sec"),
                rec.get("t_first_full3d_sec"),
            ))
    lines.append("")
    with open(os.path.join(dump_root, "INDEX.md"), "w", encoding="utf-8") as handle:
        handle.write("\n".join(lines) + "\n")
    return os.path.join(dump_root, "INDEX.md")
