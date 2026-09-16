"""Eval-only RGB-D + FK-pose dump for Open3D TSDF samples.

Offline mcap: no ROS graph, never imported by online nodes. Poses come
from bag ``/tf`` + ``/tf_static`` (FK), not Livox IMU odometry.
"""
from __future__ import division

import json
import os

import numpy as np

from luggage_perception.eval.bag_frame_join import (
    dedupe_stamped_entries,
    plan_frame_join,
)
from luggage_perception.eval.bag_mcap_source import (
    COLOR_INFO_TOPIC,
    DEPTH_INFO_TOPIC,
    TF_STATIC_TOPIC,
    TF_TOPIC,
    decode_color_message,
    decode_depth_message,
    find_mcap_file,
    iter_bag_messages,
    scan_bag,
    select_image_topics,
)
from luggage_perception.eval.bag_tf import BagTfBuffer, invert_matrix
from luggage_perception.ros_message_adapters import camera_info_frame_from_msg

NS_PER_MS = 1_000_000


def _bag_name(bag_path, scan):
    if os.path.isdir(bag_path):
        name = os.path.basename(os.path.normpath(scan.bag_path))
    else:
        name = os.path.basename(str(bag_path))
    if name.endswith(".mcap"):
        name = name[:-len(".mcap")]
    return name


def _open3d_intrinsic(frame):
    return {
        "width": int(frame.width),
        "height": int(frame.height),
        "intrinsic_matrix": [
            float(frame.fx), 0.0, float(frame.cx),
            0.0, float(frame.fy), float(frame.cy),
            0.0, 0.0, 1.0,
        ],
    }


def _write_trajectory_log(path, camera_to_world_poses):
    """Open3D Redwood ``.log``: world-to-camera extrinsics (``save_poses``)."""
    lines = []
    for idx, cam_to_world in enumerate(camera_to_world_poses):
        world_to_cam = invert_matrix(cam_to_world)
        lines.append("%d %d %d" % (idx, idx, idx + 1))
        for row in range(4):
            lines.append(" ".join("%.9g" % float(v) for v in world_to_cam[row]))
    with open(path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(lines) + "\n")


def _write_open3d_config_yml(path, fields):
    lines = []
    for key, value in fields.items():
        if isinstance(value, bool):
            rendered = "true" if value else "false"
        elif isinstance(value, str):
            rendered = "'%s'" % value.replace("'", "''")
        else:
            rendered = str(value)
        lines.append("%s: %s" % (key, rendered))
    with open(path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(lines) + "\n")


def _write_png16(path, depth_mm):
    import cv2
    arr = np.asarray(depth_mm)
    if arr.dtype != np.uint16:
        arr = np.clip(arr, 0, 65535).astype(np.uint16)
    if not cv2.imwrite(path, arr):
        raise IOError("failed to write depth png: %s" % path)


def _write_jpeg(path, rgb, quality=90):
    import cv2
    bgr = np.ascontiguousarray(rgb[:, :, ::-1])
    ok = cv2.imwrite(path, bgr, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        raise IOError("failed to write color jpeg: %s" % path)


def export_open3d_rgbd(bag_path, out_dir, world_frame="world",
                       stride=1, max_frames=0, join_tolerance_ms=30.0,
                       jpeg_quality=90, source_iter=None):
    """Dump Open3D reconstruction_system layout from one bag.

    Only frames with a colour/depth join **and** a TF chain from the
    colour optical frame into *world_frame* are numbered into ``color/``
    and ``depth/``. Missing poses fail closed (recorded, not guessed).
    """
    scan = scan_bag(bag_path)
    color_topic, depth_topic = select_image_topics(scan)
    mcap_path = find_mcap_file(bag_path)
    stream = source_iter or (lambda topics: iter_bag_messages(
        mcap_path, topics=topics))
    os.makedirs(out_dir, exist_ok=True)
    color_dir = os.path.join(out_dir, "color")
    depth_dir = os.path.join(out_dir, "depth")
    os.makedirs(color_dir, exist_ok=True)
    os.makedirs(depth_dir, exist_ok=True)

    index_topics = [color_topic, depth_topic, COLOR_INFO_TOPIC,
                    DEPTH_INFO_TOPIC, TF_TOPIC, TF_STATIC_TOPIC]
    color_entries, depth_entries = [], []
    info_first = {}
    tf_buffer = BagTfBuffer()
    for rec in stream(index_topics):
        stamp = rec.header_stamp_ns
        if rec.topic == color_topic:
            color_entries.append((stamp, rec.log_time_ns, None))
        elif rec.topic == depth_topic:
            depth_entries.append((stamp, rec.log_time_ns, None))
        elif rec.topic in (COLOR_INFO_TOPIC, DEPTH_INFO_TOPIC):
            if rec.topic not in info_first:
                info_first[rec.topic] = rec.message
        elif rec.topic == TF_STATIC_TOPIC:
            tf_buffer.add_tf_message(rec.message, static=True)
        elif rec.topic == TF_TOPIC:
            tf_buffer.add_tf_message(rec.message, static=False)

    color_sorted, _color_dup = dedupe_stamped_entries(color_entries)
    depth_sorted, _depth_dup = dedupe_stamped_entries(depth_entries)
    plan = plan_frame_join(
        [s for s, _log, _p in color_sorted],
        [s for s, _log, _p in depth_sorted],
        tolerance_ns=int(join_tolerance_ms * NS_PER_MS))
    pairs = list(plan.pairs)
    stride = max(1, int(stride))
    pairs = pairs[::stride]
    if max_frames and int(max_frames) > 0:
        pairs = pairs[:int(max_frames)]
    wanted_color = {p.stamp_ns: p for p in pairs}
    wanted_depth = {p.depth_stamp_ns for p in pairs}

    info_msg = info_first.get(COLOR_INFO_TOPIC) or info_first.get(
        DEPTH_INFO_TOPIC)
    if info_msg is None:
        raise ValueError("bag has no camera_info for colour or aligned depth")
    frame = camera_info_frame_from_msg(info_msg)
    optical_frame = str(getattr(info_msg.header, "frame_id", "") or "")

    pending_color = {}
    pending_depth = {}
    poses = []
    emitted = []
    skipped = []

    def _try_emit(pair, rgb, depth_mm):
        if rgb is None or depth_mm is None:
            skipped.append({
                "stamp_ns": int(pair.stamp_ns),
                "reason": "decode_failed",
            })
            return
        pose = None
        if optical_frame:
            pose = tf_buffer.lookup_matrix(
                world_frame, optical_frame, pair.stamp_ns)
        if pose is None:
            skipped.append({
                "stamp_ns": int(pair.stamp_ns),
                "reason": "missing_tf",
                "optical_frame": optical_frame,
                "world_frame": world_frame,
            })
            return
        idx = len(poses)
        _write_jpeg(os.path.join(color_dir, "%06d.jpg" % idx), rgb,
                    quality=jpeg_quality)
        _write_png16(os.path.join(depth_dir, "%06d.png" % idx), depth_mm)
        poses.append(np.asarray(pose, dtype=np.float64))
        emitted.append({
            "index": idx,
            "stamp_ns": int(pair.stamp_ns),
            "depth_stamp_ns": int(pair.depth_stamp_ns),
            "join": pair.source,
        })

    def _flush_ready():
        for color_stamp, pair in list(wanted_color.items()):
            if (pair.stamp_ns not in pending_color
                    or pair.depth_stamp_ns not in pending_depth):
                continue
            _try_emit(pair, pending_color.pop(pair.stamp_ns),
                      pending_depth.pop(pair.depth_stamp_ns, None))
            wanted_color.pop(color_stamp, None)

    for rec in stream([color_topic, depth_topic]):
        if rec.topic == color_topic and rec.header_stamp_ns in wanted_color:
            pending_color[rec.header_stamp_ns] = decode_color_message(
                rec.message)
        elif rec.topic == depth_topic and rec.header_stamp_ns in wanted_depth:
            pending_depth[rec.header_stamp_ns] = decode_depth_message(
                rec.message)
        _flush_ready()

    for stamp_ns, _pair in list(wanted_color.items()):
        skipped.append({
            "stamp_ns": int(stamp_ns),
            "reason": "image_not_in_second_pass",
        })

    intrinsic_path = os.path.join(out_dir, "intrinsic.json")
    traj_path = os.path.join(out_dir, "trajectory.log")
    with open(intrinsic_path, "w", encoding="utf-8") as handle:
        json.dump(_open3d_intrinsic(frame), handle, indent=2)
        handle.write("\n")
    _write_trajectory_log(traj_path, poses)

    dataset_root = os.path.abspath(out_dir) + os.sep
    yml_fields = {
        "name": _bag_name(bag_path, scan),
        "path_dataset": dataset_root,
        "path_intrinsic": os.path.abspath(intrinsic_path),
        "path_trajectory": os.path.abspath(traj_path),
        "depth_folder": "depth",
        "color_folder": "color",
        "depth_min": 0.1,
        "depth_max": 3.0,
        "depth_scale": 1000.0,
        "integrate_color": True,
        "voxel_size": 0.01,
        "device": "CUDA:0",
        "engine": "tensor",
        "multiprocessing": False,
    }
    yml_path = os.path.join(out_dir, "config.yml")
    _write_open3d_config_yml(yml_path, yml_fields)
    config_path = os.path.join(out_dir, "config.json")
    with open(config_path, "w", encoding="utf-8") as handle:
        json.dump(yml_fields, handle, indent=2)
        handle.write("\n")

    summary = {
        "bag": os.path.abspath(scan.bag_path),
        "mcap": os.path.abspath(scan.mcap_path),
        "out_dir": os.path.abspath(out_dir),
        "color_topic": color_topic,
        "depth_topic": depth_topic,
        "optical_frame": optical_frame,
        "world_frame": world_frame,
        "n_joined": len(plan.pairs),
        "n_selected": len(pairs),
        "n_emitted": len(emitted),
        "n_skipped": len(skipped),
        "tf_frames": sorted(tf_buffer.frames()),
        "emitted": emitted,
        "skipped": skipped,
        "files": {
            "color": "color/%06d.jpg",
            "depth": "depth/%06d.png",
            "trajectory": "trajectory.log",
            "intrinsic": "intrinsic.json",
            "config": "config.yml",
            "config_json": "config.json",
        },
    }
    with open(os.path.join(out_dir, "manifest.json"), "w",
              encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2)
        handle.write("\n")
    return summary
