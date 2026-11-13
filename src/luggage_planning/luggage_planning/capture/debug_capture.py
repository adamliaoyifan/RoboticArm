#!/usr/bin/env python3
"""Failure-time capture: occupancy snapshot + source cloud + depth frame.

Node-layer helper for ``motion_planner_node``: when a segment fails to plan
or execute, freeze the cargo occupancy the planner was reasoning on and the
sensor data it came from, so a failure can be replayed and audited offline
(same evidence rule as the boundary dumps). Everything here is best effort:
an artifact that cannot be captured or written records itself under
``missing`` and never raises — dumping must not be able to fail a motion.
"""

from __future__ import division

import json
import os
import struct
import time

import numpy as np

from luggage_perception.ros_message_adapters import (
    cloud_points_from_msg,
    depth_array_from_msg,
)

# PLY decimation ceiling (matches the cargo-map driver's cloud dumps).
CLOUD_MAX_POINTS = 20000


def _stamp_sec(stamp):
    try:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9
    except (AttributeError, TypeError, ValueError):
        return None


def write_ply_xyz(path, points):
    """Binary little-endian XYZ PLY (empty face element for CloudComparer)."""
    points = np.asarray(points, dtype="<f4")
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must be (N, 3)")
    header = (
        "ply\n"
        "format binary_little_endian 1.0\n"
        "element vertex %d\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "element face 0\n"
        "property list uchar int vertex_indices\n"
        "end_header\n" % int(points.shape[0])).encode("ascii")
    tmp = path + ".tmp"
    with open(tmp, "wb") as handle:
        handle.write(header)
        handle.write(points.tobytes(order="C"))
    os.replace(tmp, path)


def _decimate(points, cap=CLOUD_MAX_POINTS):
    if points.shape[0] <= cap:
        return points, 1
    step = int(points.shape[0] // cap) + 1
    return points[::step], step


def _capture_cloud(dump_root, tag, msg):
    out = {"path": None, "stamp": None, "frame_id": None,
           "n_points": None, "decimation": None}
    if msg is None:
        return out, "cloud_absent"
    points = cloud_points_from_msg(msg)
    if points is None or points.shape[0] == 0:
        return out, "cloud_decode_failed"
    points, step = _decimate(points)
    path = os.path.join(dump_root, "fail_%s_cloud.ply" % tag)
    write_ply_xyz(path, points)
    out.update({
        "path": path,
        "stamp": _stamp_sec(msg.header.stamp),
        "frame_id": str(msg.header.frame_id),
        "n_points": int(points.shape[0]),
        "decimation": int(step),
    })
    return out, None


def _capture_depth(dump_root, tag, msg):
    out = {"path": None, "stamp": None, "frame_id": None,
           "shape": None, "encoding": None}
    if msg is None:
        return out, "depth_absent"
    array = depth_array_from_msg(msg)
    if array is None:
        return out, "depth_decode_failed"
    path = os.path.join(dump_root, "fail_%s_depth.npy" % tag)
    tmp = path + ".tmp"
    # File object, not a path: np.save appends .npy to path arguments.
    with open(tmp, "wb") as handle:
        np.save(handle, np.asarray(array), allow_pickle=False)
    os.replace(tmp, path)
    out.update({
        "path": path,
        "stamp": _stamp_sec(msg.header.stamp),
        "frame_id": str(msg.header.frame_id),
        "shape": [int(array.shape[0]), int(array.shape[1])],
        "encoding": str(msg.encoding),
    })
    return out, None


def write_failure_capture(dump_root, tag, surface=None, surface_age_s=None,
                          cloud_msg=None, depth_msg=None):
    """Write ``fail_<tag>_{surface_2d.json,cloud.ply,depth.npy}`` + sidecar.

    Returns a JSON-friendly dict for the boundary record:
    ``{"surface_2d_dump", "cloud", "depth", "sidecar", "missing"}``. Never
    raises; per-artifact failures land in ``missing`` with a reason.
    """
    capture = {
        "tag": str(tag),
        "t_wall": time.time(),
        "surface_2d_dump": None,
        "map_revision": None,
        "geometry_hash": None,
        "surface_age_s": surface_age_s,
        "cloud": None,
        "depth": None,
        "sidecar": None,
        "missing": [],
    }
    try:
        os.makedirs(dump_root, exist_ok=True)
    except OSError as exc:
        capture["missing"].append("dump_root: %s" % exc)
        return capture
    if surface is None:
        capture["missing"].append("surface_2d_absent")
    else:
        try:
            path = os.path.join(dump_root, "fail_%s_surface_2d.json" % tag)
            tmp = path + ".tmp"
            with open(tmp, "w", encoding="utf-8") as handle:
                json.dump(surface, handle, indent=1, sort_keys=True,
                          default=str)
                handle.write("\n")
            os.replace(tmp, path)
            capture["surface_2d_dump"] = path
            capture["map_revision"] = surface.get("map_revision")
            capture["geometry_hash"] = surface.get("geometry_hash")
        except (OSError, TypeError, ValueError) as exc:
            capture["missing"].append("surface_2d: %s" % exc)
    cloud, why = _capture_cloud(dump_root, tag, cloud_msg)
    capture["cloud"] = cloud
    if why:
        capture["missing"].append(why)
    depth, why = _capture_depth(dump_root, tag, depth_msg)
    capture["depth"] = depth
    if why:
        capture["missing"].append(why)
    try:
        sidecar = os.path.join(dump_root, "fail_%s_capture.json" % tag)
        tmp = sidecar + ".tmp"
        with open(tmp, "w", encoding="utf-8") as handle:
            json.dump(capture, handle, indent=1, sort_keys=True,
                      default=str)
            handle.write("\n")
        os.replace(tmp, sidecar)
        capture["sidecar"] = sidecar
    except (OSError, TypeError, ValueError) as exc:
        capture["missing"].append("sidecar: %s" % exc)
    return capture
