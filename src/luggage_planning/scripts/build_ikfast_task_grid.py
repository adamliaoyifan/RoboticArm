#!/usr/bin/env python3
"""Turn a multi-branch IKFast sample into the task-grid files.

Reads the sparse base-frame archive (one or more joint branches per pose),
keeps the branch nearest the observe seed, and bins tool centers that lie
inside the container hull. Interior cells with no solution are unreachable.
A stored seed whose forearm or wrist crosses a closed container face is
dropped.
"""

from __future__ import division

import argparse
import hashlib
import math
import os
import sys

import numpy as np

_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
for _pkg in ("luggage_planning", "luggage_description"):
    _path = os.path.join(_SRC, _pkg)
    if _path not in sys.path:
        sys.path.insert(0, _path)

from luggage_description.container_geometry import (  # noqa: E402
    contains_point,
    normalize_descriptor,
)
from luggage_description.scene_tf_config_utils import (  # noqa: E402
    _mat_vec,
    _rpy_matrix,
    container_in_base_link,
    load_scene_tf_config,
)
from luggage_planning.atlas.task_grid import (  # noqa: E402
    REACHABLE,
    UNREACHABLE,
    reduce_pose_seeds,
    save_task_grid,
    save_workspace,
)

RESOLUTION = 0.05

# Fixed chain from elfin_s20_kinematics.urdf: (xyz, rpy, revolute).
_CHAIN = (
    ((0.0, 0.0, 0.171), (0.0, 0.0, 0.0), True),
    ((0.0, -0.2295, 0.0), (math.pi / 2.0, 0.0, 0.0), True),
    ((-0.85, 0.0, -0.1885), (0.0, 0.0, math.pi), True),
    ((0.712, 0.0, 0.0), (0.0, 0.0, 0.0), True),
    ((0.0, 0.0, 0.138), (-math.pi / 2.0, 0.0, 0.0), True),
    ((0.0, 0.0, 0.138), (math.pi / 2.0, 0.0, 0.0), True),
    ((0.0, 0.0, 0.1257), (0.0, 0.0, math.pi), False),
    ((0.0006, 0.0, 0.0), (-1.56451314, 0.0, math.pi), False),
    ((0.0, -0.286, 0.0), (math.pi / 2.0, 0.0, 0.0), False),
)


def _md5(path):
    digest = hashlib.md5()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _homog(xyz, rpy):
    rot = np.asarray(_rpy_matrix(*rpy), dtype=np.float64)
    out = np.eye(4)
    out[:3, :3] = rot
    out[:3, 3] = xyz
    return out


def _rot_z(angle):
    c, s = math.cos(angle), math.sin(angle)
    out = np.eye(4)
    out[0, 0] = c
    out[0, 1] = -s
    out[1, 0] = s
    out[1, 1] = c
    return out


def link_points_in_base(joints):
    """Forearm, wrist, and suction origins in elfin_base_link."""
    frame = np.eye(4)
    q_index = 0
    points = []
    for xyz, rpy, revolute in _CHAIN:
        frame = frame.dot(_homog(xyz, rpy))
        if revolute:
            frame = frame.dot(_rot_z(float(joints[q_index])))
            q_index += 1
        if q_index >= 3:
            points.append(frame[:3, 3].copy())
    return points


def pierces_closed_face(point_container, geometry):
    """True when a link point crosses a wall, floor, ceiling, or chamfer.

    Points outboard of the door (negative X) are allowed. The arm stands
    outside the opening.
    """
    x = float(point_container[0])
    if x < -geometry.half_x:
        return False
    return not contains_point(geometry, point_container)


def _base_to_container(point, translation, inv_rot):
    delta = (
        float(point[0]) - translation[0],
        float(point[1]) - translation[1],
        float(point[2]) - translation[2],
    )
    return _mat_vec(inv_rot, delta)


def build(sample_path, scene_path, urdf_path, out_dir):
    raw = np.load(sample_path)
    positions, yaws, joints, counts = reduce_pose_seeds(
        raw["base_position"], raw["yaw_bin"], raw["joints"],
        raw["n_solutions"], raw["pose_id"])
    os.makedirs(out_dir, exist_ok=True)
    workspace_path = os.path.join(out_dir, "workspace_ikfast.npz")
    save_workspace(workspace_path, positions, yaws, joints, counts)

    config = load_scene_tf_config(scene_path)
    inner = config["container"]["inner"]
    geometry = normalize_descriptor({
        "frame_id": "container_link",
        "length": inner["length"],
        "width": inner["width"],
        "floor_z": inner["floor_z"],
        "ceiling_z": inner["ceiling_z"],
        "chamfer": dict(inner["chamfer"]),
    })
    n_depth = int(math.ceil((geometry.length - 1e-9) / RESOLUTION))
    n_across = int(math.ceil((geometry.width - 1e-9) / RESOLUTION))
    n_height = int(math.ceil(
        (geometry.ceiling_z - geometry.floor_z - 1e-9) / RESOLUTION))
    shape = (n_depth, n_across, n_height, 2)
    origin_d = -geometry.half_x
    origin_a = -geometry.half_y
    origin_z = geometry.floor_z
    status = np.zeros(shape, dtype=np.uint8)
    contact = np.full(shape + (6,), np.nan, dtype=np.float64)
    best = np.full(shape, np.inf, dtype=np.float64)

    xyz, rpy = container_in_base_link(config)
    rot = _rpy_matrix(*rpy)
    inv = [
        [rot[0][0], rot[1][0], rot[2][0]],
        [rot[0][1], rot[1][1], rot[2][1]],
        [rot[0][2], rot[1][2], rot[2][2]],
    ]
    pierced = 0
    for i, base in enumerate(positions):
        local = _base_to_container(base, xyz, inv)
        if not contains_point(geometry, local):
            continue
        if any(pierces_closed_face(_base_to_container(p, xyz, inv), geometry)
               for p in link_points_in_base(joints[i])):
            pierced += 1
            continue
        id_ = int(math.floor((local[0] - origin_d) / RESOLUTION))
        ia = int(math.floor((local[1] - origin_a) / RESOLUTION))
        iz = int(math.floor((local[2] - origin_z) / RESOLUTION))
        iyaw = int(yaws[i])
        if not (0 <= id_ < n_depth and 0 <= ia < n_across and 0 <= iz < n_height):
            continue
        if iyaw < 0 or iyaw > 1:
            continue
        center = (
            origin_d + (id_ + 0.5) * RESOLUTION,
            origin_a + (ia + 0.5) * RESOLUTION,
            origin_z + (iz + 0.5) * RESOLUTION,
        )
        dist = sum((local[k] - center[k]) ** 2 for k in range(3))
        if dist < best[id_, ia, iz, iyaw]:
            best[id_, ia, iz, iyaw] = dist
            contact[id_, ia, iz, iyaw] = joints[i]
            status[id_, ia, iz, iyaw] = REACHABLE

    for id_ in range(n_depth):
        for ia in range(n_across):
            for iz in range(n_height):
                center = (
                    origin_d + (id_ + 0.5) * RESOLUTION,
                    origin_a + (ia + 0.5) * RESOLUTION,
                    origin_z + (iz + 0.5) * RESOLUTION,
                )
                if not contains_point(geometry, center):
                    continue
                for iyaw in range(2):
                    if status[id_, ia, iz, iyaw] == 0:
                        status[id_, ia, iz, iyaw] = UNREACHABLE

    reachable = int(np.count_nonzero(status == REACHABLE))
    unreachable = int(np.count_nonzero(status == UNREACHABLE))
    meta = {
        "resolution": RESOLUTION,
        "origin_d": origin_d,
        "origin_a": origin_a,
        "origin_z": origin_z,
        "shape": list(shape),
        "yaw_bins": [0.0, math.pi / 2.0],
        "opening_at_high_index": False,
        "opening_side": "negative_x",
        "frame": "container_link",
        "base_frame": "elfin_base_link",
        "ik_link": "suction_contact_frame",
        "scene_tf_hash": _md5(scene_path),
        "geometry_hash": geometry.geometry_hash,
        "urdf_hash": _md5(urdf_path),
        "ikfast_solver": "ikfast0x10000049.Transform6D.0_1_2_3_4_5",
        "ikfast_maxcasedepth": 1,
        "payload_size": None,
        "workspace_count": int(positions.shape[0]),
        "task_reachable_cells": reachable,
        "task_unreachable_interior_cells": unreachable,
        "closed_face_rejects": pierced,
    }
    save_task_grid(
        os.path.join(out_dir, "task_reachable.npz"),
        os.path.join(out_dir, "task_reachable.yaml"),
        status, contact, contact.copy(), meta)
    print("workspace_poses", positions.shape[0])
    print("reachable", reachable, "unreachable", unreachable, "pierced", pierced)
    return meta


def main():
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
    parser = argparse.ArgumentParser()
    parser.add_argument("--sample", required=True)
    parser.add_argument("--scene-tf", default=os.path.join(
        root, "src/luggage_description/config/scene_tf.yaml"))
    parser.add_argument("--urdf", default=os.path.join(
        root, "src/luggage_planning/data/reachability_ikfast/elfin_s20_kinematics.urdf"))
    parser.add_argument("--out-dir", default=os.path.join(
        root, "src/luggage_planning/data/reachability_ikfast"))
    args = parser.parse_args()
    build(args.sample, args.scene_tf, args.urdf, args.out_dir)


if __name__ == "__main__":
    main()
