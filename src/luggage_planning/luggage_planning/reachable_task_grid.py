#!/usr/bin/env python3
"""Dense container reachability grid stored as parallel arrays.

Disk layout is the one in
``docs/agents/eng/2026-09-24_1521_reachable-grid-design.md``. Cells are not
Python objects. ``occupied_count`` and ``opening_hops`` are session memory
and are not written into the npz.

ST06/F14: this task grid keeps its ORIGINAL v1 semantics -- pi-folded yaw
bins sampled in base_link, no contract block. It has no runtime consumer
(the unified selector and the motion seed chain read the reachability
atlas); it is explicitly v1-isolated and must never be relabelled as a v2
directed concrete-tool artifact. Every artifact written by this module
carries ``semantics: v1_pi_folded``, and loads expose the marker so a
future consumer can refuse instead of guessing.
"""

from __future__ import division

import math
import os

import numpy as np
import yaml

UNKNOWN = 0
UNREACHABLE = 1
MARGINAL = 2
REACHABLE = 3

OBSERVE_SEED = np.array([0.0, -1.57, 1.57, math.pi, 1.57, 0.0], dtype=np.float64)

# ST06: explicit legacy semantics marker written into every task-grid
# artifact and asserted on load (see module docstring).
V1_SEMANTICS = "v1_pi_folded"


def ang_diff(a, b):
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


def cell_indices(x, y, z, origin_d, origin_a, origin_z, resolution):
    """Return ``(id, ia, iz)`` for a container_link point."""
    res = float(resolution)
    return (
        int(math.floor((float(x) - float(origin_d)) / res)),
        int(math.floor((float(y) - float(origin_a)) / res)),
        int(math.floor((float(z) - float(origin_z)) / res)),
    )


def in_shape(index, shape):
    id_, ia, iz = index
    n_depth, n_across, n_height = shape[:3]
    return 0 <= id_ < n_depth and 0 <= ia < n_across and 0 <= iz < n_height


def save_workspace(path, base_position, yaw_bin, joints, n_solutions):
    """One selected seed per solved pose, in ``elfin_base_link``."""
    np.savez_compressed(
        path,
        base_position=np.asarray(base_position, dtype=np.float32),
        yaw_bin=np.asarray(yaw_bin, dtype=np.uint8),
        joints=np.asarray(joints, dtype=np.float64),
        n_solutions=np.asarray(n_solutions, dtype=np.uint8),
    )


def load_workspace(path):
    data = np.load(path)
    return {
        "base_position": np.asarray(data["base_position"], dtype=np.float32),
        "yaw_bin": np.asarray(data["yaw_bin"], dtype=np.uint8),
        "joints": np.asarray(data["joints"], dtype=np.float64),
        "n_solutions": np.asarray(data["n_solutions"], dtype=np.uint8),
    }


def save_task_grid(npz_path, yaml_path, status, contact_seeds, transit_seeds, meta):
    saved_meta = dict(meta)
    saved_meta["semantics"] = V1_SEMANTICS
    np.savez_compressed(
        npz_path,
        status=np.asarray(status, dtype=np.uint8),
        contact_seeds=np.asarray(contact_seeds, dtype=np.float64),
        transit_seeds=np.asarray(transit_seeds, dtype=np.float64),
    )
    with open(yaml_path, "w", encoding="utf-8") as handle:
        yaml.safe_dump(saved_meta, handle, sort_keys=False)


def load_task_grid(npz_path, yaml_path):
    with open(yaml_path, "r", encoding="utf-8") as handle:
        meta = yaml.safe_load(handle)
    # ST06: expose the declared semantics; artifacts written before the
    # marker are v1 by construction (no v2 writer ever existed here).
    meta.setdefault("semantics", V1_SEMANTICS)
    data = np.load(npz_path)
    status = np.asarray(data["status"], dtype=np.uint8)
    return {
        "meta": meta,
        "status": status,
        "contact_seeds": np.asarray(data["contact_seeds"], dtype=np.float64),
        "transit_seeds": np.asarray(data["transit_seeds"], dtype=np.float64),
        "by_depth": build_by_depth(status),
        "occupied_count": np.zeros(status.shape, dtype=np.uint8),
        "opening_hops": np.full(status.shape, np.uint16(0xFFFF), dtype=np.uint16),
    }


def build_by_depth(status):
    """Per-depth index lists, ``iz`` ascending, only reachable or marginal.

    ``by_depth[n_depth - 1]`` is the back wall when the opening is at the low
    end of axis 0. Walk that list from the start to take the lowest cells first.
    """
    return [_depth_band(status, depth) for depth in range(status.shape[0])]


def _depth_band(status, depth):
    ia, iz, iyaw = np.where(status[depth] >= MARGINAL)
    order = np.argsort(iz, kind="mergesort")
    packed = np.empty(order.shape[0], dtype=np.uint32)
    n_across = status.shape[1]
    n_height = status.shape[2]
    n_yaw = status.shape[3]
    for out_i, src in enumerate(order):
        packed[out_i] = np.uint32(
            ((int(depth) * n_across + int(ia[src])) * n_height + int(iz[src])) * n_yaw
            + int(iyaw[src]))
    return packed


def unpack_index(flat, shape):
    n_across, n_height, n_yaw = shape[1], shape[2], shape[3]
    iyaw = int(flat) % n_yaw
    rest = int(flat) // n_yaw
    iz = rest % n_height
    rest = rest // n_height
    ia = rest % n_across
    depth = rest // n_across
    return depth, ia, iz, iyaw


def select_branch(branches, observe=None):
    """Joint row closest to the observe seed. ``branches`` is (K, 6)."""
    rows = np.asarray(branches, dtype=np.float64)
    if rows.ndim == 1:
        rows = rows.reshape(1, 6)
    target = OBSERVE_SEED if observe is None else np.asarray(observe, dtype=np.float64)
    best_i = 0
    best_cost = None
    for i, row in enumerate(rows):
        cost = float(sum(ang_diff(row[j], target[j]) ** 2 for j in range(6)))
        if best_cost is None or cost < best_cost:
            best_cost = cost
            best_i = i
    return rows[best_i], int(rows.shape[0])


def reduce_pose_seeds(base_position, yaw_bin, joints, n_solutions, pose_id):
    """Collapse every IK branch to one seed per pose."""
    pose_id = np.asarray(pose_id)
    if pose_id.size == 0:
        return (
            np.zeros((0, 3), np.float32),
            np.zeros((0,), np.uint8),
            np.zeros((0, 6), np.float64),
            np.zeros((0,), np.uint8),
        )
    cuts = np.flatnonzero(np.diff(pose_id)) + 1
    starts = np.concatenate(([0], cuts))
    ends = np.concatenate((cuts, [pose_id.size]))
    count = starts.size
    positions = np.empty((count, 3), np.float32)
    yaws = np.empty((count,), np.uint8)
    chosen = np.empty((count, 6), np.float64)
    counts = np.empty((count,), np.uint8)
    for i, (start, end) in enumerate(zip(starts, ends)):
        seed, n_found = select_branch(joints[start:end])
        positions[i] = base_position[start]
        yaws[i] = yaw_bin[start]
        chosen[i] = seed
        recorded = int(n_solutions[start])
        counts[i] = recorded if recorded else n_found
    return positions, yaws, chosen, counts


def usable(status, occupied_count, opening_hops, index):
    id_, ia, iz, iyaw = index
    return (
        int(status[id_, ia, iz, iyaw]) >= MARGINAL
        and int(occupied_count[id_, ia, iz, iyaw]) == 0
        and int(opening_hops[id_, ia, iz, iyaw]) != 0xFFFF
    )
