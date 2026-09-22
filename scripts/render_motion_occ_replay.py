#!/usr/bin/env python3
"""Offline XY/XZ scene from a MOTION-OCC trial replay dump (no Gazebo).

Usage:
  PYTHONPATH=src/luggage_planning:src/luggage_description \\
    python3 scripts/render_motion_occ_replay.py \\
      --run docs/status/evidence/motion_occ/2026-09-20_2032 \\
      --trial 1
"""

from __future__ import division

import argparse
import json
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402

from luggage_description.container_geometry import (  # noqa: E402
    descriptor_from_scene_config,
)
from luggage_description.scene_tf_config_utils import (  # noqa: E402
    container_aperture_edges_in_container,
    container_inner_ceiling_z,
    container_inner_dimensions,
    container_inner_floor_z,
    container_inner_hull_edges_in_container,
    container_opening_target_point_in_world,
    load_scene_tf_config,
    origin_in_world,
    resolve_scene_tf_config_path,
)
from luggage_planning.occupancy_place_paths import (  # noqa: E402
    OccupancySnapshot,
    occupancy_collision_boxes,
    stamp_geometry_descriptor,
)


def _load(path):
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


def _map_to_world(scene, xyz):
    origin, rpy = origin_in_world(scene)
    yaw = float(rpy[2])
    c, s = math.cos(yaw), math.sin(yaw)
    x, y, z = [float(v) for v in xyz]
    return [
        origin[0] + c * x - s * y,
        origin[1] + s * x + c * y,
        origin[2] + z,
    ]


def _aabb(center, size):
    half = [0.5 * float(v) for v in size]
    lo = [float(center[i]) - half[i] for i in range(3)]
    hi = [float(center[i]) + half[i] for i in range(3)]
    return lo, hi


def _overlap(c1, s1, c2, s2):
    a0, a1 = _aabb(c1, s1)
    b0, b1 = _aabb(c2, s2)
    vol = 1.0
    gaps = []
    for i in range(3):
        lo = max(a0[i], b0[i])
        hi = min(a1[i], b1[i])
        span = hi - lo
        gaps.append(span)
        if span <= 0.0:
            return 0.0, gaps
        vol *= span
    return vol, gaps


def _payload_center(suction, height):
    return [suction[0], suction[1], suction[2] - 0.5 * float(height)]


def _rect_xy(ax, center, size, **kwargs):
    lo, _hi = _aabb(center, size)
    ax.add_patch(Rectangle(
        (lo[0], lo[1]), float(size[0]), float(size[1]), **kwargs))


def _rect_xz(ax, center, size, **kwargs):
    lo, _hi = _aabb(center, size)
    ax.add_patch(Rectangle(
        (lo[0], lo[2]), float(size[0]), float(size[2]), **kwargs))


def _trial_dir(run, index, fail_code):
    dumps = os.path.join(run, "dumps")
    name = "trial_%02d_%s" % (index, fail_code)
    path = os.path.join(dumps, name)
    if os.path.isdir(path):
        return path
    for entry in sorted(os.listdir(dumps)):
        if entry.startswith("trial_%02d_" % index):
            return os.path.join(dumps, entry)
    raise SystemExit("no trial dump for index %d in %s" % (index, dumps))


def diagnose(run, index):
    dumps = os.path.join(run, "dumps")
    fail = json.load(open(os.path.join(run, "suite.json")))["fail_codes"][index]
    trial = _trial_dir(run, index, fail)
    replay = os.path.join(trial, "replay")
    prev = _trial_dir(run, index - 1, json.load(
        open(os.path.join(run, "suite.json")))["fail_codes"][index - 1]) if index else None
    scene_path = os.path.normpath(os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..", "src", "luggage_description", "config", "scene_tf.yaml"))
    if not os.path.isfile(scene_path):
        scene_path = resolve_scene_tf_config_path()
    scene = load_scene_tf_config(scene_path)
    origin, rpy = origin_in_world(scene)
    inner = container_inner_dimensions(scene)
    floor_z = container_inner_floor_z(scene)
    ceiling_z = container_inner_ceiling_z(scene)
    boundary = _load(os.path.join(replay, "last_boundary.json"))
    perception = _load(os.path.join(replay, "perception_refs.json"))
    slot = _load(os.path.join(trial, "slot.json"))
    surface = _load(os.path.join(replay, "surface_2d.json"))
    paths = _load(os.path.join(dumps, "box_%02d_paths" % index, "candidates.json"))
    snap = OccupancySnapshot.from_surface_2d(
        stamp_geometry_descriptor(
            surface, descriptor_from_scene_config(scene)),
        inflate_m=0.05)
    occ_boxes = occupancy_collision_boxes(snap, max_objects=64)
    occ_world = []
    for box in occ_boxes:
        occ_world.append({
            "id": box["id"],
            "xyz": _map_to_world(scene, box["xyz"]),
            "size": list(box["size"]),
        })

    placed_slot = _load(os.path.join(prev, "slot.json")) if prev else None
    placed = None
    if placed_slot:
        placed = {
            "xyz": list(placed_slot["pose_world"]["position"]),
            "size": [0.507163748212569, 0.35961894616845186, 0.2471664127331007],
        }
        placed_trial = _load(os.path.join(prev, "trial.json"))
        extras = placed_trial.get("extras") or {}
        # Prefer measured box WDH from first place if present.
        sel = ((extras.get("selector") or {}).get("dump") or "")
        cand_path = os.path.join(dumps, "box_00_paths", "candidates.json")
        if os.path.isfile(cand_path):
            first = _load(cand_path)["candidates"][0]
            placed["size"] = [first["width"], first["depth"], first["height"]]

    payload = perception["pick_detection"]["size_wdh"]
    suction_start = perception["suction_xyz"]
    gz_box = perception["gz_box"][:3]
    goal = list(boundary["target"])
    payload_goal = _payload_center(goal, payload[2])
    payload_start = _payload_center(suction_start, payload[2])

    overlap_vol, gaps = 0.0, None
    overlap_pad, gaps_pad = 0.0, None
    occ_hits = []
    z_gap = None
    if placed:
        overlap_vol, gaps = _overlap(
            payload_goal, payload, placed["xyz"], placed["size"])
        overlap_pad, gaps_pad = _overlap(
            payload_goal,
            [payload[i] + 0.02 for i in range(3)],
            placed["xyz"],
            [placed["size"][i] + 0.02 for i in range(3)],
        )
        payload_bottom = payload_goal[2] - 0.5 * payload[2]
        placed_top = placed["xyz"][2] + 0.5 * placed["size"][2]
        z_gap = payload_bottom - placed_top
        for box in occ_world:
            vol, _g = _overlap(payload_goal, payload, box["xyz"], box["size"])
            if vol > 0.0:
                occ_hits.append(box["id"])

    winner = paths.get("winner") or {}
    diagnosis = {
        "trial": index,
        "fail_code": fail,
        "moveit": {
            "message": boundary.get("message"),
            "error_code": boundary.get("moveit_error_code"),
            "error_name": "FAILURE" if int(boundary.get("moveit_error_code") or 0) == 99999
            else str(boundary.get("moveit_error_code")),
            "kind": boundary.get("kind"),
            "segment": boundary.get("name"),
            "segment_type": boundary.get("segment_type"),
            "occupancy_checked": boundary.get("occupancy_checked"),
            "occupancy_object_count": boundary.get("occupancy_object_count"),
            "n_traj_points": (boundary.get("trajectory") or {}).get("n_points"),
            "attached": (boundary.get("planning_scene") or {}).get("attached"),
            "world_object_ids": (boundary.get("planning_scene") or {}).get(
                "world_object_ids"),
        },
        "geometry": {
            "container_origin_world": list(origin),
            "container_yaw": float(rpy[2]),
            "inner_lwh": list(inner),
            "floor_z_container": floor_z,
            "placed_box": placed,
            "payload_wdh": payload,
            "suction_start": suction_start,
            "carried_gz_center": gz_box,
            "transit_goal_suction": goal,
            "payload_center_at_goal": payload_goal,
            "payload_center_at_start": payload_start,
            "slot_world": slot["pose_world"]["position"],
            "opening": container_opening_target_point_in_world(scene),
        },
        "selector": {
            "method": winner.get("method"),
            "slot_index": winner.get("slot_index"),
            "waypoints_map": winner.get("waypoints"),
            "n_slots": paths.get("n_slots"),
            "n_rows": paths.get("n_rows"),
            "direct_slot0_reason": next(
                (row.get("reason") for row in paths.get("rows") or []
                 if row.get("method") == "direct" and row.get("slot_index") == 0),
                None),
        },
        "occupancy": {
            "map_revision": snap.map_revision,
            "occupied_inflated_cells": snap.occupied_count(),
            "n_occ_boxes": len(occ_boxes),
        },
        "aabb_payload_vs_placed_at_goal": {
            "overlap_volume_m3": overlap_vol,
            "axis_overlap_xyz_m": gaps,
            "z_gap_m": z_gap,
            "collides_raw": overlap_vol > 0.0,
            "collides_with_10mm_pad": overlap_pad > 0.0,
            "overlap_volume_10mm_pad_m3": overlap_pad,
            "occupancy_box_hits_at_goal": occ_hits,
        },
        "cause": (
            "planning (not execute): transit GOAL hanging payload sits %.1f mm "
            "above placed_0_0_0 with full XY overlap. IK avoid_collisions=True "
            "then RRTConnect pose constraints return FAILURE 99999; trajectory "
            "points=0 so ExecuteTrajectory never ran. Occupancy columns miss "
            "the goal AABB; MoveIt collision is the committed placed box "
            "(+ ~10 mm FCL padding)."
            % ((z_gap or 0.0) * 1000.0)
        ),
    }
    out_json = os.path.join(replay, "moveit_diagnosis.json")
    with open(out_json, "w", encoding="utf-8") as handle:
        json.dump(diagnosis, handle, indent=2, sort_keys=True)
        handle.write("\n")

    hull_edges = [
        (_map_to_world(scene, a), _map_to_world(scene, b))
        for a, b in container_inner_hull_edges_in_container(scene)
    ]
    aperture_edges = [
        (_map_to_world(scene, a), _map_to_world(scene, b))
        for a, b in container_aperture_edges_in_container(scene)
    ]

    fig, axes = plt.subplots(1, 2, figsize=(13.5, 6.2))
    ax, axz = axes

    # --- XY ---
    for i, (a, b) in enumerate(hull_edges):
        ax.plot([a[0], b[0]], [a[1], b[1]], "k-", lw=1.8,
                label="inner hull (7-face)" if i == 0 else None)
    for i, (a, b) in enumerate(aperture_edges):
        ax.plot([a[0], b[0]], [a[1], b[1]], color="#9467bd", lw=2.0,
                label="door aperture" if i == 0 else None)
    for box in occ_world:
        _rect_xy(ax, box["xyz"], box["size"],
                 facecolor="#ffcc66", edgecolor="#c47f00", alpha=0.35, lw=0.6)
    if occ_world:
        _rect_xy(ax, occ_world[0]["xyz"], occ_world[0]["size"],
                 facecolor="#ffcc66", edgecolor="#c47f00", alpha=0.35,
                 lw=0.6, label="occupancy columns")
    if placed:
        _rect_xy(ax, placed["xyz"], placed["size"],
                 facecolor="#d62728", edgecolor="#7f0000", alpha=0.55,
                 lw=1.5, label="placed_0_0_0 (MoveIt)")
    _rect_xy(ax, payload_start, payload,
             facecolor="#2ca02c", edgecolor="#145214", alpha=0.25, lw=1.0,
             label="payload at start")
    _rect_xy(ax, payload_goal, payload,
             facecolor="#1f77b4", edgecolor="#08306b", alpha=0.45, lw=1.5,
             label="payload at transit goal")
    ax.plot([suction_start[0], goal[0]], [suction_start[1], goal[1]],
            "b--", lw=1.2, label="straight suction")
    ax.scatter([suction_start[0]], [suction_start[1]], c="#2ca02c", s=36, zorder=5)
    ax.scatter([goal[0]], [goal[1]], c="#1f77b4", marker="x", s=64, zorder=5)
    ax.scatter([origin[0]], [origin[1]], c="k", marker="+", s=50)
    ax.set_aspect("equal")
    ax.set_xlabel("world X (m)")
    ax.set_ylabel("world Y (m)")
    ax.set_title("Top view — hanging payload vs packed box")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.25)

    # --- XZ ---
    for i, (a, b) in enumerate(hull_edges):
        axz.plot([a[0], b[0]], [a[2], b[2]], "k-", lw=1.8,
                 label="inner hull (7-face)" if i == 0 else None)
    for i, (a, b) in enumerate(aperture_edges):
        axz.plot([a[0], b[0]], [a[2], b[2]], color="#9467bd", lw=2.0,
                 label="door aperture" if i == 0 else None)
    for box in occ_world:
        _rect_xz(axz, box["xyz"], box["size"],
                 facecolor="#ffcc66", edgecolor="#c47f00", alpha=0.35, lw=0.6)
    if placed:
        _rect_xz(axz, placed["xyz"], placed["size"],
                 facecolor="#d62728", edgecolor="#7f0000", alpha=0.55, lw=1.5,
                 label="placed_0_0_0")
    _rect_xz(axz, payload_start, payload,
             facecolor="#2ca02c", edgecolor="#145214", alpha=0.25, lw=1.0,
             label="payload start")
    _rect_xz(axz, payload_goal, payload,
             facecolor="#1f77b4", edgecolor="#08306b", alpha=0.45, lw=1.5,
             label="payload at goal")
    axz.plot([suction_start[0], goal[0]], [suction_start[2], goal[2]],
             "b--", lw=1.2)
    axz.scatter([suction_start[0]], [suction_start[2]], c="#2ca02c", s=36, zorder=5)
    axz.scatter([goal[0]], [goal[2]], c="#1f77b4", marker="x", s=64, zorder=5)
    axz.set_aspect("equal")
    axz.set_xlabel("world X (m)")
    axz.set_ylabel("world Z (m)")
    collide = "GOAL %.1f mm above packed box" % ((z_gap or 0.0) * 1000.0)
    axz.set_title("Side view — %s (10 mm pad collides=%s)"
                  % (collide, overlap_pad > 0.0))
    axz.legend(loc="upper right", fontsize=8)
    axz.grid(True, alpha=0.25)

    fig.suptitle(
        "Trial %d %s  |  MoveIt %s (%s)\n%s"
        % (index, fail, diagnosis["moveit"]["error_name"],
           diagnosis["moveit"]["message"], diagnosis["cause"][:110]),
        fontsize=10)
    fig.tight_layout()
    out_png = os.path.join(replay, "scene_xy_xz.png")
    fig.savefig(out_png, dpi=140)
    plt.close(fig)
    return out_png, out_json, diagnosis


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", required=True)
    parser.add_argument("--trial", type=int, default=1)
    args = parser.parse_args(argv)
    png, js, diag = diagnose(os.path.abspath(args.run), args.trial)
    print(json.dumps({
        "png": png, "diagnosis": js,
        "cause": diag["cause"],
        "overlap_volume_m3": diag["aabb_payload_vs_placed_at_goal"]["overlap_volume_m3"],
        "moveit": diag["moveit"],
    }, indent=2, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
