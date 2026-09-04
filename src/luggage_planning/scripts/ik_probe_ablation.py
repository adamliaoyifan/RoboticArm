#!/usr/bin/env python3
"""IK probe ablation with payload (box) collision (E8 payload extension).

Extends E8 to answer: does the floor=0 conclusion hold when the box is
attached (real placement), and how does it vary by box size?

For each box size (carryon/standard/large) + empty-load baseline:
  - attach the box to suction_contact_frame (offset [0,0,h/2], box hangs below
    contact -- real placement geometry; box IS in the collision scene)
  - sample the floor group at z=h (box-bottom on container floor => contact at
    box top = h, the box-bottom reverse-computation per the placement design)
  - run the 6 ablation configs (each isolates seed/collision/transit/tolerance)
  - detach

Output: per box-size x config contact% table. The key comparison is
config 1 (collision on, with box) = real placement reachability, vs
config 3 (collision off) = kinematic upper bound.

Usage (atlas_stack.launch running):
    rosrun luggage_planning ik_probe_ablation.py \
        _scene_tf_config:=$(rospack find luggage_description)/config/scene_tf.yaml.example \
        _output_yaml:=$(rospack find luggage_planning)/data/reachability_atlas/ik_probe_ablation_payload.yaml
"""
from __future__ import division

import math
import os
import random
import sys

import numpy as np
import rospkg
import rospy
import yaml

PKG_SCRIPTS = os.path.join(rospkg.RosPack().get_path("luggage_planning"), "scripts")
DESC_SCRIPTS = os.path.join(rospkg.RosPack().get_path("luggage_description"), "scripts")
for _p in (PKG_SCRIPTS, DESC_SCRIPTS):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from reachability_atlas_builder import ReachabilityAtlasBuilder  # noqa: E402

JOINT_LIMITS = [
    (-6.28, 6.28), (-3.32, 0.17), (-2.93, 2.93),
    (-6.28, 6.28), (-6.28, 6.28), (-6.28, 6.28),
]
TOL_15DEG = math.radians(15.0)
# Catalog box sizes [l, w, h] (m) -- match box_catalog.yaml.example.
BOX_SIZES = [
    ("carryon",  [0.55, 0.40, 0.25]),
    ("standard", [0.70, 0.45, 0.28]),
    ("large",    [0.80, 0.50, 0.32]),
]


def _random_seeds(n, rng):
    return [tuple(rng.uniform(lo, hi) for (lo, hi) in JOINT_LIMITS) for _ in range(n)]


def _group_cells(builder, group, box_h=None):
    """Cells for the named group. If box_h given, floor samples at z=h
    (box-bottom on container floor => contact at box top)."""
    res = builder._resolution
    nx = int(math.ceil(builder._inner_l / res))
    ny = int(math.ceil(builder._inner_w / res))
    iy_samples = [0, ny // 2, ny - 1] if ny >= 3 else list(range(ny))
    yaw_idx = 0

    def cell_at(ix, iz, iy):
        x = -builder._inner_l * 0.5 + (ix + 0.5) * res
        y = -builder._inner_w * 0.5 + (iy + 0.5) * res
        z = builder._z_min_offset + (iz + 0.5) * res
        return (x, y, z, builder._yaw_bins[yaw_idx])

    def cell_at_z(ix, iy, z):
        x = -builder._inner_l * 0.5 + (ix + 0.5) * res
        y = -builder._inner_w * 0.5 + (iy + 0.5) * res
        return (x, y, z, builder._yaw_bins[yaw_idx])

    if group == "A_opening":
        return [cell_at(0, iz, iy) for iy in iy_samples for iz in range(3, 9)]
    if group == "B_floor":
        ix_samples = [1, 3, 5, 7, 9]
        if box_h is not None:
            # Box-bottom on floor (z=0) => contact at box top z=h.
            return [cell_at_z(ix, iy, box_h) for ix in ix_samples for iy in iy_samples]
        return [cell_at(ix, iz, iy) for ix in ix_samples for iy in iy_samples
                for iz in range(0, 3)]
    raise ValueError("unknown group %s" % group)


def _test_cell(builder, cell, config, rng):
    x, y, z, yaw = cell
    contact_pos, contact_quat = builder._construct_base_pose(x, y, z, yaw)
    transit_pos, transit_quat = builder._construct_base_pose(
        x, y, z + builder._transit_clearance, yaw)
    seeds = builder._fixed_seeds if config["seeds"] == "observe" else _random_seeds(32, rng)
    transit_ok = False
    contact_ok = False
    for seed in seeds:
        t_sol = None
        if config["transit"]:
            t_sol = builder._solve_ik_transit_with_tolerance(
                transit_pos, transit_quat, seed)
            if t_sol is None:
                continue
            transit_ok = True
        c_seed = t_sol if t_sol is not None else seed
        c_sol = builder._solve_ik(contact_pos, contact_quat, c_seed)
        if c_sol is not None:
            contact_ok = True
            break
    return transit_ok, contact_ok


CONFIGS = [
    {"name": "1_baseline",    "seeds": "observe", "collision": True,  "transit": True,  "tol": 0.0},
    {"name": "2_+seeds",      "seeds": "random",  "collision": True,  "transit": True,  "tol": 0.0},
    {"name": "3_-collision",  "seeds": "observe", "collision": False, "transit": True,  "tol": 0.0},
    {"name": "4_contact-only","seeds": "observe", "collision": True,  "transit": False, "tol": 0.0},
    {"name": "5_+tol",        "seeds": "observe", "collision": True,  "transit": True,  "tol": TOL_15DEG},
    {"name": "6_all-relaxed", "seeds": "random",  "collision": False, "transit": False, "tol": TOL_15DEG},
]


def _attach_box(builder, size):
    h = size[2]
    builder._payload_enabled = True
    builder._payload_size = [float(v) for v in size]
    builder._payload_shape = "box"
    builder._payload_attach_link = builder._ik_link
    builder._payload_offset = [0.0, 0.0, h * 0.5]  # box hangs below contact
    builder._payload_touch_links = ["suction_panel"]
    builder._payload_id = "ablation_payload"
    builder._attach_payload()
    rospy.sleep(1.0)


def _detach_box(builder):
    try:
        builder._mc_scene.remove_attached_object(builder._payload_id)
        rospy.sleep(0.5)
    except Exception as exc:
        rospy.logwarn("detach failed: %s", exc)
    builder._payload_enabled = False


def _run_group(builder, group, box_h, rng):
    cells = _group_cells(builder, group, box_h)
    out = {"n_cells": len(cells), "configs": {}}
    for cfg in CONFIGS:
        builder._avoid_collisions = cfg["collision"]
        builder._tool_down_tolerance = cfg["tol"]
        t_ok = c_ok = 0
        for cell in cells:
            t, c = _test_cell(builder, cell, cfg, rng)
            t_ok += int(t); c_ok += int(c)
        n = len(cells)
        out["configs"][cfg["name"]] = {
            "transit_ok": t_ok, "contact_ok": c_ok,
            "transit_pct": round(100.0 * t_ok / n, 1),
            "contact_pct": round(100.0 * c_ok / n, 1),
        }
        rospy.loginfo("    %-16s contact=%2d/%d (%.0f%%)",
                      cfg["name"], c_ok, n, 100.0 * c_ok / n)
    return out


def main():
    rospy.init_node("ik_probe_ablation", anonymous=True)
    scene_tf_path = rospy.get_param(
        "~scene_tf_config",
        os.path.join(rospkg.RosPack().get_path("luggage_description"),
                     "config", "scene_tf.yaml.example"))
    output_yaml = rospy.get_param(
        "~output_yaml",
        os.path.join(rospkg.RosPack().get_path("luggage_planning"),
                     "data", "reachability_atlas", "ik_probe_ablation_payload.yaml"))
    rng = random.Random(0x5EED)

    rospy.loginfo("Instantiating ReachabilityAtlasBuilder (waits for /compute_ik)...")
    builder = ReachabilityAtlasBuilder()
    try:
        from std_srvs.srv import Trigger
        sync = rospy.ServiceProxy("/scene_manager/sync_static_scene", Trigger)
        sync.wait_for_service(timeout=5.0)
        sync(); rospy.sleep(2.0)
    except Exception as exc:
        rospy.logwarn("scene sync failed: %s", exc)

    # Conditions: empty-load baseline + 3 box sizes.
    conditions = [("empty_load", None)] + [(name, size) for name, size in BOX_SIZES]
    results = {}
    for cond_name, size in conditions:
        box_h = size[2] if size else None
        rospy.loginfo("=== Condition: %s %s ===", cond_name,
                      "size=%s h=%.2f" % (size, box_h) if size else "(no box)")
        if size is not None:
            _attach_box(builder, size)
        results[cond_name] = {"box_size": size, "box_h": box_h, "groups": {}}
        for group in ("A_opening", "B_floor"):
            rospy.loginfo("  Group %s:", group)
            results[cond_name]["groups"][group] = _run_group(builder, group, box_h, rng)
        if size is not None:
            _detach_box(builder)

    os.makedirs(os.path.dirname(output_yaml), exist_ok=True)
    with open(output_yaml, "w") as f:
        yaml.safe_dump({"results": results, "scene_tf": scene_tf_path}, f,
                       default_flow_style=False, sort_keys=False)

    rospy.loginfo("=== Floor contact%% by box-size x config ===")
    rospy.loginfo("%-12s " + " ".join("%-12s" % c["name"] for c in CONFIGS), "condition")
    for cond_name in [c for c, _ in conditions]:
        floor = results[cond_name]["groups"]["B_floor"]["configs"]
        row = " ".join("%-12s" % ("%s%%" % floor[c["name"]]["contact_pct"]) for c in CONFIGS)
        rospy.loginfo("%-12s %s", cond_name, row)
    rospy.loginfo("Saved to %s", output_yaml)


if __name__ == "__main__":
    main()
