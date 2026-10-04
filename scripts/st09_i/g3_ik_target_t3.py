#!/usr/bin/env python3
"""Offline T3 dump: atlas seeds vs logged place TCP targets (no ROS graph).

For every NO_FEASIBLE_ROUTE row in a Run's ``box_NN_paths/candidates.json``
this replays the trajectory selector's atlas prior on the exact logged
``target_world`` poses: container-frame query pose, atlas status and the
contact seeds it returns, FK of each seed, the seed-FK-to-target gap, the
re-refined residual from each seed, and the base-to-target distance.

Usage:
  g3_ik_target_t3.py --run DIR --scene SCENE_TF_YAML \
      --atlas-config ATLAS_RUNTIME_YAML --out OUT.json
"""
import argparse
import glob
import json
import math
import os

from luggage_description.scene_tf_config_utils import (
    load_scene_tf_config,
    robot_base_in_world,
    world_transform,
)
from luggage_planning.atlas_runtime import (
    load_atlas_runtime_for_scene,
    load_runtime_atlas,
)
from luggage_planning.offline_ik import load_urdf, refine_to_pose
from luggage_planning.reachable_graph import make_offline_fk
from st09_contracts.attachment import compose_transform, invert_transform


def _quat_yaw(q):
    return math.atan2(2.0 * (q[3] * q[2] + q[0] * q[1]),
                      1.0 - 2.0 * (q[1] ** 2 + q[2] ** 2))


def _dist(a, b):
    return math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)))


def _rpy_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def _pose_xyz_quat(pose):
    """Scene utils return (xyz, rpy); offline FK returns (xyz, quat)."""
    xyz, rot = tuple(pose[0]), tuple(pose[1])
    return xyz, (_rpy_quat(*rot) if len(rot) == 3 else rot)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", required=True)
    parser.add_argument("--scene", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--atlas-config", required=True,
                        help="the runtime yaml the live run passed as "
                             "atlas_runtime_config (no scene-hash fallback)")
    args = parser.parse_args()

    scene = load_scene_tf_config(args.scene)
    container_xyz, container_quat = _pose_xyz_quat(
        world_transform(scene, "container_link"))
    base_xyz, _base_quat = _pose_xyz_quat(robot_base_in_world(scene))
    inv_xyz, inv_quat = invert_transform(container_xyz, container_quat)

    cfg = dict(load_atlas_runtime_for_scene(args.scene, args.atlas_config))
    cfg["hash_policy"] = "off"
    atlas, diag = load_runtime_atlas(cfg)
    urdf = load_urdf(scene_tf_config=args.scene)
    tcp_path = urdf.path_to("suction_contact_frame")
    fk = make_offline_fk(urdf, tcp_path)

    report = {
        "run": args.run,
        "scene": args.scene,
        "container_world": [list(container_xyz), list(container_quat)],
        "base_world": list(base_xyz),
        "atlas": {"npz": diag.get("npz"), "meta": diag.get("meta"),
                  "reason": diag.get("reason")},
        "boxes": [],
    }
    for path in sorted(glob.glob(os.path.join(
            args.run, "dumps", "box_*_paths", "candidates.json"))):
        doc = json.load(open(path))
        box = {"file": os.path.relpath(path, args.run), "targets": []}
        for row in doc.get("rows", []):
            if row.get("reason") != "NO_FEASIBLE_ROUTE":
                continue
            seen = set()
            for refine_row in row.get("ik_refine", []):
                position, quat = refine_row["target_world"]
                key = tuple(round(v, 6) for v in position + quat)
                if key in seen:
                    continue
                seen.add(key)
                c_pos, c_quat = compose_transform(inv_xyz, inv_quat,
                                                  position, quat)
                yaw = _quat_yaw(c_quat)
                result = atlas.query(float(c_pos[0]), float(c_pos[1]),
                                     float(c_pos[2]), yaw)
                seeds = [tuple(float(v) for v in s) for s in
                         (getattr(result, "contact_seeds", None) or ())]
                seed_rows = []
                for seed in seeds:
                    seed_xyz, seed_quat = _pose_xyz_quat(fk(seed))
                    q, info = refine_to_pose(urdf, tcp_path, position, quat,
                                             list(seed), pos_tol=2e-3,
                                             rot_tol=2e-2)
                    info = info if isinstance(info, dict) else {}
                    seed_rows.append({
                        "seed_q": [round(v, 4) for v in seed],
                        "seed_fk_world": [round(v, 4) for v in seed_xyz],
                        "seed_fk_container": [round(v, 4) for v in
                                              compose_transform(
                                                  inv_xyz, inv_quat,
                                                  seed_xyz, seed_quat)[0]],
                        "seed_fk_to_target_m": round(_dist(seed_xyz,
                                                           position), 4),
                        "refined_ok": q is not None,
                        "refined_pos_err_m": info.get("pos_err_m"),
                        "refined_rot_err_rad": info.get("rot_err_rad"),
                    })
                box["targets"].append({
                    "slot_index": row.get("slot_index"),
                    "target_world": [round(v, 4) for v in position],
                    "target_yaw_world_deg": round(math.degrees(
                        _quat_yaw(quat)), 1),
                    "target_container": [round(v, 4) for v in c_pos],
                    "target_yaw_container_deg": round(math.degrees(yaw), 1),
                    "base_to_target_m": round(_dist(base_xyz, position), 4),
                    "atlas_status": int(result.status),
                    "atlas_fields": {k: v for k, v in result._asdict().items()
                                     if k not in ("contact_seeds",
                                                  "transit_seeds")}
                    if hasattr(result, "_asdict") else {},
                    "seeds": seed_rows,
                })
        report["boxes"].append(box)
    with open(args.out, "w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=1, sort_keys=True, default=str)


if __name__ == "__main__":
    main()
