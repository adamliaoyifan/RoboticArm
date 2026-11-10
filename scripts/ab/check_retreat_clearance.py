#!/usr/bin/env python3
"""Offline item-3 check: the FIXED retreat trajectory (recorded joints)
is applicable under all three arms.

FK the recorded retreat-window joints (obs_rel_07 tool_stream) with the
production offline IK model (world-base patched URDF), transform TCP
into container_link, and require the tool to stay clear of the BASELINE
cavity walls while inside the container and to pass through the aperture
when exiting. A0 is wider than the baseline, so baseline-clear implies
A0/A'/B-clear; a baseline violation flags the pair comparison invalid
per the frozen contract.

Also re-verifies the full release input: box 3-D extents at the release
height and attitude (yaw from the recorded quaternion) under the
baseline cavity.
"""
import argparse
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from design_baseline import load_baseline, inside_cavity, chamfer_y  # noqa

_SCENE_TF = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))),
    "src", "luggage_description", "config", "scene_tf_deploy_update.yaml")


def box_extents_valid(b, center, half, yaw):
    """All 8 corners of the yawed box inside the baseline cavity."""
    cy, sy = math.cos(yaw), math.sin(yaw)
    corners = []
    for sx in (-1, 1):
        for sy_ in (-1, 1):
            for sz in (-1, 1):
                dx = sx * half[0]
                dy = sy_ * half[1]
                corners.append((
                    center[0] + cy * dx - sy * dy,
                    center[1] + sy * dx + cy * dy,
                    center[2] + sz * half[2]))
    return all(inside_cavity(b, c) for c in corners), corners


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--tool-stream",
                        default="/home/adamliao/work/elfin_humble_ws/docs/"
                        "status/evidence/master_pack_recovery/"
                        "2026-10-14_g3/obs_rel_07/tool_stream.jsonl")
    parser.add_argument("--release-t-sim", type=float, default=127.32)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    b = load_baseline(_SCENE_TF)

    # ---- Release input validity (full box, recorded attitude) --------
    spawn = [-1.98001, 0.460003, 0.691353]
    yaw = -math.pi / 2.0  # from the recorded quat (z=-0.707)
    half = (0.2585, 0.164, 0.1045)  # inch20 wdh rotated into place
    world_to_container = np.array([[-1.0, 0.0, 0.0],
                                   [0.0, -1.0, 0.0],
                                   [0.0, 0.0, 1.0]])
    center_c = world_to_container.dot(
        np.array(spawn) - np.array([-1.5, 0.6, 0.0]))
    ok, corners = box_extents_valid(b, center_c, half, yaw)
    release = {
        "center_container": center_c.tolist(),
        "half_extents": list(half), "yaw": yaw,
        "corners_valid": bool(ok),
    }

    # ---- Retreat clearance under the baseline ------------------------
    from luggage_planning.offline_ik import load_urdf, JOINTS
    urdf = load_urdf(scene_tf_config=_SCENE_TF)
    ik_link = "suction_contact_frame"
    # Full path from the URDF root keeps the world_base patch, so FK is
    # in the WORLD frame (slicing at the arm joint would give base-frame
    # FK like the atlas builder uses).
    tcp_path = urdf.path_to(ik_link)

    samples = []
    for line in open(args.tool_stream):
        d = json.loads(line)
        if d["kind"] != "joint_states":
            continue
        t = d["t_sim_ns"] / 1e9
        if not (args.release_t_sim - 1.0 <= t <= args.release_t_sim + 45.0):
            continue
        pos = d["position"]
        names = d.get("name") or []
        if names and set(JOINTS) <= set(names):
            q = [pos[names.index(j)] for j in JOINTS]
        else:
            q = [float(v) for v in pos[:6]]
        samples.append((t, q))

    worst = {"in_cavity_min_wall_mm": None,
             "aperture_transit_ok": None}
    in_cavity_dists = []
    transited = False
    aperture_ok = True
    for t, q in samples:
        tf = urdf.tcp(list(q), tcp_path)
        tcp_w = np.array([tf[0][3], tf[1][3], tf[2][3]])
        tcp_c = world_to_container.dot(tcp_w - np.array([-1.5, 0.6, 0.0]))
        x, y, z = tcp_c
        if b["x_lo"] < x < b["x_hi"]:
            # inside the container footprint: distance to every wall
            d = min(x - b["x_lo"], b["x_hi"] - x,
                    y - b["y_lo"], chamfer_y(b, z) - y,
                    z - b["floor_z"], b["ceiling_z"] - z)
            in_cavity_dists.append(d)
        elif b["o_x_lo"] - 0.05 < x < b["x_lo"]:
            # in the -X wall band: must be inside the aperture rect
            transited = True
            if not (b["aperture"]["y_lo"] - 0.02 < y
                    < b["aperture"]["y_hi"] + 0.02
                    and b["aperture"]["z_lo"] - 0.02 < z
                    < b["aperture"]["z_hi"] + 0.02):
                aperture_ok = False
    if in_cavity_dists:
        worst["in_cavity_min_wall_mm"] = round(
            min(in_cavity_dists) * 1000, 2)
    worst["aperture_transit_ok"] = bool(transited and aperture_ok)
    retreat = {
        "samples": len(samples),
        "clearance": worst,
        "pass": bool(worst["in_cavity_min_wall_mm"] is not None
                     and worst["in_cavity_min_wall_mm"] > 0.0
                     and worst["aperture_transit_ok"]),
        "note": "positive wall distance inside the container and a clean "
                "aperture transit mean the fixed retreat applies under "
                "baseline (hence A'/B); A0 is strictly wider.",
    }

    report = {
        "baseline": b["source"],
        "release_input": release,
        "retreat": retreat,
        "pass": bool(release["corners_valid"] and retreat["pass"]),
    }
    with open(args.out, "w") as handle:
        json.dump(report, handle, indent=2)
    print(json.dumps(report, indent=1))
    raise SystemExit(0 if report["pass"] else 1)


if __name__ == "__main__":
    main()
