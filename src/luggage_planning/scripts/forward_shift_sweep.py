#!/usr/bin/env python3
"""Forward-shift × tool-down-tolerance × transit-clearance reachability sweep.

Sweeps base forward offsets (toward the container opening) crossed with
tool-down tolerance and transit clearance, builds a collision-aware atlas per
combo via ReachabilityAtlasBuilder.compute(), and records coverage stats
(total / floor-layer / deep-layer). Picks the best combo by absolute coverage
(floor-layer priority) -- NOT the buggy best-vs-baseline relative decision (E4).

Usage (inside the noetic container, with atlas_stack.launch running):
    rosrun luggage_planning forward_shift_sweep.py \
        _scene_tf_config:=$(rospack find luggage_description)/config/scene_tf.yaml.example \
        _output_yaml:=$(rospack find luggage_planning)/data/reachability_atlas/forward_sweep_results.yaml

See docs/status/experiment_log.md E1/E5 and the approved execution plan.
"""
from __future__ import division

import copy
import itertools
import math
import os
import sys
import tempfile

import numpy as np
import rospkg
import rospy
import yaml

PKG_SCRIPTS = os.path.join(rospkg.RosPack().get_path("luggage_planning"), "scripts")
DESC_SCRIPTS = os.path.join(rospkg.RosPack().get_path("luggage_description"), "scripts")
for _p in (PKG_SCRIPTS, DESC_SCRIPTS):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from layout_atlas import effective_scene_tf_xyz  # noqa: E402
from scene_tf_config_utils import (  # noqa: E402
    container_in_base_link, load_scene_tf_config,
)
from reachability_atlas_builder import (  # noqa: E402
    ReachabilityAtlasBuilder, REACHABLE, MARGINAL,
)


def _opening_near_distance(scene_config):
    """Distance from base_link origin to the nearest opening aperture corner.

    Used to verify that a forward base shift actually moves the opening closer
    (i.e. the sweep axis/sign is correct). Opening corners live in container_link
    frame; transform to base_link via container_in_base_link.
    """
    import numpy as np
    base_xyz, base_rpy = container_in_base_link(scene_config)
    # rotation matrix from rpy
    cr, sr = math.cos(base_rpy[0]), math.sin(base_rpy[0])
    cp, sp = math.cos(base_rpy[1]), math.sin(base_rpy[1])
    cy, sy = math.cos(base_rpy[2]), math.sin(base_rpy[2])
    R = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - sy * sr],
        [-sp, cp * sr, cp * cr],
    ])
    opening = scene_config.get("container", {}).get("opening", {})
    corners = opening.get("aperture", {}).get("corners", [])
    if not corners:
        return float("inf")
    dists = []
    for c in corners:
        c = np.array(c, dtype=float)
        base_pt = np.array(base_xyz, dtype=float) + R.dot(c)
        dists.append(float(np.linalg.norm(base_pt)))
    return min(dists)


def _write_eff_yaml(base_config, dx, axis, sign):
    """Write a scene_tf with the container shifted opposite to the base move."""
    delta = [0.0, 0.0, 0.0]
    delta[axis] = sign * dx
    eff = effective_scene_tf_xyz(base_config, *delta)
    fd, path = tempfile.mkstemp(suffix=".yaml", prefix="fwd_eff_")
    with os.fdopen(fd, "w") as f:
        yaml.safe_dump(eff, f, default_flow_style=False, sort_keys=False)
    return path


def _sync_scene(config_path):
    """Point scene_manager at config_path and sync the planning scene."""
    from std_srvs.srv import Trigger
    rospy.set_param("/scene_manager/scene_tf_config", config_path)
    try:
        sync = rospy.ServiceProxy("/scene_manager/sync_static_scene", Trigger)
        sync.wait_for_service(timeout=5.0)
        resp = sync()
        return bool(resp.success)
    except Exception as exc:
        rospy.logwarn("scene sync failed for %s: %s", config_path, exc)
        return False


def _layer_stats(status):
    """Floor-layer (iz=0,1,2) and deep-layer (ix=7,8,9) reachable counts."""
    nx, ny, nz, nyaw = status.shape
    floor = int(np.count_nonzero(status[:, :, 0:3, :] >= MARGINAL))
    deep = int(np.count_nonzero(status[7:10, :, :, :] >= MARGINAL)) if nx >= 10 else 0
    total_reach = int(np.count_nonzero(status >= MARGINAL))
    return floor, deep, total_reach


def main():
    rospy.init_node("forward_shift_sweep", anonymous=True)

    scene_tf_path = rospy.get_param(
        "~scene_tf_config",
        os.path.join(rospkg.RosPack().get_path("luggage_description"),
                     "config", "scene_tf.yaml.example"))
    base_config = load_scene_tf_config(scene_tf_path)

    # Sweep grid (defaults match the approved plan).
    forward_axis = {"x": 0, "y": 1, "z": 2}[rospy.get_param("~forward_axis", "x")]
    forward_sign = int(rospy.get_param("~forward_sign", 1))
    dx_list = [float(v) for v in rospy.get_param(
        "~dx_list", [0.0, 0.10, 0.20, 0.30, 0.385])]
    tol_list = [float(v) for v in rospy.get_param(
        "~tol_list", [0.0, 0.2618])]  # 0, 15°
    clearance_list = [float(v) for v in rospy.get_param(
        "~clearance_list", [0.30])]
    resolution = float(rospy.get_param("~resolution_xyz", 0.15))
    output_yaml = rospy.get_param(
        "~output_yaml",
        os.path.join(rospkg.RosPack().get_path("luggage_planning"),
                     "data", "reachability_atlas", "forward_sweep_results.yaml"))

    # Verify forward direction: opening should get closer as dx grows.
    d0 = _opening_near_distance(base_config)
    cfg_far = effective_scene_tf_xyz(
        base_config,
        *[sign * max(dx_list) if i == forward_axis else 0.0
          for i, sign in enumerate([forward_sign] * 3)])
    d_far = _opening_near_distance(cfg_far)
    rospy.loginfo("Opening near-distance: dx=0 -> %.3f m, dx=%.3f -> %.3f m",
                  d0, max(dx_list), d_far)
    if d_far >= d0:
        rospy.logwarn(
            "Forward shift INCREASED opening distance (%.3f -> %.3f); "
            "axis/sign may be wrong. Flip ~forward_sign or ~forward_axis.",
            d0, d_far)

    rospy.loginfo("Sweep: %d dx x %d tol x %d clearance = %d combos",
                  len(dx_list), len(tol_list), len(clearance_list),
                  len(dx_list) * len(tol_list) * len(clearance_list))

    results = []
    for dx, tol, clearance in itertools.product(dx_list, tol_list, clearance_list):
        rospy.loginfo("=== dx=%.3f tol=%.4f clearance=%.2f ===", dx, tol, clearance)
        eff_path = _write_eff_yaml(base_config, dx, forward_axis, forward_sign)
        try:
            if not _sync_scene(eff_path):
                rospy.logwarn("  skip (scene sync failed)")
                continue
            rospy.set_param("~scene_tf_config", eff_path)
            rospy.set_param("~tool_down_tolerance", tol)
            rospy.set_param("~transit_clearance", clearance)
            rospy.set_param("~resolution_xyz", resolution)
            rospy.set_param("~avoid_collisions", True)
            rospy.set_param("~output_dir", "")
            builder = ReachabilityAtlasBuilder()
            data, meta = builder.compute()
            status = data["status"]
            floor, deep, total = _layer_stats(status)
            rate = meta["stats"]["reachability_rate"]
            row = {
                "dx": dx, "tool_down_tolerance": tol, "transit_clearance": clearance,
                "reachability_rate": rate, "total_reachable": total,
                "floor_reachable": floor, "deep_reachable": deep,
            }
            results.append(row)
            rospy.loginfo(
                "  rate=%.4f total=%d floor=%d deep=%d", rate, total, floor, deep)
        except Exception as exc:
            rospy.logwarn("  combo failed: %s", exc)
        finally:
            try:
                os.unlink(eff_path)
            except OSError:
                pass
            rospy.set_param("~scene_tf_config", scene_tf_path)

    # Sort by floor coverage (primary) then total (secondary).
    results.sort(key=lambda r: (-r["floor_reachable"], -r["total_reachable"]))

    os.makedirs(os.path.dirname(output_yaml), exist_ok=True)
    with open(output_yaml, "w") as f:
        yaml.safe_dump({"results": results, "scene_tf": scene_tf_path}, f,
                       default_flow_style=False, sort_keys=False)

    rospy.loginfo("=== Results (sorted by floor then total reachable) ===")
    rospy.loginfo("%-7s %-8s %-10s %-8s %-7s %-7s %-7s",
                  "dx", "tol", "clearance", "rate", "total", "floor", "deep")
    for r in results:
        rospy.loginfo("%-7.3f %-8.4f %-10.2f %-8.4f %-7d %-7d %-7d",
                      r["dx"], r["tool_down_tolerance"], r["transit_clearance"],
                      r["reachability_rate"], r["total_reachable"],
                      r["floor_reachable"], r["deep_reachable"])
    rospy.loginfo("Best: dx=%.3f tol=%.4f clearance=%.2f floor=%d total=%d",
                  results[0]["dx"], results[0]["tool_down_tolerance"],
                  results[0]["transit_clearance"], results[0]["floor_reachable"],
                  results[0]["total_reachable"])
    rospy.loginfo("Saved %d results to %s", len(results), output_yaml)


if __name__ == "__main__":
    main()
