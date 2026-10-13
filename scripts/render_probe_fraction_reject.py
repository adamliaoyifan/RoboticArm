#!/usr/bin/env python3
"""Replay the rehearsal-52 second-box probe rejection onto the saved map.

The live dump kept path hashes and the last cartesian fraction, not the
polylines. This rebuilds the geometric carries on that same surface_2d and
draws them over the sensor cells the first box wrote.
"""

from __future__ import division

import argparse
import json
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

REPO = os.path.normpath(os.path.join(os.path.dirname(__file__), ".."))
for path in (
        os.path.join(REPO, "src", "luggage_packing"),
        os.path.join(REPO, "src", "luggage_planning"),
        os.path.join(REPO, "src", "luggage_description"),
        os.path.join(REPO, "src", "luggage_gazebo")):
    if path not in sys.path:
        sys.path.insert(0, path)

from luggage_description.scene_tf_config_utils import (  # noqa: E402
    _point_in_container_link,
    container_opening_normal_in_world,
    container_opening_target_point_in_world,
    load_scene_tf_config,
    resolve_scene_tf_config_path,
    xyz_world_to_base_link,
)
from luggage_packing.placement_solver import solve_placement  # noqa: E402
from luggage_planning.occupancy_place_paths import (  # noqa: E402
    OccupancySnapshot,
)
from luggage_planning.path_candidates import GeometricVariantGenerator  # noqa: E402
from luggage_planning.trajectory_selector import (  # noqa: E402
    SELECTOR_DEFAULTS,
    load_selector_config,
)


def _world_to_map(scene, xyz):
    base = xyz_world_to_base_link(scene, xyz)
    return _point_in_container_link(base, scene)


def _portal(scene):
    world = container_opening_target_point_in_world(scene)
    local = _world_to_map(scene, world)
    return (local[0], local[1], max(local[2] + 0.35, 0.80))


def _opening_tangent(scene):
    normal = container_opening_normal_in_world(scene)
    return (-float(normal[1]), float(normal[0]))


def _slots(surface, box):
    result = solve_placement(
        surface, box, allowed_yaws=[0.0, math.pi / 2.0],
        params={"top_n": 8, "keep_rejected": 0})
    slots = []
    center = surface.get("center_base") or [0.0, 0.0, 0.0]
    for index, cand in enumerate(result.get("candidates") or []):
        if not cand.get("feasible"):
            continue
        local = cand.get("center_container") or cand.get("center_local") or [
            0.0, 0.0, 0.0]
        tool_z = float(local[2]) + 0.5 * float(box[2])
        slots.append({
            "index": index,
            "score": float(cand.get("score") or 0.0),
            "target": (float(local[0]), float(local[1]), tool_z),
            "yaw": float(cand.get("box_yaw") or cand.get("yaw") or 0.0),
            "box_xy": (float(local[0]), float(local[1])),
        })
        if len(slots) >= 5:
            break
    return slots, result


def _rows(surface, slots, scene, box):
    cfg = dict(SELECTOR_DEFAULTS)
    cfg.update(load_selector_config() or {})
    snapshot = OccupancySnapshot.from_surface_2d(
        surface, expected_hash=str(surface.get("geometry_hash") or ""),
        expected_revision=int(surface.get("map_revision") or 0))
    gen = GeometricVariantGenerator(
        cfg.get("max_candidates_per_slot", 4),
        cfg.get("arm_radius_m", 0.08), cfg)
    portal = _portal(scene)
    tangent = _opening_tangent(scene)
    rows = []
    for slot in slots:
        rows.extend(gen.rows(
            snapshot, slot, portal, box, tangent, slot["yaw"]))
    rows.sort(key=lambda row: (
        0 if row.get("feasible") else 1,
        0 if row.get("method") == "corridor" else 1,
        float(row.get("cost") or 0.0)))
    return rows, portal


def _polyline_fraction(points, fraction):
    if len(points) < 2:
        return points[:1]
    lengths = [0.0]
    for a, b in zip(points, points[1:]):
        lengths.append(lengths[-1] + math.sqrt(
            sum((float(b[i]) - float(a[i])) ** 2 for i in range(3))))
    total = lengths[-1]
    if total < 1e-9:
        return points[:1]
    stop = float(fraction) * total
    kept = [points[0]]
    for index in range(1, len(points)):
        if lengths[index] <= stop:
            kept.append(points[index])
            continue
        span = lengths[index] - lengths[index - 1]
        t = 0.0 if span < 1e-9 else (stop - lengths[index - 1]) / span
        a, b = points[index - 1], points[index]
        kept.append([a[i] + t * (b[i] - a[i]) for i in range(3)])
        break
    return kept


def render(run, out):
    surface = json.load(open(os.path.join(
        run, "dumps", "trial_01_PLACE_PROBE_FRACTION_REJECT", "replay",
        "surface_2d.json")))
    probe = json.load(open(os.path.join(run, "dumps", "box_01_paths",
                                        "candidates.json")))["probe"]
    measured = json.load(open(os.path.join(
        run, "dumps", "trial_01_PLACE_PROBE_FRACTION_REJECT", "replay",
        "perception_refs.json")))
    box = [float(v) for v in measured["pick_detection"]["size_wdh"]]
    slot = json.load(open(os.path.join(run, "dumps", "trial_00_ok", "slot.json")))
    scene = load_scene_tf_config(resolve_scene_tf_config_path())
    placed = _world_to_map(scene, slot["pose_world"]["position"])
    slots, solved = _slots(surface, box)
    rows, portal = _rows(surface, slots, scene, box)
    fraction = float(probe["fraction"])
    os.makedirs(out, exist_ok=True)

    origin = surface.get("origin_local") or [-0.745, -0.985]
    res = float(surface.get("resolution") or 0.05)
    height = np.asarray(surface.get("height") or [], dtype=float)
    state = surface.get("state") or []
    conf = surface.get("confidence") or []
    show = np.zeros_like(height)
    for ix in range(height.shape[0]):
        for iy in range(height.shape[1]):
            if (state[ix][iy] == "occupied" and conf[ix][iy] == "sensor"):
                show[ix, iy] = height[ix, iy]
    show = np.ma.masked_where(show <= 0.0, show)

    fig, axes = plt.subplots(1, 2, figsize=(12.4, 6.4))
    ax = axes[0]
    extent = [
        origin[0], origin[0] + height.shape[0] * res,
        origin[1], origin[1] + height.shape[1] * res]
    image = ax.imshow(
        show.T, origin="lower", extent=extent, cmap="YlOrRd",
        vmin=0.05, vmax=0.30, interpolation="nearest", aspect="equal")
    fig.colorbar(image, ax=ax, label="sensor height above floor (m)")
    yaw = float(slot["pose_world"]["yaw"])
    # First box was yawed; draw its footprint in the map using the carryon size
    # recorded on the first integrate (0.55 x 0.40), rotated by the slot yaw.
    hw, hd = 0.275, 0.20
    c, s = math.cos(yaw), math.sin(yaw)
    corners = []
    for sx, sy in ((-hw, -hd), (hw, -hd), (hw, hd), (-hw, hd), (-hw, -hd)):
        corners.append((
            placed[0] + c * sx - s * sy,
            placed[1] + s * sx + c * sy))
    ax.plot([p[0] for p in corners], [p[1] for p in corners],
            color="#1b4f72", lw=2.0, label="box 0 footprint")
    colors = ["#c0392b", "#8e44ad", "#2980b9", "#16a085", "#d35400"]
    summary = []
    for index, row in enumerate(rows[:5]):
        pts = row.get("waypoints") or []
        color = colors[index % len(colors)]
        ax.plot([p[0] for p in pts], [p[1] for p in pts],
                color=color, lw=1.6, label="path %d %s" % (
                    index, row.get("method")))
        done = _polyline_fraction(pts, fraction)
        if len(done) >= 2:
            ax.plot([p[0] for p in done], [p[1] for p in done],
                    color=color, lw=3.2)
            ax.scatter([done[-1][0]], [done[-1][1]], color=color, s=28, zorder=4)
        summary.append({
            "method": row.get("method"),
            "feasible": bool(row.get("feasible")),
            "cost": row.get("cost"),
            "n_waypoints": len(pts),
            "path_hash": row.get("path_hash"),
            "yaw": row.get("yaw"),
        })
    ax.scatter([portal[0]], [portal[1]], c="black", s=36, zorder=5, label="portal")
    ax.set_xlabel("container_link x (m)")
    ax.set_ylabel("container_link y (m)")
    ax.set_title("Top view")
    ax.legend(loc="upper right", fontsize=7)
    side = axes[1]
    side.scatter([portal[0]], [portal[2]], c="black", s=36, zorder=5, label="portal")
    for index, row in enumerate(rows[:5]):
        pts = row.get("waypoints") or []
        color = colors[index % len(colors)]
        side.plot([p[0] for p in pts], [p[2] for p in pts],
                  color=color, lw=1.4, alpha=0.45)
        done = _polyline_fraction(pts, fraction)
        if len(done) >= 2:
            side.plot([p[0] for p in done], [p[2] for p in done],
                      color=color, lw=2.8,
                      label="path %d reached %.0f%%" % (index, 100.0 * fraction))
            side.scatter([done[-1][0]], [done[-1][2]], color=color, s=28, zorder=4)
    side.axhline(placed[2] + 0.125, color="#1b4f72", ls="--", lw=1.0,
                 label="box 0 lid")
    side.set_xlabel("container_link x (m)")
    side.set_ylabel("container_link z (m)")
    side.set_title("Side view, thick = cartesian fraction %.3f" % fraction)
    side.legend(loc="upper right", fontsize=7)
    fig.suptitle(
        "Box 2 PLACE_PROBE_FRACTION_REJECT on the revision-2 map\n"
        "live probe: 5 geometric paths, IK ok, last fraction %.3f < 0.95"
        % fraction)
    fig.tight_layout()
    png = os.path.join(out, "probe_fraction_reject.png")
    fig.savefig(png, dpi=140)
    plt.close(fig)

    record = {
        "recorded_probe": probe,
        "measured_box_m": box,
        "placement_message": solved.get("message"),
        "n_replay_paths": len(rows),
        "paths": summary,
        "note": (
            "Thick stroke is the first %.0f%% of each replayed polyline, "
            "the fraction the live probe reported for the last candidate. "
            "The dump did not keep the five waypoint lists, so these are "
            "the geometric carries rebuilt on the saved revision-2 map."
            % (100.0 * fraction)),
    }
    with open(os.path.join(out, "replay.json"), "w", encoding="utf-8") as handle:
        json.dump(record, handle, indent=2, sort_keys=True)
        handle.write("\n")
    return png, len(rows), fraction


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    png, n_paths, fraction = render(args.run, args.out)
    print("png", png)
    print("paths", n_paths, "fraction", fraction)


if __name__ == "__main__":
    main()
