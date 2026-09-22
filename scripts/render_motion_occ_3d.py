#!/usr/bin/env python3
"""Dump-driven 3D visualizer + process video for MOTION-OCC A1 evidence.

These runs have no rosbag. Replay is reconstructed from T2 dumps (tf_trace,
slot, surface_2d, last_boundary, selector waypoints). The container outline
is the scene_tf 7-face inner hull (AABB minus the +Y triangular prism;
±X ends are pentagons), not a cuboid. Motion between measured suction
keyframes is linear. Unexecuted transit is a ghost along selector
waypoints, not the OMPL curve.

Usage:
  PYTHONPATH=src/luggage_planning:src/luggage_description \\
    python3 scripts/render_motion_occ_3d.py \\
      --run docs/status/evidence/motion_occ/2026-09-20_2032
"""

from __future__ import division

import argparse
import json
import math
import os
import shutil
import subprocess
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402

try:
    import plotly.graph_objects as go
except ImportError:
    go = None

from luggage_description.container_geometry import (  # noqa: E402
    descriptor_from_scene_config,
)
from luggage_description.scene_tf_config_utils import (  # noqa: E402
    container_aperture_edges_in_container,
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


def _optional_json(path):
    if path and os.path.isfile(path):
        return _load(path)
    return {}


def _dist(a, b):
    return math.sqrt(sum((float(a[i]) - float(b[i])) ** 2 for i in range(3)))


def _jsonl(path):
    rows = []
    if not os.path.isfile(path):
        return rows
    with open(path, "r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def _trial_dir(run, index, fail_code):
    dumps = os.path.join(run, "dumps")
    path = os.path.join(dumps, "trial_%02d_%s" % (index, fail_code))
    if os.path.isdir(path):
        return path
    for entry in sorted(os.listdir(dumps)):
        if entry.startswith("trial_%02d_" % index):
            return os.path.join(dumps, entry)
    raise SystemExit("no trial dump for index %d in %s" % (index, dumps))


def _map_to_world(origin, yaw, xyz):
    c, s = math.cos(yaw), math.sin(yaw)
    x, y, z = [float(v) for v in xyz]
    return [
        origin[0] + c * x - s * y,
        origin[1] + s * x + c * y,
        origin[2] + z,
    ]


def _edges_world(origin, yaw, edges):
    return [
        (_map_to_world(origin, yaw, a), _map_to_world(origin, yaw, b))
        for a, b in (edges or [])
    ]


def _edges_xyz(edges_world):
    xs, ys, zs = [], [], []
    for a, b in edges_world:
        xs.extend([float(a[0]), float(b[0]), None])
        ys.extend([float(a[1]), float(b[1]), None])
        zs.extend([float(a[2]), float(b[2]), None])
    return xs, ys, zs


def _edges_scatter(edges_world, color, name, width=3):
    xs, ys, zs = _edges_xyz(edges_world)
    return go.Scatter3d(
        x=xs, y=ys, z=zs, mode="lines",
        line=dict(color=color, width=width),
        name=name, hoverinfo="name",
    )


def _plot_edges(ax, edges_world, dims, label=None, **kwargs):
    i, j = dims
    first = True
    for a, b in edges_world:
        ax.plot(
            [a[i], b[i]], [a[j], b[j]],
            label=(label if first else None), **kwargs)
        first = False


def _plot_edges_iso(ax, edges_world, label=None, **kwargs):
    first = True
    for a, b in edges_world:
        pa, pb = _project(*a), _project(*b)
        ax.plot(
            [pa[0], pb[0]], [pa[1], pb[1]],
            label=(label if first else None), **kwargs)
        first = False


def _workspace_scene_tf():
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.normpath(os.path.join(
        here, "..", "src", "luggage_description", "config", "scene_tf.yaml"))
    if os.path.isfile(path):
        return path
    return resolve_scene_tf_config_path()


def _scene_hull_world():
    """Usable inner hull in world. Fail closed if chamfer is missing."""
    path = _workspace_scene_tf()
    config = load_scene_tf_config(path)
    origin, rpy = origin_in_world(config)
    origin = [float(v) for v in origin]
    yaw = float(rpy[2])
    hull = container_inner_hull_edges_in_container(config)
    if len(hull) < 15:
        raise SystemExit(
            "scene_tf inner hull has %d edges (cuboid AABB). "
            "Expected the 7-face chamfered hull from %s" % (len(hull), path))
    return {
        "path": path,
        "origin": origin,
        "yaw": yaw,
        "floor_z": float(container_inner_floor_z(config)),
        "opening": [float(v) for v in container_opening_target_point_in_world(config)],
        "hull_edges": _edges_world(origin, yaw, hull),
        "aperture_edges": _edges_world(
            origin, yaw, container_aperture_edges_in_container(config)),
    }


def _waypoints_world(origin, yaw, waypoints):
    return [_map_to_world(origin, yaw, wp) for wp in (waypoints or [])]


def _aabb(center, size):
    half = [0.5 * float(v) for v in size]
    lo = [float(center[i]) - half[i] for i in range(3)]
    hi = [float(center[i]) + half[i] for i in range(3)]
    return lo, hi


def _payload_center(suction, height):
    return [suction[0], suction[1], suction[2] - 0.5 * float(height)]


def _lerp(a, b, u):
    return [float(a[i]) + u * (float(b[i]) - float(a[i])) for i in range(3)]


def _cuboid_corners(center, size):
    hx, hy, hz = [0.5 * float(v) for v in size]
    cx, cy, cz = [float(v) for v in center]
    return [
        (cx - hx, cy - hy, cz - hz),
        (cx + hx, cy - hy, cz - hz),
        (cx + hx, cy + hy, cz - hz),
        (cx - hx, cy + hy, cz - hz),
        (cx - hx, cy - hy, cz + hz),
        (cx + hx, cy - hy, cz + hz),
        (cx + hx, cy + hy, cz + hz),
        (cx - hx, cy + hy, cz + hz),
    ]


_CUBE_EDGES = (
    (0, 1), (1, 2), (2, 3), (3, 0),
    (4, 5), (5, 6), (6, 7), (7, 4),
    (0, 4), (1, 5), (2, 6), (3, 7),
)

_CUBE_I = [0, 0, 0, 7, 7, 7, 1, 1, 2, 2, 3, 3]
_CUBE_J = [1, 3, 4, 2, 4, 6, 2, 5, 3, 6, 4, 7]
_CUBE_K = [2, 4, 5, 6, 5, 3, 5, 6, 7, 7, 7, 6]


def _mesh3d(center, size, color, opacity, name, visible=True):
    xs, ys, zs = zip(*_cuboid_corners(center, size))
    return go.Mesh3d(
        x=list(xs), y=list(ys), z=list(zs),
        i=_CUBE_I, j=_CUBE_J, k=_CUBE_K,
        color=color, opacity=opacity, name=name,
        showscale=False, visible=visible, hoverinfo="name",
        flatshading=True,
    )


def _wire_traces(center, size, color, name, width=3):
    corners = _cuboid_corners(center, size)
    traces = []
    xs, ys, zs = [], [], []
    for a, b in _CUBE_EDGES:
        xa, ya, za = corners[a]
        xb, yb, zb = corners[b]
        xs.extend([xa, xb, None])
        ys.extend([ya, yb, None])
        zs.extend([za, zb, None])
    traces.append(go.Scatter3d(
        x=xs, y=ys, z=zs, mode="lines",
        line=dict(color=color, width=width),
        name=name, hoverinfo="name",
    ))
    return traces


def _project(x, y, z, azim=-55.0, elev=22.0):
    az = math.radians(azim)
    el = math.radians(elev)
    xp = math.cos(az) * x + math.sin(az) * y
    yp = (-math.sin(az) * math.sin(el) * x
          + math.cos(az) * math.sin(el) * y
          + math.cos(el) * z)
    return xp, yp


def _draw_box_iso(ax, center, size, color, lw=1.2, alpha=1.0, ls="-"):
    corners = _cuboid_corners(center, size)
    proj = [_project(*p) for p in corners]
    for a, b in _CUBE_EDGES:
        ax.plot(
            [proj[a][0], proj[b][0]],
            [proj[a][1], proj[b][1]],
            color=color, lw=lw, alpha=alpha, ls=ls, solid_capstyle="round",
        )


def _rect_xy(ax, center, size, **kwargs):
    lo, _hi = _aabb(center, size)
    ax.add_patch(Rectangle(
        (lo[0], lo[1]), float(size[0]), float(size[1]), **kwargs))


def _rect_xz(ax, center, size, **kwargs):
    lo, _hi = _aabb(center, size)
    ax.add_patch(Rectangle(
        (lo[0], lo[2]), float(size[0]), float(size[2]), **kwargs))


def _occ_world(surface, origin, yaw):
    if not surface:
        return []
    hull = descriptor_from_scene_config(load_scene_tf_config(_workspace_scene_tf()))
    snap = OccupancySnapshot.from_surface_2d(
        stamp_geometry_descriptor(surface, hull), inflate_m=0.05)
    boxes = occupancy_collision_boxes(snap, max_objects=64)
    out = []
    for box in boxes:
        out.append({
            "id": box["id"],
            "xyz": _map_to_world(origin, yaw, box["xyz"]),
            "size": list(box["size"]),
        })
    return out


def _pose_row(row, label):
    suction = row.get("suction")
    if not suction:
        return None
    vac = row.get("vacuum") or {}
    return {
        "label": label,
        "suction": [float(v) for v in suction],
        "t_ros": float(row.get("t_ros") or 0.0),
        "attached": bool(vac.get("attached", False)),
    }


def _after_keyframes(tf_rows):
    """Start pose plus one sample per after_* label, in file order."""
    keys = []
    for row in tf_rows:
        label = str(row.get("label") or "")
        if not keys and label.startswith("before_"):
            item = _pose_row(row, "start")
            if item:
                keys.append(item)
            continue
        if not label.startswith("after_"):
            continue
        item = _pose_row(row, label[6:])
        if item:
            keys.append(item)
    return keys


def _polyline_frames(points, n):
    if len(points) < 2:
        return list(points)
    n = max(2, int(n))
    lengths = []
    total = 0.0
    for i in range(len(points) - 1):
        d = math.sqrt(sum(
            (points[i + 1][k] - points[i][k]) ** 2 for k in range(3)))
        lengths.append(max(d, 1e-6))
        total += lengths[-1]
    out = []
    for s in range(n):
        target = (s / float(n - 1)) * total
        acc = 0.0
        chosen = points[-1]
        for i, length in enumerate(lengths):
            if acc + length >= target:
                u = (target - acc) / length
                chosen = _lerp(points[i], points[i + 1], u)
                break
            acc += length
        out.append(chosen)
    return out


def build_scene(run):
    suite = _load(os.path.join(run, "suite.json"))
    fail_codes = suite["fail_codes"]
    dumps = os.path.join(run, "dumps")
    trial0 = _trial_dir(run, 0, fail_codes[0])
    trial1 = _trial_dir(run, 1, fail_codes[1]) if len(fail_codes) > 1 else None
    diag = _optional_json(
        os.path.join(trial1, "replay", "moveit_diagnosis.json") if trial1 else "")
    geom = dict(diag.get("geometry") or {})
    boundary = _optional_json(
        os.path.join((trial1 or trial0), "replay", "last_boundary.json"))
    surface = _optional_json(
        os.path.join((trial1 or trial0), "replay", "surface_2d.json"))
    slot0 = _optional_json(os.path.join(trial0, "slot.json"))
    refs0 = _load(os.path.join(trial0, "replay", "perception_refs.json"))
    refs1 = _optional_json(
        os.path.join(trial1, "replay", "perception_refs.json") if trial1 else "")
    hull = _scene_hull_world()
    origin = hull["origin"]
    yaw = hull["yaw"]
    inner = [float(v) for v in (
        geom.get("inner_lwh") or surface.get("inner_size") or [1.49, 1.97, 2.01])]
    floor_z = hull["floor_z"]
    opening = hull["opening"]
    placed = geom.get("placed_box")
    payload0 = refs0["pick_detection"]["size_wdh"]
    if not placed and slot0.get("pose_world"):
        placed = {
            "xyz": [float(v) for v in slot0["pose_world"]["position"]],
            "size": [float(v) for v in payload0],
        }
    payload1 = [float(v) for v in geom.get("payload_wdh") or (
        (refs1.get("pick_detection") or {}).get("size_wdh") or payload0)]
    goal = [float(v) for v in (
        geom.get("transit_goal_suction") or boundary.get("target")
        or [1.08, -0.56, 1.057])]
    if not diag.get("cause"):
        n_pts = (boundary.get("trajectory") or {}).get("n_points")
        diag["cause"] = "%s (error_code=%s, traj_pts=%s)" % (
            boundary.get("message") or fail_codes[-1],
            boundary.get("moveit_error_code"),
            n_pts,
        )
    occ0 = _occ_world(
        _load(os.path.join(trial0, "replay", "surface_2d.json"))
        if os.path.isfile(os.path.join(trial0, "replay", "surface_2d.json"))
        else {},
        origin, yaw)
    occ1 = _occ_world(
        _load(os.path.join(trial1, "replay", "surface_2d.json"))
        if trial1 and os.path.isfile(
            os.path.join(trial1, "replay", "surface_2d.json"))
        else {},
        origin, yaw)
    paths0 = os.path.join(dumps, "box_00_paths", "candidates.json")
    paths1 = os.path.join(dumps, "box_01_paths", "candidates.json")
    wp0 = []
    wp1 = []
    if os.path.isfile(paths0):
        winner = _load(paths0).get("winner") or {}
        rows = _load(paths0).get("rows") or []
        selected = next((r for r in rows if r.get("selected")), None)
        wp0 = _waypoints_world(
            origin, yaw, (winner.get("waypoints") or (
                selected or {}).get("waypoints")))
    if os.path.isfile(paths1):
        winner = _load(paths1).get("winner") or {}
        wp1 = _waypoints_world(origin, yaw, winner.get("waypoints_map") or winner.get("waypoints"))
        if not wp1:
            diag_wp = (diag.get("selector") or {}).get("waypoints_map") or []
            wp1 = _waypoints_world(origin, yaw, diag_wp)

    keys0 = _after_keyframes(_jsonl(os.path.join(trial0, "tf_trace.jsonl")))
    keys1 = _after_keyframes(_jsonl(os.path.join(trial1, "tf_trace.jsonl"))) if trial1 else []

    frames = []

    def add_segment(start, end, n, trial, attached, label, occ, payload,
                    ghost=None, placed_visible=False):
        pts = _polyline_frames([start] + ([end] if end != start else []), n)
        if start == end:
            pts = [start] * n
        for i, suction in enumerate(pts):
            frames.append({
                "suction": suction,
                "trial": trial,
                "attached": attached,
                "label": label,
                "occ": occ,
                "payload": payload,
                "ghost": ghost,
                "placed_visible": placed_visible,
                "u": i / float(max(len(pts) - 1, 1)),
            })

    # Trial 0: measured path; transit uses planned waypoints if present.
    prev = keys0[0]["suction"] if keys0 else [-1.0, 0.0, 1.45]
    placed_visible = False
    for key in keys0:
        label = key["label"]
        attached = key["attached"] and label not in ("retreat", "place_exit")
        if label == "descend":
            attached = True
        if label == "transit" and wp0:
            path = [prev] + wp0 + [key["suction"]]
            pts = _polyline_frames(path, 28)
            for suction in pts:
                frames.append({
                    "suction": suction,
                    "trial": 0,
                    "attached": True,
                    "label": "transit",
                    "occ": occ0,
                    "payload": payload0,
                    "ghost": None,
                    "placed_visible": False,
                    "u": 0.0,
                })
        else:
            n = 18 if label in ("stage_mid", "place_exit") else 10
            add_segment(prev, key["suction"], n, 0, attached, label,
                        occ0, payload0, placed_visible=placed_visible)
        if label == "descend":
            placed_visible = True
            # Hold at release so the packed box is readable.
            add_segment(key["suction"], key["suction"], 8, 0, False,
                        "release", occ0, payload0, placed_visible=True)
        prev = key["suction"]

    # Trial 1: staging, then either executed transit or a planned ghost.
    if keys1:
        prev = keys1[0]["suction"]
        fail_label = fail_codes[1] if len(fail_codes) > 1 else "PLACE_PLAN_transit"
        saw_transit = False
        for key in keys1:
            label = key["label"]
            if label == "start":
                prev = key["suction"]
                continue
            if label != "transit":
                add_segment(prev, key["suction"], 12, 1, True, label,
                            occ1, payload1, placed_visible=True)
                prev = key["suction"]
                continue
            moved = _dist(key["suction"], prev) > 0.05
            if moved:
                add_segment(prev, key["suction"], 24, 1, True, "transit (executed)",
                            occ1, payload1, placed_visible=True)
                prev = key["suction"]
            elif not saw_transit:
                ghost_path = [prev] + wp1 + [goal] if wp1 else [prev, goal]
                ghosts = _polyline_frames(ghost_path, 24)
                for i, ghost_suction in enumerate(ghosts):
                    frames.append({
                        "suction": prev,
                        "trial": 1,
                        "attached": True,
                        "label": "transit FAIL (planned, not executed)",
                        "occ": occ1,
                        "payload": payload1,
                        "ghost": ghost_suction,
                        "placed_visible": True,
                        "u": i / float(max(len(ghosts) - 1, 1)),
                    })
            saw_transit = True
        if saw_transit:
            for _ in range(12):
                frames.append({
                    "suction": prev,
                    "trial": 1,
                    "attached": True,
                    "label": fail_label,
                    "occ": occ1,
                    "payload": payload1,
                    "ghost": goal,
                    "placed_visible": True,
                    "u": 1.0,
                })

    return {
        "origin": origin,
        "yaw": yaw,
        "inner": inner,
        "floor_z": floor_z,
        "opening": opening,
        "placed": placed,
        "payload0": payload0,
        "payload1": payload1,
        "goal": goal,
        "occ0": occ0,
        "occ1": occ1,
        "wp0": wp0,
        "wp1": wp1,
        "hull_edges": hull["hull_edges"],
        "aperture_edges": hull["aperture_edges"],
        "frames": frames,
        "diag": diag,
        "fail_codes": fail_codes,
        "trial0": trial0,
        "trial1": trial1,
        "run_id": os.path.basename(os.path.abspath(run)),
    }


def write_html(scene, out_html):
    if go is None:
        raise SystemExit("plotly is required for the 3D HTML visualizer")
    frames_in = scene["frames"]
    placed = scene["placed"]
    origin = scene["origin"]
    floor_z = scene["floor_z"]

    traces = []
    traces.append(_edges_scatter(
        scene["hull_edges"], "#222222", "inner hull (7-face)"))
    traces.append(_edges_scatter(
        scene["aperture_edges"], "#9467bd", "door aperture", width=5))
    occ_boxes = scene["occ1"] or scene["occ0"]
    for i, box in enumerate(occ_boxes):
        mesh = _mesh3d(box["xyz"], box["size"], "#f0a202", 0.28, "occupancy")
        if i:
            mesh.showlegend = False
        traces.append(mesh)
    if placed:
        traces.append(_mesh3d(
            placed["xyz"], placed["size"], "#d62728", 0.85, "placed_0_0_0"))
    traces.append(_mesh3d(
        _payload_center(frames_in[0]["suction"], frames_in[0]["payload"][2]),
        frames_in[0]["payload"], "#2ca02c", 0.7, "carried payload"))
    traces.append(_mesh3d(
        _payload_center(scene["goal"], scene["payload1"][2]),
        scene["payload1"], "#17becf", 0.15, "planned transit goal",
        visible=True))
    traces.append(go.Scatter3d(
        x=[frames_in[0]["suction"][0]],
        y=[frames_in[0]["suction"][1]],
        z=[frames_in[0]["suction"][2]],
        mode="markers", marker=dict(size=5, color="#111111"),
        name="suction"))
    traces.append(go.Scatter3d(
        x=[frames_in[0]["suction"][0]],
        y=[frames_in[0]["suction"][1]],
        z=[frames_in[0]["suction"][2]],
        mode="lines", line=dict(color="#1f77b4", width=5),
        name="measured suction path"))
    traces.append(go.Scatter3d(
        x=[scene["opening"][0]], y=[scene["opening"][1]], z=[scene["opening"][2]],
        mode="markers+text", marker=dict(size=4, color="#9467bd", symbol="diamond"),
        text=["opening"], textposition="top center", name="opening"))
    traces.append(go.Scatter3d(
        x=[origin[0]], y=[origin[1]], z=[origin[2] + floor_z],
        mode="markers", marker=dict(size=3, color="black", symbol="x"),
        name="container origin"))

    def mesh_xyz(center, size):
        xs, ys, zs = zip(*_cuboid_corners(center, size))
        return list(xs), list(ys), list(zs)

    def payload_center(frame):
        if frame["attached"]:
            return _payload_center(frame["suction"], frame["payload"][2])
        if placed:
            return placed["xyz"]
        return _payload_center(frame["suction"], frame["payload"][2])

    def ghost_center(frame):
        suction = frame["ghost"] or scene["goal"]
        return _payload_center(suction, scene["payload1"][2])

    anim_frames = []
    trail = []
    payload_idx = next(i for i, t in enumerate(traces) if t.name == "carried payload")
    ghost_idx = next(i for i, t in enumerate(traces) if t.name == "planned transit goal")
    suction_idx = next(i for i, t in enumerate(traces) if t.name == "suction")
    trail_idx = next(i for i, t in enumerate(traces) if t.name == "measured suction path")
    placed_idx = next(
        (i for i, t in enumerate(traces) if t.name == "placed_0_0_0"), None)

    for idx, frame in enumerate(frames_in):
        if not trail or trail[-1] != frame["suction"]:
            if frame["ghost"] is None:
                trail.append(frame["suction"])
        px, py, pz = mesh_xyz(payload_center(frame), frame["payload"])
        gx, gy, gz = mesh_xyz(ghost_center(frame), scene["payload1"])
        mesh_kw = dict(i=_CUBE_I, j=_CUBE_J, k=_CUBE_K, showscale=False)
        frame_data = [
            go.Mesh3d(x=px, y=py, z=pz, color=(
                "#2ca02c" if frame["trial"] == 0 else "#1f77b4"),
                opacity=(0.18 if not frame["attached"] else 0.7), **mesh_kw),
            go.Mesh3d(x=gx, y=gy, z=gz, color="#17becf",
                      opacity=(0.5 if frame["ghost"] is not None else 0.05),
                      **mesh_kw),
            go.Scatter3d(
                x=[frame["suction"][0]],
                y=[frame["suction"][1]],
                z=[frame["suction"][2]]),
            go.Scatter3d(
                x=[p[0] for p in trail],
                y=[p[1] for p in trail],
                z=[p[2] for p in trail]),
        ]
        frame_traces = [payload_idx, ghost_idx, suction_idx, trail_idx]
        if placed_idx is not None:
            pxs, pys, pzs = mesh_xyz(placed["xyz"], placed["size"])
            frame_data.append(go.Mesh3d(
                x=pxs, y=pys, z=pzs, color="#d62728",
                opacity=(0.85 if frame["placed_visible"] else 0.02), **mesh_kw))
            frame_traces.append(placed_idx)
        anim_frames.append(go.Frame(
            data=frame_data,
            traces=frame_traces,
            name=str(idx),
            layout=go.Layout(title=_title(scene, frame)),
        ))

    fig = go.Figure(data=traces, frames=anim_frames)
    fig.update_layout(
        title=_title(scene, frames_in[0]),
        scene=dict(
            xaxis_title="world X (m)",
            yaxis_title="world Y (m)",
            zaxis_title="world Z (m)",
            aspectmode="data",
            camera=dict(eye=dict(x=1.6, y=-1.8, z=0.9)),
        ),
        margin=dict(l=0, r=0, t=60, b=0),
        legend=dict(x=0.01, y=0.99),
        updatemenus=[dict(
            type="buttons", showactive=False, x=0.05, y=0.02,
            buttons=[
                dict(label="Play", method="animate",
                     args=[None, dict(frame=dict(duration=90, redraw=True),
                                      fromcurrent=True,
                                      transition=dict(duration=0))]),
                dict(label="Pause", method="animate",
                     args=[[None], dict(frame=dict(duration=0, redraw=False),
                                        mode="immediate")]),
            ],
        )],
        sliders=[dict(
            active=0, x=0.18, y=0.02, len=0.75,
            currentvalue=dict(prefix="frame "),
            steps=[dict(method="animate", args=[[str(i)], dict(
                mode="immediate", frame=dict(duration=0, redraw=True),
                transition=dict(duration=0))], label=str(i))
                   for i in range(0, len(anim_frames), max(1, len(anim_frames) // 40))],
        )],
        annotations=[dict(
            text=(
                "Rotate with the mouse. Inner hull is scene_tf 7-face "
                "(AABB minus +Y prism; pentagon ends), not a cuboid. "
                "Play uses measured suction keyframes; transit ghost follows "
                "selector waypoints, not OMPL. No rosbag on this run."
            ),
            x=0.5, y=1.0, xref="paper", yref="paper", showarrow=False,
            font=dict(size=11),
        )],
    )
    fig.write_html(out_html, include_plotlyjs=True, auto_play=False)
    return out_html


def _title(scene, frame):
    cause = ((scene.get("diag") or {}).get("cause") or "")[:90]
    return (
        "Trial %s  |  %s  |  %s"
        % (frame["trial"], frame["label"],
           cause if frame["trial"] == 1 else "first place")
    )


def write_video(scene, out_dir, fps=12):
    frame_dir = os.path.join(out_dir, "frames")
    if os.path.isdir(frame_dir):
        shutil.rmtree(frame_dir)
    os.makedirs(frame_dir)
    placed = scene["placed"]
    hull_edges = scene["hull_edges"]
    aperture_edges = scene["aperture_edges"]
    paths = []
    keep_idx = set([0, len(scene["frames"]) // 2, len(scene["frames"]) - 1])
    for i, frame in enumerate(scene["frames"]):
        if frame["label"].startswith("PLACE_PLAN") or "FAIL" in frame["label"]:
            keep_idx.add(i)
        fig, axes = plt.subplots(2, 2, figsize=(12.8, 7.2), dpi=100)
        ax_xy, ax_xz = axes[0]
        ax_iso, ax_txt = axes[1]
        occ = frame["occ"]
        payload = frame["payload"]
        suction = frame["suction"]
        if frame["attached"]:
            p_center = _payload_center(suction, payload[2])
        elif placed:
            p_center = placed["xyz"]
        else:
            p_center = _payload_center(suction, payload[2])
        ghost = frame["ghost"]
        ghost_center = (
            _payload_center(ghost, scene["payload1"][2]) if ghost else None)

        # XY
        _plot_edges(ax_xy, hull_edges, (0, 1), label="inner hull",
                    color="black", lw=1.6)
        _plot_edges(ax_xy, aperture_edges, (0, 1), label="aperture",
                    color="#9467bd", lw=1.8)
        for box in occ:
            _rect_xy(ax_xy, box["xyz"], box["size"],
                     facecolor="#ffcc66", edgecolor="#c47f00", alpha=0.28, lw=0.5)
        if placed and frame["placed_visible"]:
            _rect_xy(ax_xy, placed["xyz"], placed["size"],
                     facecolor="#d62728", edgecolor="#7f0000", alpha=0.55, lw=1.4,
                     label="placed_0_0_0")
        _rect_xy(ax_xy, p_center, payload,
                 facecolor="#2ca02c" if frame["trial"] == 0 else "#1f77b4",
                 edgecolor="#08306b", alpha=0.45, lw=1.2, label="payload")
        if ghost_center is not None:
            _rect_xy(ax_xy, ghost_center, scene["payload1"],
                     facecolor="none", edgecolor="#17becf", lw=1.6, ls="--",
                     label="planned goal")
        ax_xy.scatter([suction[0]], [suction[1]], c="k", s=18, zorder=5)
        ax_xy.set_aspect("equal")
        ax_xy.set_xlabel("X (m)")
        ax_xy.set_ylabel("Y (m)")
        ax_xy.set_title("Top (XY)")
        ax_xy.grid(True, alpha=0.25)
        ax_xy.legend(loc="upper right", fontsize=7)

        # XZ (opening is the -X pentagon; chamfer is in YZ and shows in iso)
        _plot_edges(ax_xz, hull_edges, (0, 2), color="black", lw=1.6)
        _plot_edges(ax_xz, aperture_edges, (0, 2), color="#9467bd", lw=1.8)
        for box in occ:
            _rect_xz(ax_xz, box["xyz"], box["size"],
                     facecolor="#ffcc66", edgecolor="#c47f00", alpha=0.28, lw=0.5)
        if placed and frame["placed_visible"]:
            _rect_xz(ax_xz, placed["xyz"], placed["size"],
                     facecolor="#d62728", edgecolor="#7f0000", alpha=0.55, lw=1.4)
        _rect_xz(ax_xz, p_center, payload,
                 facecolor="#2ca02c" if frame["trial"] == 0 else "#1f77b4",
                 edgecolor="#08306b", alpha=0.45, lw=1.2)
        if ghost_center is not None:
            _rect_xz(ax_xz, ghost_center, scene["payload1"],
                     facecolor="none", edgecolor="#17becf", lw=1.6, ls="--")
        ax_xz.scatter([suction[0]], [suction[2]], c="k", s=18, zorder=5)
        ax_xz.set_aspect("equal")
        ax_xz.set_xlabel("X (m)")
        ax_xz.set_ylabel("Z (m)")
        ax_xz.set_title("Side (XZ)")
        ax_xz.grid(True, alpha=0.25)

        # Isometric (edge projection; matplotlib Axes3D is broken in this env)
        _plot_edges_iso(ax_iso, hull_edges, label="inner hull",
                        color="#222222", lw=1.4)
        _plot_edges_iso(ax_iso, aperture_edges, color="#9467bd", lw=2.0)
        for box in occ:
            _draw_box_iso(ax_iso, box["xyz"], box["size"], "#c47f00", lw=0.6, alpha=0.5)
        if placed and frame["placed_visible"]:
            _draw_box_iso(ax_iso, placed["xyz"], placed["size"], "#d62728", lw=2.0)
        _draw_box_iso(ax_iso, p_center, payload, "#1f77b4", lw=1.6)
        if ghost_center is not None:
            _draw_box_iso(ax_iso, ghost_center, scene["payload1"],
                          "#17becf", lw=1.6, ls="--")
        sp = _project(*suction)
        ax_iso.scatter([sp[0]], [sp[1]], c="k", s=22, zorder=5)
        ax_iso.set_aspect("equal")
        ax_iso.set_title("3D isometric (7-face hull, +Y cut)")
        ax_iso.set_xticks([])
        ax_iso.set_yticks([])

        ax_txt.axis("off")
        cause = ((scene.get("diag") or {}).get("cause") or "")
        ax_txt.text(
            0.02, 0.95,
            "Trial %s  ·  %s\nfail_codes: %s\n\n%s\n\n"
            "Suction [%.3f, %.3f, %.3f]\n"
            "Replay: dump reconstruction (no rosbag on this run).\n"
            "Inner hull is scene_tf 7-face, not a cuboid.\n"
            "Transit interpolation follows selector waypoints, not OMPL."
            % (frame["trial"], frame["label"],
               ", ".join(scene["fail_codes"]),
               cause, suction[0], suction[1], suction[2]),
            va="top", ha="left", fontsize=10, family="monospace",
            wrap=True, transform=ax_txt.transAxes)
        fig.suptitle(
            "MOTION-OCC A1 %s  ·  process replay" % scene["run_id"],
            fontsize=13)
        fig.tight_layout()
        path = os.path.join(frame_dir, "frame_%04d.png" % i)
        fig.savefig(path)
        plt.close(fig)
        paths.append(path)

    mp4 = os.path.join(out_dir, "process.mp4")
    cmd = [
        "ffmpeg", "-y", "-loglevel", "error",
        "-framerate", str(int(fps)),
        "-i", os.path.join(frame_dir, "frame_%04d.png"),
        "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "23",
        mp4,
    ]
    subprocess.check_call(cmd)
    stills = os.path.join(out_dir, "stills")
    os.makedirs(stills, exist_ok=True)
    for i in sorted(keep_idx):
        src = os.path.join(frame_dir, "frame_%04d.png" % i)
        if os.path.isfile(src):
            shutil.copy2(src, os.path.join(stills, "frame_%04d.png" % i))
    shutil.rmtree(frame_dir)
    return mp4


def write_index(out_dir, html_name, mp4_name, run_id):
    path = os.path.join(out_dir, "index.html")
    with open(path, "w", encoding="utf-8") as handle:
        handle.write("""<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>MOTION-OCC 3D replay</title>
<style>
body { font-family: sans-serif; margin: 16px; max-width: 1280px; }
code, pre { background: #f4f4f4; padding: 2px 4px; }
video { width: 100%%; max-width: 1280px; }
iframe { width: 100%%; height: 720px; border: 1px solid #ccc; }
</style></head><body>
<h1>MOTION-OCC 3D replay (%s)</h1>
<p>This scored run has <b>no rosbag</b>. The video and 3D viewer rebuild the
scene from T2 dumps. The black outline is the <b>scene_tf 7-face inner hull</b>
(AABB minus the +Y triangular prism; ±X ends are pentagons), not a cuboid.
Purple is the door aperture on the -X pentagon. Transit motion is interpolated
along selector waypoints, not the OMPL curve.</p>
<h2>Process video</h2>
<video controls src="%s"></video>
<h2>Interactive 3D</h2>
<p>Open <a href="%s">%s</a> and use Play / rotate.</p>
<iframe src="%s"></iframe>
<h2>ros2 bag vs this dump replay</h2>
<pre>
This run: cannot ros2 bag play (no bag was recorded).

Future A1 (cameras excluded, isolated domain — never 7):
  scripts/motion_occ_place_path_run.sh --out DIR --n 2 --record-bag
  scripts/motion_occ_bag_replay.sh --bag DIR/rosbag/motion_occ

RViz (domain 42): RobotModel + PlanningScene + TF.
Dump HTML is better for occupancy AABB vs hanging payload.
Bag+RViz is better for the robot mesh and live /tf.
Do not ros2 bag play onto ROS_DOMAIN_ID=7.
</pre>
</body></html>
""" % (run_id, mp4_name, html_name, html_name, html_name))
    return path


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", required=True)
    parser.add_argument("--skip-video", action="store_true")
    args = parser.parse_args(argv)
    run = os.path.abspath(args.run)
    out_dir = os.path.join(run, "replay_viz")
    os.makedirs(out_dir, exist_ok=True)
    scene = build_scene(run)
    html = write_html(scene, os.path.join(out_dir, "scene_3d.html"))
    mp4 = None
    if not args.skip_video:
        mp4 = write_video(scene, out_dir)
    index = write_index(
        out_dir, "scene_3d.html", "process.mp4", scene["run_id"])
    if scene.get("trial1"):
        replay = os.path.join(scene["trial1"], "replay")
        os.makedirs(replay, exist_ok=True)
        shutil.copy2(html, os.path.join(replay, "scene_3d.html"))
        if mp4:
            shutil.copy2(mp4, os.path.join(replay, "process.mp4"))
    print(json.dumps({
        "index": index,
        "html": html,
        "mp4": mp4,
        "n_frames": len(scene["frames"]),
        "n_hull_edges": len(scene["hull_edges"]),
        "n_occ_trial1": len(scene["occ1"]),
        "fail_codes": scene["fail_codes"],
    }, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
