#!/usr/bin/env python3
"""3D debug scene for one single-box place attempt.

World frame. The container is the chamfered 7-face hull (pentagon ends,
aperture cut out of the -X face), not a cuboid. Placement centers are
recomputed with the same solver gates as placement_planner_node and checked
against the logged ComputePlacement line. Red X marks are the suction
targets of the reachable attempts that motion actually tried. Marginal
slots stay unmarked by a target when the budget ends inside the reachable
prefix. The blue curve is the executed suction path from the bag TF.

Usage (from the workspace root, ROS sourced for the bag):

  PYTHONPATH=src/luggage_description:src/luggage_packing:src/luggage_planning \\
    python3 scripts/render_place_scene_3d.py \\
      --run docs/status/evidence/frame_migration/ST09-parallel-I-s6-readiness/5db68f4

The atlas directory defaults to the name stored in the planning request
(``reachability_atlas_deploy_sim`` for the s6 readiness runs). Pass
``--atlas-dir`` when that directory is not under
``src/luggage_planning/data``. Recompute uses whichever
``placement_solver`` is on ``PYTHONPATH``; ``scene_3d.json`` records
whether its band line matches the logged ComputePlacement message.
"""

from __future__ import division

import argparse
import json
import math
import os
import sys

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
DEFAULT_SCENE = os.path.join(
    REPO, "src", "luggage_description", "config", "scene_tf_deploy.yaml")

# The apt mpl_toolkits package is a regular package and hides matplotlib's
# namespace mplot3d when both are on sys.path. Prefer the mplot3d that ships
# beside the matplotlib we actually imported.
import matplotlib  # noqa: E402


def _prefer_matching_mplot3d():
    import types
    site = os.path.dirname(os.path.dirname(os.path.abspath(matplotlib.__file__)))
    mplot = os.path.join(site, "mpl_toolkits", "mplot3d")
    if not os.path.isdir(mplot):
        return
    for name in list(sys.modules):
        if name == "mpl_toolkits" or name.startswith("mpl_toolkits."):
            del sys.modules[name]
    package = types.ModuleType("mpl_toolkits")
    package.__path__ = [os.path.join(site, "mpl_toolkits")]
    package.__package__ = "mpl_toolkits"
    sys.modules["mpl_toolkits"] = package


_prefer_matching_mplot3d()
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402
import numpy as np  # noqa: E402
import plotly.graph_objects as go  # noqa: E402
from PIL import Image  # noqa: E402

from luggage_description.container_geometry import (
    normalize_descriptor,
    y_max_at_z,
    yz_polygon,
)
from luggage_packing.placement_reachability import (
    make_atlas_annotate,
    resolve_atlas_path,
)
from luggage_packing import placement_solver as _placement_solver
from luggage_packing.placement_solver import (
    placement_constraint_verdict,
    solve_placement,
)

# Newer solvers export the same-layer gap and the band line the node logs.
# Older trees still annotate atlas_status; the figure falls back locally.
SAME_LAYER_CLEARANCE_M = getattr(
    _placement_solver, "SAME_LAYER_CLEARANCE_M", 0.05)
_solver_prefix_note = getattr(_placement_solver, "candidate_prefix_note", None)
from luggage_planning.atlas_runtime import apply_query_semantics
from luggage_planning.reachability_atlas import ReachabilityAtlas

INK = "#0b0b0b"
INK2 = "#52514e"
BLUE = "#2a78d6"
AQUA = "#1baf7a"
CRIT = "#d03b3b"
NEUTRAL = "#efeeea"
RAMP = ("#cde2fb", "#86b6ef", "#3987e5", "#1c5cab", "#104281")
HULL = "#9ec3f5"
CHAMFER = "#5b97e0"
PLACE_UNKNOWN = "#86b6ef"
PLACE_REACH = "#1c5cab"
HULL_REJECT = "#d03b3b"
APERTURE_REJECT = "#8a8680"

# Locked flange / contact chain, elfin_link6 -> suction_contact_frame.
# URDF literals: elfin_end_joint, suction_flange_origin.xacro,
# suction_contact_frame_joint.
FIXED_CHAIN = (
    ((0.0, 0.0, 0.1257), (0.0, 0.0, 3.1416)),
    ((0.0006, 0.0, 0.0), (-1.56451314, 0.0, 3.14159265)),
    ((0.0, -0.286, 0.0), (1.57079632679, 0.0, 0.0)),
)
# Node default headings. placement_planner_node declares the same four.
ALLOWED_YAWS = [0.0, 1.5707963, 3.14159265, -1.5707963]


def _load(path):
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


def _rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _T(xyz, rpy):
    out = np.eye(4)
    out[:3, :3] = _rpy_matrix(*rpy)
    out[:3, 3] = np.asarray(xyz, dtype=float)
    return out


def _quat_to_R(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _yaw_of_quat(quat):
    x, y, z, w = [float(v) for v in quat]
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _apply(T, points):
    pts = np.asarray(points, dtype=float)
    if pts.ndim == 1:
        return (T[:3, :3] @ pts) + T[:3, 3]
    return (T[:3, :3] @ pts.T).T + T[:3, 3]


def _fan(poly):
    """Triangles of a convex polygon. poly is (N, 3)."""
    tris = []
    for i in range(1, len(poly) - 1):
        tris.append([poly[0], poly[i], poly[i + 1]])
    return tris


def _quad(a, b, c, d):
    return [[a, b, c], [a, c, d]]


def _box_corners(center, size, yaw):
    cx, cy, cz = [float(v) for v in center]
    sx, sy, sz = [0.5 * float(v) for v in size]
    c, s = math.cos(float(yaw)), math.sin(float(yaw))
    corners = []
    for dx in (-sx, sx):
        for dy in (-sy, sy):
            for dz in (-sz, sz):
                corners.append((
                    cx + c * dx - s * dy,
                    cy + s * dx + c * dy,
                    cz + dz,
                ))
    return corners


BOX_EDGES = (
    (0, 1), (2, 3), (4, 5), (6, 7),
    (0, 2), (1, 3), (4, 6), (5, 7),
    (0, 4), (1, 5), (2, 6), (3, 7),
)


def aperture_bounds(corners):
    """YZ extent of the four aperture corners in container_link."""
    ys = [float(corner[1]) for corner in corners]
    zs = [float(corner[2]) for corner in corners]
    return {
        "y0": min(ys), "y1": max(ys),
        "z0": min(zs), "z1": max(zs),
    }


def _hull_faces(geometry, aper):
    """Seven faces in container_link. The -X pentagon keeps the wall around
    the aperture; the doorway itself is not filled."""
    poly = yz_polygon(geometry)
    hx = geometry.half_x
    minus = [np.array([-hx, y, z]) for y, z in poly]
    plus = [np.array([hx, y, z]) for y, z in poly]
    faces = {"hull": [], "chamfer": [], "door_wall": []}
    n = len(poly)
    for i in range(n):
        j = (i + 1) % n
        y0, z0 = poly[i]
        y1, z1 = poly[j]
        tris = _quad(minus[i], plus[i], plus[j], minus[j])
        rising = z1 > z0 + 1e-6 and y1 > y0 + 1e-6
        key = "chamfer" if rising else "hull"
        faces[key].extend(tris)
    faces["hull"].extend(_fan(plus))
    if geometry.chamfer is None:
        faces["hull"].extend(_fan(minus))
        return faces, poly, aper
    for quad in _door_wall_quads(geometry, hx, aper):
        faces["door_wall"].extend(_quad(*quad))
    return faces, poly, aper


def _door_wall_quads(geometry, hx, aper):
    """Wall pieces of the -X pentagon that are not the doorway."""
    x = -hx
    y_south = -geometry.half_y
    y_north = geometry.half_y
    floor_z = geometry.floor_z
    ceiling = geometry.ceiling_z
    wall_z = geometry.chamfer.wall_z

    def y_lim(z):
        return y_max_at_z(geometry, z)

    def P(y, z):
        return np.array([x, y, z])

    quads = []
    # South sliver, full height (vertical -Y wall).
    quads.append((
        P(y_south, floor_z), P(aper["y0"], floor_z),
        P(aper["y0"], ceiling), P(y_south, ceiling),
    ))
    # Sill under the door, west of the chamfer.
    quads.append((
        P(aper["y0"], floor_z), P(aper["y1"], floor_z),
        P(aper["y1"], aper["z0"]), P(aper["y0"], aper["z0"]),
    ))
    # Chamfer corner beside the sill.
    quads.append((
        P(aper["y1"], floor_z), P(y_lim(floor_z), floor_z),
        P(y_lim(aper["z0"]), aper["z0"]), P(aper["y1"], aper["z0"]),
    ))
    # North wall across the chamfer, beside the door.
    quads.append((
        P(aper["y1"], aper["z0"]), P(y_lim(aper["z0"]), aper["z0"]),
        P(y_lim(wall_z), wall_z), P(aper["y1"], wall_z),
    ))
    # Vertical +Y wall beside the door, up to the aperture top.
    quads.append((
        P(aper["y1"], wall_z), P(y_north, wall_z),
        P(y_north, aper["z1"]), P(aper["y1"], aper["z1"]),
    ))
    # Lintel. The aperture top is 1 mm under the ceiling.
    if ceiling - aper["z1"] > 1e-4:
        quads.append((
            P(aper["y0"], aper["z1"]), P(aper["y1"], aper["z1"]),
            P(aper["y1"], ceiling), P(aper["y0"], ceiling),
        ))
    return quads


def _aperture_loop(hx, aper):
    x = -hx
    y0, y1 = aper["y0"], aper["y1"]
    z0, z1 = aper["z0"], aper["z1"]
    return [
        np.array([x, y0, z0]), np.array([x, y1, z0]),
        np.array([x, y1, z1]), np.array([x, y0, z1]),
        np.array([x, y0, z0]),
    ]


def _bottom_center_box(translation, size, yaw):
    """Box whose URDF/scene origin is the bottom face center."""
    center = [
        float(translation[0]),
        float(translation[1]),
        float(translation[2]) + 0.5 * float(size[2]),
    ]
    return _box_corners(center, size, yaw)


def _node_validator(hull, floor_z, inner_h, aperture_y):
    """Same gates placement_planner_node applies on top of the solver."""

    def hull_contains(point):
        x, y, z_rel = point
        z = z_rel + floor_z
        if not (float(hull.floor_z) - 1e-9 <= z <= float(hull.ceiling_z)):
            return False
        if not (-hull.half_x <= x <= hull.half_x):
            return False
        if y < -hull.half_y:
            return False
        return y <= y_max_at_z(hull, z) + 1e-9

    def validator(candidate):
        kwargs = {
            "placed_aabbs": [],
            "aperture_y": aperture_y,
            "hull_contains": hull_contains,
            "inner_size": (hull.length, hull.width, inner_h),
            "smallest_size": candidate["size"],
            "hull": hull,
        }
        parameters = placement_constraint_verdict.__code__.co_varnames
        if "floor_z" in parameters:
            kwargs["floor_z"] = floor_z
        if "same_layer_clearance_m" in parameters:
            kwargs["same_layer_clearance_m"] = SAME_LAYER_CLEARANCE_M
        if "support_tol" in parameters:
            kwargs["support_tol"] = 0.05
        return placement_constraint_verdict(candidate, inner_h, **kwargs)

    return validator


def _prefix_note(returned, feasible_count):
    if _solver_prefix_note is not None:
        return _solver_prefix_note(returned, feasible_count)
    bands = {}
    depths = []
    for candidate in returned:
        status = candidate.get("atlas_status")
        if status == 3:
            band = "reachable"
        elif status == 2:
            band = "marginal"
        elif str(candidate.get("atlas_status_name") or "") == "no_atlas":
            band = "no_atlas"
        else:
            band = "unknown"
        bands[band] = bands.get(band, 0) + 1
        depths.append(float(candidate.get("depth_m") or 0.0))
    band_text = ",".join(
        "%s:%d" % item for item in sorted(bands.items())) or "none"
    span = (
        "%.3f..%.3f" % (min(depths), max(depths)) if depths else "none")
    return "returned=%d/%d bands=%s depth=%s" % (
        len(returned), int(feasible_count), band_text, span)


def _solve(surface, box, atlas_dir, aperture_y):
    paths = resolve_atlas_path(
        atlas_dir, "s20_container_collision_aware", box)
    if paths is None:
        raise SystemExit("atlas not found under %s" % atlas_dir)
    atlas = ReachabilityAtlas.load(*paths)
    floor_z = float(surface["geometry_descriptor"]["floor_z"])
    inner_h = float(surface["inner_size"][2])
    atlas = apply_query_semantics(
        atlas, {"query_semantics": {"enabled": True}},
        floor_z=floor_z, box_height=float(box[2]))
    hook = make_atlas_annotate(
        atlas, floor_z=floor_z, inner_h=inner_h, score_weight=0.3,
        hard_reject=True, dump_only=False)
    hull = normalize_descriptor(surface["geometry_descriptor"])
    result = solve_placement(
        surface, box,
        allowed_yaws=ALLOWED_YAWS,
        params={
            "clearance_margin": 0.03,
            "same_layer_clearance_m": SAME_LAYER_CLEARANCE_M,
            "min_support_ratio": 0.6,
            "top_n": 2000,
            "keep_rejected": 200,
            "hull_margin": 0.0,
        },
        candidate_validator=_node_validator(
            hull, floor_z, inner_h, aperture_y),
        atlas_annotate=hook,
    )
    feasible = [c for c in result["candidates"] if c.get("feasible")]
    result["prefix"] = _prefix_note(feasible, len(feasible))
    result["atlas_files"] = list(paths)
    return result


def _tcp_path(bag, base_xyz, base_rpy):
    """Suction-contact path in world. Empty when the bag or ROS is absent."""
    if not bag or not os.path.isdir(bag):
        return np.zeros((0, 3)), [], np.zeros((0,))
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:
        print("tcp path skipped: %s" % error)
        return np.zeros((0, 3)), [], np.zeros((0,))

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr"),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    tf_type = get_message(types["/tf"])
    T_tool = np.eye(4)
    for xyz, rpy in FIXED_CHAIN:
        T_tool = T_tool @ _T(xyz, rpy)
    T_world_base = _T(base_xyz, base_rpy)
    order = [
        ("elfin_base", "elfin_link1"),
        ("elfin_link1", "elfin_link2"),
        ("elfin_link2", "elfin_link3"),
        ("elfin_link3", "elfin_link4"),
        ("elfin_link4", "elfin_link5"),
        ("elfin_link5", "elfin_link6"),
    ]
    tcp = []
    arms = []
    stamps = []
    while reader.has_next():
        topic, data, _ts = reader.read_next()
        if topic != "/tf":
            continue
        msg = deserialize_message(data, tf_type)
        by_edge = {}
        stamp = None
        for tr in msg.transforms:
            key = (tr.header.frame_id, tr.child_frame_id)
            q = tr.transform.rotation
            t = tr.transform.translation
            T = np.eye(4)
            T[:3, :3] = _quat_to_R(q.x, q.y, q.z, q.w)
            T[:3, 3] = (t.x, t.y, t.z)
            by_edge[key] = T
            stamp = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
        if any(edge not in by_edge for edge in order):
            continue
        T = T_world_base.copy()
        joints = [_apply(T, np.zeros(3))]
        for edge in order:
            T = T @ by_edge[edge]
            joints.append(_apply(T, np.zeros(3)))
        T_contact = T @ T_tool
        tip = _apply(T_contact, np.zeros(3))
        joints.append(tip)
        tcp.append(tip)
        arms.append(np.vstack(joints))
        stamps.append(stamp)
    if not tcp:
        return np.zeros((0, 3)), [], np.zeros((0,))
    return np.vstack(tcp), arms, np.asarray(stamps)


def _unique_segments(rows):
    seen = {}
    for row in rows:
        for edge in row.get("cartesian") or []:
            a = tuple(round(float(v), 4) for v in edge["from_position"])
            b = tuple(round(float(v), 4) for v in edge["to_position"])
            frac = float(edge["fraction"])
            key = (a, b)
            seen[key] = max(frac, seen.get(key, 0.0))
    return [(np.array(a), np.array(b), frac) for (a, b), frac in seen.items()]


def _reason_of(candidate):
    if candidate.get("feasible"):
        status = candidate.get("atlas_status")
        if status == 3:
            return "reachable"
        if status == 2:
            return "marginal"
        return "unknown"
    return str(candidate.get("reason") or "rejected")


def suction_contact_local(center, box_height):
    """Box-top contact in the same frame as ``center``.

    This is ``solver_contact_point`` for a ``center_container`` candidate:
    the suction contact is the box top, not the box center.
    """
    point = np.array(center, dtype=float)
    point[2] = float(center[2]) + 0.5 * float(box_height)
    return point


def _best_query_status(statuses):
    """Prefer a positive atlas sample over unknown, and unknown over a hard miss."""
    rank = {3: 0, 2: 1, 0: 2, 1: 3}
    return min((int(s) for s in statuses), key=lambda s: rank.get(s, 9))


def _status_name(status):
    return {3: "reachable", 2: "marginal", 0: "unknown", 1: "unreachable"}.get(
        int(status), "other")


def candidate_layers(candidates, dumped_rows, box_height):
    """Separate box centers from the points the atlas and the TCP actually use.

    Feasible box centers are not colored by atlas status. A dump row's
    ``atlas_queries[].query_xyz`` is the recorded query. A candidate with no
    recorded query is drawn at its suction contact plus that row's
    ``query_offset`` (zero when the dump does not carry one). Hull and
    aperture rejects stay at the box center: that is the point the gate
    rejected.
    """
    recorded = {}
    configured_offset = np.zeros(3)
    for row in dumped_rows or []:
        pose = (row.get("slot_pose") or {}).get("position")
        queries = row.get("atlas_queries") or []
        if pose is None or not queries:
            continue
        key = (round(float(pose[0]), 3), round(float(pose[1]), 3))
        offset = row.get("query_offset") or [0.0, 0.0, 0.0]
        if len(offset) == 3:
            configured_offset = np.array(offset, dtype=float)
        by_point = recorded.setdefault(key, {})
        for query in queries:
            xyz = query.get("query_xyz")
            if not xyz:
                continue
            point = tuple(round(float(v), 6) for v in xyz)
            by_point.setdefault(point, []).append(int(query.get("status", 0)))

    layers = {
        "box_center": [],
        "suction_contact": [],
        "reachable": [],
        "marginal": [],
        "unknown": [],
        "unreachable": [],
        "outside_hull": [],
        "outside_aperture": [],
        "other": [],
    }
    for candidate in candidates or []:
        center = candidate.get("center_container")
        if center is None:
            continue
        center = np.array(center, dtype=float)
        reason = _reason_of(candidate)
        if reason in ("outside_hull", "outside_aperture", "other"):
            layers[reason].append(center)
            continue
        layers["box_center"].append(center)
        contact = suction_contact_local(center, box_height)
        layers["suction_contact"].append(contact)
        key = (round(float(center[0]), 3), round(float(center[1]), 3))
        queries = recorded.get(key)
        if queries:
            for point, statuses in queries.items():
                layers[_status_name(_best_query_status(statuses))].append(
                    np.array(point, dtype=float))
            continue
        query = contact + configured_offset
        layers[reason].append(query)
    return layers, [float(v) for v in configured_offset]


def _median_z(points):
    if points is None or len(points) == 0:
        return None
    return round(float(np.median(np.asarray(points, dtype=float)[:, 2])), 6)


def layer_heights(local_layers, tcp_local, configured_offset):
    """Container-frame heights of the four point meanings.

    Atlas-query height prefers points that are not sitting on the suction
    contact, so a zero ``query_offset`` does not hide a recorded TCP query.
    """
    contact_z = {
        round(float(point[2]), 4) for point in local_layers["suction_contact"]}
    query_z = []
    for key in ("reachable", "marginal", "unknown", "unreachable"):
        for point in local_layers[key]:
            if round(float(point[2]), 4) not in contact_z:
                query_z.append(float(point[2]))
    if not query_z:
        for key in ("reachable", "marginal", "unknown", "unreachable"):
            query_z.extend(float(point[2]) for point in local_layers[key])
    return {
        "box_center_z_m": _median_z(local_layers["box_center"]),
        "suction_contact_z_m": _median_z(local_layers["suction_contact"]),
        "atlas_query_z_m": (
            round(float(np.median(query_z)), 6) if query_z else None),
        "tcp_target_z_m": _median_z(tcp_local),
        "configured_query_offset_m": [
            round(float(v), 6) for v in configured_offset],
        "status_is_drawn_at": "atlas query point, not the box center",
    }


def _edges_built(row):
    coverage = (row.get("detail") or {}).get("coverage") or {}
    return int(coverage.get("edges_built") or 0)


def _scatter3(xs, ys, zs, color, name, size=3, symbol="circle", opacity=0.85):
    return go.Scatter3d(
        x=xs, y=ys, z=zs, mode="markers", name=name,
        marker=dict(size=size, color=color, symbol=symbol, opacity=opacity),
        hovertemplate=name + "<br>%{x:.3f}, %{y:.3f}, %{z:.3f}<extra></extra>",
    )


def _line3(points, color, name, width=4):
    xs, ys, zs = [], [], []
    for a, b in points:
        xs.extend([float(a[0]), float(b[0]), None])
        ys.extend([float(a[1]), float(b[1]), None])
        zs.extend([float(a[2]), float(b[2]), None])
    return go.Scatter3d(
        x=xs, y=ys, z=zs, mode="lines", name=name,
        line=dict(color=color, width=width), hoverinfo="name",
    )


def _mesh(tris, color, name, opacity):
    verts = []
    for tri in tris:
        verts.extend(tri)
    xyz = np.asarray(verts, dtype=float)
    n = len(tris)
    i = list(range(0, 3 * n, 3))
    j = list(range(1, 3 * n, 3))
    k = list(range(2, 3 * n, 3))
    return go.Mesh3d(
        x=xyz[:, 0], y=xyz[:, 1], z=xyz[:, 2],
        i=i, j=j, k=k, color=color, opacity=opacity, name=name,
        hoverinfo="name", flatshading=True, showlegend=True,
    )


def _draw_edges(ax, edges, color, lw=0.8, alpha=1.0):
    for a, b in edges:
        ax.plot(
            [a[0], b[0]], [a[1], b[1]], [a[2], b[2]],
            color=color, lw=lw, alpha=alpha, solid_capstyle="round")


def _draw_edges_2d(ax, edges, dims, color, lw=0.8, alpha=1.0, ls="-"):
    i, j = dims
    for a, b in edges:
        ax.plot(
            [a[i], b[i]], [a[j], b[j]],
            color=color, lw=lw, alpha=alpha, ls=ls, solid_capstyle="round")


def _world_edges(T, corners):
    pts = _apply(T, np.asarray(corners, dtype=float))
    return [(pts[i], pts[j]) for i, j in BOX_EDGES]


def _audit(fig):
    fig.canvas.draw()
    rend = fig.canvas.get_renderer()
    bad = 0
    for ax_ in fig.axes:
        boxes = [
            (t.get_text().split("\n")[0], t.get_window_extent(rend))
            for t in ax_.texts if t.get_text().strip()
        ]
        for label, bb in boxes:
            if (bb.x0 < 1 or bb.x1 > fig.bbox.x1 - 1
                    or bb.y0 < 1 or bb.y1 > fig.bbox.y1 - 1):
                print("CLIP", repr(label))
                bad += 1
        for i in range(len(boxes)):
            for j in range(i + 1, len(boxes)):
                l1, b1 = boxes[i]
                l2, b2 = boxes[j]
                ox = min(b1.x1, b2.x1) - max(b1.x0, b2.x0)
                oy = min(b1.y1, b2.y1) - max(b1.y0, b2.y0)
                if (ox > 2 and oy > 2
                        and ox * oy > 0.10 * min(
                            b1.width * b1.height, b2.width * b2.height)):
                    print("OVERLAP", repr(l1), "<->", repr(l2))
                    bad += 1
    if fig._suptitle is not None:
        st = fig._suptitle.get_window_extent(rend)
        if st.y1 > fig.bbox.y1 - 1 or st.x0 < 1 or st.x1 > fig.bbox.x1 - 1:
            print("CLIP suptitle")
            bad += 1
    print("AUDIT:", "CLEAN" if bad == 0 else bad)
    return bad


def atlas_dir_from_identity(identity, explicit=""):
    """Directory of the atlas named in the planning request.

    ``identity`` is ``<data dir>|<basename>|<hash>``. An explicit ``--atlas-dir``
    wins. A named directory that is not in this tree is an error: falling
    through to a different atlas would redraw the wrong bands.
    """
    if explicit:
        return os.path.abspath(explicit)
    name = str(identity or "").split("|", 1)[0].strip()
    if not name:
        name = "reachability_atlas_deploy"
    path = os.path.join(
        REPO, "src", "luggage_planning", "data", name)
    if os.path.isdir(path):
        return path
    raise SystemExit(
        "atlas directory %s is not under src/luggage_planning/data. "
        "Pass --atlas-dir." % name)


def resolve_evidence(run):
    """Accept either the run directory or its ``single_box`` child."""
    run = os.path.abspath(run)
    single = os.path.join(run, "single_box")
    if os.path.isdir(os.path.join(run, "dumps", "box_00_paths")):
        single = run
        root = os.path.dirname(run)
    elif os.path.isdir(os.path.join(single, "dumps", "box_00_paths")):
        root = run
    else:
        raise SystemExit(
            "no box_00_paths dump under %s" % run)
    dumps = os.path.join(single, "dumps")
    trials = sorted(
        name for name in os.listdir(dumps)
        if name.startswith("trial_")
        and os.path.isdir(os.path.join(dumps, name)))
    if len(trials) != 1:
        raise SystemExit("expected one trial_* directory, found %r" % trials)
    return {
        "root": root,
        "single": single,
        "trial": os.path.join(dumps, trials[0]),
        "paths": os.path.join(dumps, "box_00_paths"),
        "bag": os.path.join(single, "bag"),
    }


def load_cell(scene_path):
    """Aperture, arm base, platform, and pedestal from a scene_tf file."""
    from luggage_description.scene_tf_config_utils import (
        container_opening_aperture_corners_in_container,
        load_scene_tf_config,
        pedestal_config,
        pedestal_enabled,
        pickup_platform_config,
        pickup_platform_enabled,
        robot_base_in_world,
    )
    config = load_scene_tf_config(scene_path)
    aper = aperture_bounds(
        container_opening_aperture_corners_in_container(config))
    base_xyz, base_rpy = robot_base_in_world(config)
    platform = None
    if pickup_platform_enabled(config):
        plat = pickup_platform_config(config)
        platform = (
            plat["translation"], plat["size"], float(plat["rotation_rpy"][2]))
    pedestal = None
    if pedestal_enabled(config):
        ped = pedestal_config(config)
        pedestal = (
            ped["translation"], ped["size"], float(ped["rotation_rpy"][2]))
    return {
        "aperture": aper,
        "aperture_y": (aper["y0"], aper["y1"]),
        "base_xyz": [float(v) for v in base_xyz],
        "base_rpy": [float(v) for v in base_rpy],
        "platform": platform,
        "pedestal": pedestal,
        "scene": scene_path,
    }


def placement_matches_log(message, prefix, n_feasible):
    """True when recomputed bands and feasible count equal the logged line."""
    if "bands=" not in (message or "") or "bands=" not in (prefix or ""):
        return False
    logged_bands = message.split("bands=", 1)[1].split(" depth=", 1)[0].strip()
    got_bands = prefix.split("bands=", 1)[1].split(" depth=", 1)[0].strip()
    head = message.split("returned=", 1)[0]
    logged_feasible = ""
    for part in head.split():
        if part.startswith("feasible="):
            logged_feasible = part.split("=", 1)[1]
    return (
        logged_bands == got_bands
        and logged_feasible == str(int(n_feasible)))


def scene_story(record):
    """One sentence for the figure title."""
    motion = record["motion"]
    slots = motion.get("complete_route_slots") or []
    if slots:
        route = "slot %s built %d edges and died at %s" % (
            slots[0],
            int(motion.get("complete_route_edges_built") or 0),
            motion.get("complete_route_failure") or "retreat")
    else:
        route = "no attempt built a full edge chain"
    return (
        "Pick executed. %d reachable builds failed; %s. "
        "%d marginal slots were not entered."
        % (int(motion.get("n_reachable_tried") or 0), route,
           int(motion.get("n_marginal_band") or 0)))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--run", required=True,
        help="Run directory, or its single_box child, containing the dump")
    parser.add_argument("--scene-tf", default=DEFAULT_SCENE)
    parser.add_argument(
        "--atlas-dir", default="",
        help="Reachability atlas directory. Default: the directory named "
             "in the planning request's atlas identity")
    parser.add_argument(
        "--out", default="",
        help="Directory for scene_3d.html/png/json. Default: the run directory")
    args = parser.parse_args(argv)
    found = resolve_evidence(args.run)
    cell = load_cell(args.scene_tf)
    trial_dir = found["trial"]
    path_dir = found["paths"]
    surface = _load(os.path.join(trial_dir, "replay", "surface_2d.json"))
    refs = _load(os.path.join(trial_dir, "replay", "perception_refs.json"))
    boundary = _load(os.path.join(trial_dir, "perception_boundary.json"))
    request = _load(os.path.join(path_dir, "planning_request.json"))
    dumped = _load(os.path.join(path_dir, "candidates.json"))
    trial = _load(os.path.join(trial_dir, "trial.json"))
    identity = request.get("atlas_identity") or {}
    if isinstance(identity, dict):
        identity = identity.get("identity") or ""
    args.atlas_dir = atlas_dir_from_identity(identity, args.atlas_dir)
    box = [float(v) for v in refs["pick_detection"]["size_wdh"]]
    detected = boundary["diagnostics"]["detected"]
    pick_center = np.array(detected["position"], dtype=float)
    pick_yaw = _yaw_of_quat(detected["orientation"])
    pick_top = np.array([
        pick_center[0], pick_center[1],
        float(refs["pick_detection"]["top_surface_z"])])

    geom = normalize_descriptor(surface["geometry_descriptor"])
    ct = request["container_transform"]
    T_world_container = np.eye(4)
    T_world_container[:3, :3] = _quat_to_R(*ct["quaternion_xyzw"])
    T_world_container[:3, 3] = np.asarray(ct["translation"], dtype=float)

    def to_world(points):
        return _apply(T_world_container, points)

    def to_local(points):
        R = T_world_container[:3, :3]
        t = T_world_container[:3, 3]
        pts = np.asarray(points, dtype=float)
        if pts.ndim == 1:
            return R.T @ (pts - t)
        return (R.T @ (pts - t).T).T

    # Slot center must land on the stored suction-target XY.
    probe = next(r for r in dumped["rows"] if "slot_pose" in r)
    slot = np.array(probe["slot_pose"]["position"], dtype=float)
    slot_w = to_world(slot)
    tgt0 = np.array(probe["targets"][0][0], dtype=float)
    xy_err = float(np.linalg.norm(slot_w[:2] - tgt0[:2]))
    if xy_err > 0.002:
        raise SystemExit("container transform xy error %.4f m" % xy_err)

    print("solving placement...")
    solved = _solve(surface, box, args.atlas_dir, cell["aperture_y"])
    hist = dict(solved["reject_histogram"])
    n_feasible = sum(1 for c in solved["candidates"] if c.get("feasible"))
    logged = trial["extras"].get("compute_placement") or ""
    print("solved", solved["message"])
    print("prefix", solved["prefix"])
    print("histogram", hist, "feasible", n_feasible)
    logged_ok = placement_matches_log(logged, solved["prefix"], n_feasible)
    print("matches logged compute_placement:", logged_ok)

    print("reading bag TF...")
    tcp, arms, stamps = _tcp_path(
        found["bag"], cell["base_xyz"], cell["base_rpy"])
    if len(tcp):
        dist = np.linalg.norm(tcp - pick_top, axis=1)
        grasp_i = int(np.argmin(dist))
        print(
            "tcp samples", len(tcp),
            "closest to pick top %.4f m at stamp %.2f" % (
                dist[grasp_i], stamps[grasp_i]),
            "final", np.round(tcp[-1], 3).tolist(),
        )
    else:
        dist = np.zeros((0,))
        grasp_i = None
        print("tcp samples 0")

    faces, _poly, aper = _hull_faces(geom, cell["aperture"])
    faces_w = {
        key: [[to_world(p) for p in tri] for tri in tris]
        for key, tris in faces.items()
    }
    hull_edges_l = []
    poly = yz_polygon(geom)
    hx = geom.half_x
    minus = [np.array([-hx, y, z]) for y, z in poly]
    plus = [np.array([hx, y, z]) for y, z in poly]
    for i in range(len(poly)):
        j = (i + 1) % len(poly)
        hull_edges_l.append((minus[i], minus[j]))
        hull_edges_l.append((plus[i], plus[j]))
        hull_edges_l.append((minus[i], plus[i]))
    hull_edges = [(to_world(a), to_world(b)) for a, b in hull_edges_l]
    door = [to_world(p) for p in _aperture_loop(hx, aper)]
    door_edges = list(zip(door[:-1], door[1:]))

    local_layers, configured_offset = candidate_layers(
        solved["candidates"], dumped["rows"], box[2])
    groups = {}
    for key, points in local_layers.items():
        groups[key] = (
            np.vstack([to_world(point) for point in points])
            if points else np.zeros((0, 3)))
    band_counts = {"reachable": 0, "marginal": 0, "unknown": 0}
    for candidate in solved["candidates"]:
        reason = _reason_of(candidate)
        if reason in band_counts:
            band_counts[reason] += 1

    selected = solved["selected"]
    sel_center = np.array(selected["center_container"], dtype=float)
    sel_yaw = float(selected.get("box_yaw", 0.0))
    sel_edges = _world_edges(
        T_world_container, _box_corners(sel_center, box, sel_yaw))

    tried_rows = [
        r for r in dumped["rows"]
        if "slot_pose" in r and r.get("reachability") == "reachable"
    ]
    tried_edges = []
    complete_edges = []
    complete_slots = []
    for row in tried_rows:
        center = np.array(row["slot_pose"]["position"], dtype=float)
        yaw = _yaw_of_quat(row["slot_pose"]["quaternion_xyzw"])
        edges = _world_edges(
            T_world_container, _box_corners(center, box, yaw))
        if _edges_built(row) >= 7:
            complete_edges.extend(edges)
            complete_slots.append(int(row["slot_index"]))
        else:
            tried_edges.extend(edges)
    target_xyz = [
        t[0] for row in tried_rows for t in (row.get("targets") or [])]
    targets = (
        np.asarray(target_xyz, dtype=float) if target_xyz else np.zeros((0, 3)))
    segments = _unique_segments(tried_rows)
    progress, gap = [], []
    for a, b, frac in segments:
        mid = a + frac * (b - a)
        progress.append((a, mid))
        gap.append((mid, b))
    fracs = [item[2] for item in segments]
    complete_rows = [row for row in tried_rows if _edges_built(row) >= 7]

    pick_edges = _world_edges(
        np.eye(4), _box_corners(pick_center, box, pick_yaw))
    # Carried box: rigid offset from the grasp TCP, applied at the final TCP.
    # Orientation is not stored per sample beyond the contact origin, so the
    # carried box uses the grasp-time vertical offset.
    if grasp_i is None:
        carried_edges = []
        grasp_arm = np.zeros((0, 3))
        final_arm = np.zeros((0, 3))
        final_tcp = None
    else:
        grasp_tcp = tcp[grasp_i]
        final_tcp = tcp[-1]
        offset = pick_center - grasp_tcp
        carried_center = final_tcp + offset
        carried_edges = _world_edges(
            np.eye(4), _box_corners(carried_center, box, pick_yaw))
        grasp_arm = arms[grasp_i]
        final_arm = arms[-1]

    plat_edges = []
    if cell["platform"] is not None:
        plat_edges = _world_edges(
            np.eye(4), _bottom_center_box(*cell["platform"]))
    ped_edges = []
    if cell["pedestal"] is not None:
        ped_edges = _world_edges(
            np.eye(4), _bottom_center_box(*cell["pedestal"]))

    motion = {
        "n_targets": int(len(targets)),
        "n_reachable_band": int(band_counts["reachable"]),
        "n_marginal_band": int(band_counts["marginal"]),
        "n_reachable_tried": len(tried_rows),
        "n_marginal_tried": sum(
            1 for row in dumped["rows"]
            if row.get("reachability") == "marginal"),
        "complete_route_slots": complete_slots,
        "complete_route_edges_built": (
            _edges_built(complete_rows[0]) if complete_rows else 0),
        "complete_route_failure": (
            complete_rows[0]["reason"] if complete_rows else ""),
        "tried_reasons": sorted({row["reason"] for row in tried_rows}),
        "cartesian_segments": len(segments),
        "cartesian_fraction_min": (
            None if not fracs else round(min(fracs), 4)),
        "cartesian_fraction_max": (
            None if not fracs else round(max(fracs), 4)),
        "attempt_summary": dumped["rows"][-1],
    }
    record = {
        "evidence": found["root"],
        "scene_tf": cell["scene"],
        "atlas_dir": args.atlas_dir,
        "fail_code": trial["fail_code"],
        "compute_placement_logged": trial["extras"]["compute_placement"],
        "recompute_message": solved["message"],
        "recompute_prefix": solved["prefix"],
        "recompute_matches_log": logged_ok,
        "feasible": int(n_feasible),
        "reject_histogram": hist,
        "selected": {
            "id": selected.get("candidate_id"),
            "depth_m": selected.get("depth_m"),
            "center_container": [round(float(v), 4) for v in sel_center],
            "atlas_status": selected.get("atlas_status_name"),
        },
        "motion": motion,
        "pick": {
            "center_world": [round(float(v), 4) for v in pick_center],
            "top_z": round(float(pick_top[2]), 4),
            "size_wdh": [round(float(v), 4) for v in box],
            "yaw_rad": round(pick_yaw, 4),
        },
        "tcp": {
            "n": int(len(tcp)),
            "closest_to_pick_top_m": (
                None if grasp_i is None else round(float(dist[grasp_i]), 4)),
            "final_world": (
                None if final_tcp is None
                else [round(float(v), 4) for v in final_tcp]),
        },
        "container_translation": [float(v) for v in ct["translation"]],
        "slot_to_target_xy_error_m": round(xy_err, 6),
        "atlas_files": solved["atlas_files"],
        "layers": layer_heights(
            local_layers,
            to_local(targets) if len(targets) else [],
            configured_offset),
    }
    record["note"] = scene_story(record) + (
        " Box centers, suction contacts, atlas queries, and place TCP "
        "targets are separate marks. Atlas status is drawn on the query "
        "point, not on the box center. A red box is an attempt that built "
        "at least 7 edges.")

    out_dir = os.path.abspath(args.out or found["root"])
    os.makedirs(out_dir, exist_ok=True)
    html_path = os.path.join(out_dir, "scene_3d.html")
    png_path = os.path.join(out_dir, "scene_3d.png")
    json_path = os.path.join(out_dir, "scene_3d.json")
    _write_html(
        html_path, faces_w, hull_edges, door_edges, groups, sel_edges,
        tried_edges, complete_edges, pick_edges, carried_edges, plat_edges,
        ped_edges,
        tcp, grasp_arm, final_arm, targets, pick_top, progress, gap, record)
    bad = _write_png(
        png_path, faces_w, hull_edges, door_edges, groups, sel_edges,
        tried_edges, complete_edges, pick_edges, carried_edges, plat_edges,
        ped_edges,
        tcp, grasp_arm, final_arm, targets, pick_top, progress, gap,
        to_world, to_local, geom, aper, record)
    with open(json_path, "w", encoding="utf-8") as handle:
        json.dump(record, handle, indent=2)
        handle.write("\n")
    print("wrote", png_path)
    print("wrote", html_path)
    print("audit_bad", bad)
    return bad


def _write_html(
        path, faces_w, hull_edges, door_edges, groups, sel_edges, tried_edges,
        complete_edges,
        pick_edges, carried_edges, plat_edges, ped_edges, tcp, grasp_arm,
        final_arm, targets, pick_top, progress, gap, record):
    traces = [
        _mesh(faces_w["hull"], HULL, "7-face hull", 0.22),
        _mesh(faces_w["chamfer"], "#1c5cab", "chamfer (+Y slope)", 0.72),
        _mesh(faces_w["door_wall"], "#d7e6f8", "-X wall around door", 0.55),
        _line3(hull_edges, BLUE, "hull edges", 3),
        _line3(door_edges, AQUA, "door aperture", 6),
        _line3(plat_edges, INK2, "pickup platform", 2),
        _line3(ped_edges, INK2, "pedestal", 2),
        _line3(pick_edges, BLUE, "pick box (detected)", 4),
        _line3(carried_edges, "#104281", "box at final TCP", 3),
        _line3(sel_edges, AQUA, "placement selected slot", 5),
        _line3(tried_edges, "#7aa2d6", "reachable attempts (incomplete)", 2),
        _line3(complete_edges, CRIT, "complete route, retreat missing", 5),
    ]
    style = {
        "box_center": (INK2, 2.2, "circle"),
        "suction_contact": ("#86b6ef", 3.2, "circle-open"),
        "unknown": (PLACE_UNKNOWN, 2.4, "circle"),
        "unreachable": (INK2, 2.4, "diamond"),
        "marginal": ("#3987e5", 5.5, "diamond"),
        "reachable": (PLACE_REACH, 4.2, "circle"),
        "outside_hull": (HULL_REJECT, 3.2, "circle"),
        "outside_aperture": (APERTURE_REJECT, 3.2, "diamond"),
    }
    labels = {
        "box_center": "box center",
        "suction_contact": "suction contact (box top)",
        "unknown": "atlas query unknown",
        "unreachable": "atlas query unreachable",
        "marginal": "atlas query marginal",
        "reachable": "atlas query reachable",
        "outside_hull": "box center outside hull",
        "outside_aperture": "box center outside aperture",
    }
    for key, (color, size, symbol) in style.items():
        pts = groups.get(key)
        if pts is None or len(pts) == 0:
            continue
        traces.append(_scatter3(
            pts[:, 0], pts[:, 1], pts[:, 2], color,
            "%s (%d)" % (labels[key], len(pts)), size=size, symbol=symbol))
    traces.append(go.Scatter3d(
        x=tcp[:, 0], y=tcp[:, 1], z=tcp[:, 2], mode="lines",
        name="executed suction path (pick)",
        line=dict(color=BLUE, width=6),
    ))
    traces.append(_line3(
        [(grasp_arm[i], grasp_arm[i + 1]) for i in range(len(grasp_arm) - 1)],
        INK, "arm at grasp", 4))
    traces.append(_line3(
        [(final_arm[i], final_arm[i + 1]) for i in range(len(final_arm) - 1)],
        INK2, "arm at end", 3))
    traces.append(_line3(progress, "#104281", "traverse progress", 5))
    traces.append(_line3(gap, CRIT, "traverse not reached", 4))
    traces.append(_scatter3(
        targets[:, 0], targets[:, 1], targets[:, 2], CRIT,
        "place TCP target (%d)" % len(targets), size=5, symbol="x"))
    traces.append(_scatter3(
        [pick_top[0]], [pick_top[1]], [pick_top[2]], CRIT,
        "pick target", size=8, symbol="x"))
    title = scene_story(record)
    fig = go.Figure(traces)
    fig.update_layout(
        title=dict(text=title, font=dict(size=14, color=INK)),
        paper_bgcolor="white",
        legend=dict(font=dict(size=11, color=INK), itemsizing="constant"),
        scene=dict(
            aspectmode="data",
            xaxis_title="world x (m)",
            yaxis_title="world y (m)",
            zaxis_title="world z (m)",
            camera=dict(eye=dict(x=1.35, y=-2.15, z=0.95)),
        ),
        margin=dict(l=0, r=0, t=48, b=0),
    )
    fig.write_html(path, include_plotlyjs=True)


def _write_png(
        path, faces_w, hull_edges, door_edges, groups, sel_edges, tried_edges,
        complete_edges,
        pick_edges, carried_edges, plat_edges, ped_edges, tcp, grasp_arm,
        final_arm, targets, pick_top, progress, gap, to_world, to_local,
        geom, aper, record):
    fig = plt.figure(figsize=(16.8, 8.6), dpi=140, facecolor="white")
    ax3 = fig.add_axes([0.02, 0.08, 0.56, 0.78], projection="3d")
    ax_top = fig.add_axes([0.60, 0.52, 0.36, 0.36])
    ax_sec = fig.add_axes([0.60, 0.08, 0.36, 0.36])
    for ax_ in (ax3, ax_top, ax_sec):
        ax_.set_facecolor("white")

    def add_faces(tris, color, alpha):
        poly = Poly3DCollection(
            tris, facecolors=color, edgecolors="none", alpha=alpha)
        ax3.add_collection3d(poly)

    add_faces(faces_w["hull"], HULL, 0.28)
    add_faces(faces_w["chamfer"], "#1c5cab", 0.72)
    add_faces(faces_w["door_wall"], "#d7e6f8", 0.62)
    _draw_edges(ax3, hull_edges, BLUE, 0.8)
    hx = geom.half_x
    chamfer_edges = [
        (to_world(np.array([-hx, geom.chamfer.floor_y, geom.floor_z])),
         to_world(np.array([hx, geom.chamfer.floor_y, geom.floor_z]))),
        (to_world(np.array([-hx, geom.chamfer.floor_y, geom.floor_z])),
         to_world(np.array([-hx, geom.half_y, geom.chamfer.wall_z]))),
        (to_world(np.array([hx, geom.chamfer.floor_y, geom.floor_z])),
         to_world(np.array([hx, geom.half_y, geom.chamfer.wall_z]))),
        (to_world(np.array([-hx, geom.half_y, geom.chamfer.wall_z])),
         to_world(np.array([hx, geom.half_y, geom.chamfer.wall_z]))),
    ]
    _draw_edges(ax3, chamfer_edges, "#104281", 2.0)
    _draw_edges(ax3, door_edges, AQUA, 1.6)
    _draw_edges(ax3, plat_edges, INK2, 0.6, 0.7)
    _draw_edges(ax3, ped_edges, INK2, 0.6, 0.7)
    _draw_edges(ax3, pick_edges, BLUE, 1.3)
    _draw_edges(ax3, carried_edges, "#104281", 1.0)
    _draw_edges(ax3, sel_edges, AQUA, 1.5)
    _draw_edges(ax3, tried_edges, "#7aa2d6", 0.8)
    _draw_edges(ax3, complete_edges, CRIT, 1.8)
    ax3.plot(tcp[:, 0], tcp[:, 1], tcp[:, 2], color=BLUE, lw=1.6, zorder=5)
    for arm, color, lw in ((grasp_arm, INK, 1.4), (final_arm, INK2, 1.0)):
        ax3.plot(arm[:, 0], arm[:, 1], arm[:, 2], color=color, lw=lw)
    for a, b in progress:
        ax3.plot(
            [a[0], b[0]], [a[1], b[1]], [a[2], b[2]],
            color="#104281", lw=1.4)
    for a, b in gap:
        ax3.plot(
            [a[0], b[0]], [a[1], b[1]], [a[2], b[2]],
            color=CRIT, lw=1.1, ls=(0, (2, 1.5)))
    ax3.scatter(
        targets[:, 0], targets[:, 1], targets[:, 2],
        c=CRIT, marker="x", s=18, linewidths=0.8, depthshade=False, zorder=6)
    ax3.scatter(
        [pick_top[0]], [pick_top[1]], [pick_top[2]],
        c=CRIT, marker="x", s=46, linewidths=1.4, depthshade=False, zorder=7)
    style = {
        "box_center": (INK2, 5, "o", 2),
        "suction_contact": ("#86b6ef", 9, "o", 3),
        "unknown": (PLACE_UNKNOWN, 6, "o", 4),
        "unreachable": (INK2, 7, "D", 4),
        "marginal": ("#3987e5", 18, "D", 5),
        "reachable": (PLACE_REACH, 11, "o", 5),
        "outside_hull": (HULL_REJECT, 8, "o", 4),
        "outside_aperture": (APERTURE_REJECT, 8, "D", 4),
    }
    for key, (color, size, marker, zorder) in style.items():
        pts = groups.get(key)
        if pts is None or len(pts) == 0:
            continue
        ax3.scatter(
            pts[:, 0], pts[:, 1], pts[:, 2], c=color, s=size, marker=marker,
            depthshade=False, linewidths=0, zorder=zorder)
    ax3.set_xlabel("x (m)", color=INK, labelpad=2, fontsize=8)
    ax3.set_ylabel("y (m)", color=INK, labelpad=2, fontsize=8)
    ax3.set_zlabel("z (m)", color=INK, labelpad=2, fontsize=8)
    ax3.tick_params(labelsize=7, colors=INK2)
    # From world -Y / +X so the chamfer slope and the door are both in view.
    ax3.view_init(elev=21, azim=-128)
    ax3.set_box_aspect((3.6, 2.4, 2.1))
    ax3.set_xlim(-2.35, 1.65)
    ax3.set_ylim(-1.35, 1.55)
    ax3.set_zlim(0.0, 2.15)
    ax3.set_title("World, door facing +X", color=INK, fontsize=11, pad=8)

    def top_edges(edges):
        _draw_edges_2d(ax_top, edges, (0, 1), BLUE, 0.7)

    top_edges(hull_edges)
    _draw_edges_2d(ax_top, door_edges, (0, 1), AQUA, 1.5)
    _draw_edges_2d(ax_top, plat_edges, (0, 1), INK2, 0.6)
    _draw_edges_2d(ax_top, pick_edges, (0, 1), BLUE, 1.2)
    _draw_edges_2d(ax_top, sel_edges, (0, 1), AQUA, 1.3)
    _draw_edges_2d(ax_top, tried_edges, (0, 1), "#7aa2d6", 0.7)
    _draw_edges_2d(ax_top, complete_edges, (0, 1), CRIT, 1.4)
    ax_top.plot(tcp[:, 0], tcp[:, 1], color=BLUE, lw=1.3)
    for a, b in progress:
        ax_top.plot([a[0], b[0]], [a[1], b[1]], color="#104281", lw=1.2)
    for a, b in gap:
        ax_top.plot(
            [a[0], b[0]], [a[1], b[1]], color=CRIT, lw=1.0, ls=(0, (2, 1.5)))
    for key, (color, size, marker, zorder) in style.items():
        pts = groups.get(key)
        if len(pts) == 0:
            continue
        ax_top.scatter(
            pts[:, 0], pts[:, 1], c=color, s=max(4, size - 2),
            marker=marker, linewidths=0, zorder=zorder)
    ax_top.scatter(
        targets[:, 0], targets[:, 1], c=CRIT, marker="x", s=14,
        linewidths=0.7, zorder=5)
    ax_top.scatter(
        [pick_top[0]], [pick_top[1]], c=CRIT, marker="x", s=36,
        linewidths=1.2, zorder=6)
    ax_top.set_aspect("equal")
    ax_top.set_xlim(-2.4, 1.7)
    ax_top.set_ylim(-1.4, 1.6)
    ax_top.set_xlabel("world x (m)", color=INK, fontsize=8)
    ax_top.set_ylabel("world y (m)", color=INK, fontsize=8)
    ax_top.tick_params(labelsize=7, colors=INK2)
    floor = [
        np.array([-hx, -geom.half_y, geom.floor_z]),
        np.array([hx, -geom.half_y, geom.floor_z]),
        np.array([hx, geom.chamfer.floor_y, geom.floor_z]),
        np.array([-hx, geom.chamfer.floor_y, geom.floor_z]),
        np.array([-hx, -geom.half_y, geom.floor_z]),
    ]
    floor_w = [to_world(p) for p in floor]
    _draw_edges_2d(
        ax_top, list(zip(floor_w[:-1], floor_w[1:])), (0, 1),
        "#104281", 1.6)
    ax_top.set_title("Top view", color=INK, fontsize=11, pad=4)
    ax_top.text(1.02, 0.72, "pick", color=INK, fontsize=8, ha="center")
    ax_top.text(-1.50, -1.15, "container", color=INK, fontsize=8, ha="center")
    ax_top.text(
        -0.15, 1.15,
        "%d reachable tried" % int(record["motion"]["n_reachable_tried"]),
        color=INK, fontsize=8, ha="center")

    # YZ section in container_link: the pentagon and the chamfer.
    poly = yz_polygon(geom)
    ys = [p[0] for p in poly] + [poly[0][0]]
    zs = [p[1] for p in poly] + [poly[0][1]]
    ax_sec.plot(ys, zs, color=BLUE, lw=1.4)
    ax_sec.plot(
        [aper["y0"], aper["y1"], aper["y1"], aper["y0"], aper["y0"]],
        [aper["z0"], aper["z0"], aper["z1"], aper["z1"], aper["z0"]],
        color=AQUA, lw=1.4)
    local_groups = {}
    for key, pts in groups.items():
        local_groups[key] = to_local(pts) if len(pts) else pts
    for key, (color, size, marker, zorder) in style.items():
        pts = local_groups[key]
        if len(pts) == 0:
            continue
        ax_sec.scatter(
            pts[:, 1], pts[:, 2], c=color, s=size, marker=marker,
            linewidths=0, zorder=zorder)
    # Only points whose container x is inside the hull. The pick target
    # sits 2.5 m in front of the door; projecting it onto YZ looks in-hull.
    loc_t = to_local(targets)
    inside = np.abs(loc_t[:, 0]) <= geom.half_x + 0.02
    loc_t = loc_t[inside]
    ax_sec.scatter(
        loc_t[:, 1], loc_t[:, 2], c=CRIT, marker="x", s=16,
        linewidths=0.7, zorder=5)
    ax_sec.set_aspect("equal")
    ax_sec.set_xlim(-1.15, 1.15)
    ax_sec.set_ylim(0.35, 2.15)
    ax_sec.set_xlabel("container y (m)", color=INK, fontsize=8)
    ax_sec.set_ylabel("container z (m)", color=INK, fontsize=8)
    ax_sec.tick_params(labelsize=7, colors=INK2)
    ax_sec.set_title("Container section (7-face profile)", color=INK, fontsize=11, pad=4)
    heights = record.get("layers") or {}
    ax_sec.text(
        -0.55, 1.32,
        "center %.3f\ncontact %.3f\nquery/TCP %.3f" % (
            heights.get("box_center_z_m") or 0.0,
            heights.get("suction_contact_z_m") or 0.0,
            heights.get("atlas_query_z_m") or heights.get("tcp_target_z_m") or 0.0),
        color=INK, fontsize=8, ha="center", va="center")
    ax_sec.text(
        0.98, 0.08, "chamfer", transform=ax_sec.transAxes,
        color=INK, fontsize=8, ha="right")
    ax_sec.text(
        0.04, 0.55, "door", transform=ax_sec.transAxes,
        color=INK, fontsize=8, va="center")

    handles = [
        Line2D([0], [0], color=BLUE, lw=1.6, label="executed pick path"),
        Line2D([0], [0], color="#104281", lw=1.4, label="traverse progress"),
        Line2D([0], [0], color=CRIT, lw=1.1, ls=(0, (2, 1.5)),
               label="traverse gap"),
        Line2D([0], [0], color=CRIT, marker="x", lw=0, label="place TCP target"),
        Line2D([0], [0], color=INK2, marker="o", lw=0, label="box center"),
        Line2D([0], [0], color="#86b6ef", marker="o", lw=0, label="suction contact"),
        Line2D([0], [0], color=PLACE_REACH, marker="o", lw=0,
               label="atlas query reachable"),
        Line2D([0], [0], color="#3987e5", marker="D", lw=0,
               label="atlas query marginal"),
        Line2D([0], [0], color=CRIT, lw=1.6, label="complete route failed"),
    ]
    fig.legend(
        handles=handles, loc="lower center", ncol=4, frameon=False,
        fontsize=8, bbox_to_anchor=(0.50, 0.005), labelcolor=INK)
    fig.suptitle(
        scene_story(record),
        color=INK, fontsize=11, x=0.50, y=0.975)
    bad = _audit(fig)
    fig.savefig(path, dpi=140)
    _probe(path, fig, ax3, ax_sec)
    plt.close(fig)
    return bad


def _probe(path, fig, ax3, ax_sec):
    im = Image.open(path).convert("RGB")
    rend = fig.canvas.get_renderer()
    sx = im.size[0] / fig.bbox.x1
    height = fig.bbox.y1 * sx

    def probe(ax, xd, yd, zd=None):
        bb = ax.get_window_extent(rend)
        if zd is None:
            x0, x1 = ax.get_xlim()
            y0, y1 = ax.get_ylim()
            xp = (bb.x0 + (xd - x0) / (x1 - x0) * bb.width) * sx
            yp = (bb.y0 + (yd - y0) / (y1 - y0) * bb.height) * sx
        else:
            # 3D display coords are not a linear data map; skip.
            return None
        xi, yi = int(xp), int(height - yp)
        if not (0 <= xi < im.size[0] and 0 <= yi < im.size[1]):
            return None
        return im.getpixel((xi, yi))

    door = probe(ax_sec, -0.2, 1.2)
    chamfer = probe(ax_sec, 0.75, 0.7)
    print("PROBE section door", door, "chamfer-side", chamfer)


if __name__ == "__main__":
    raise SystemExit(main())
