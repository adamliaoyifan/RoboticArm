#!/usr/bin/env python3
"""Geometry GT dumps after a successful place (no ROS).

Live cargo-map integration is slice B. This module rasterizes the same
``CargoVolumeMapper`` from scene_tf + the placed box AABB, then samples a
world-frame point cloud of the inner surfaces plus the box. That cloud is the
occupancy target: complete, labelled, not limited by the wrist camera FOV.
"""

from __future__ import division

import json
import math
import os
import struct
import zlib

from luggage_description.scene_tf_config_utils import (
    _compose,
    container_aperture_edges_in_container,
    container_hull_local_inside_fn,
    container_in_base_link,
    container_inner_ceiling_z,
    container_inner_chamfer,
    container_inner_floor_z,
    container_inner_hull_edges_in_container,
    container_inner_y_max,
    container_opening_aperture_corners_in_container,
    container_usable_center_in_base_link,
    container_usable_dimensions,
    origin_in_world,
    xyz_base_link_to_world,
)
from luggage_perception.cargo_volume_mapper import (
    FREE,
    CargoVolumeMapper,
)


DEFAULT_RESOLUTION = 0.05
DEFAULT_SPACING = 0.03
FREE_VOXEL_RGB = (72, 168, 210)
WALL_RGB = (160, 160, 170)
FLOOR_RGB = (90, 90, 100)
CHAMFER_RGB = (196, 148, 88)
APERTURE_RGB = (240, 210, 64)
CATALOG_RGB = {
    "carryon": (220, 96, 32),
    "standard": (46, 160, 140),
    "large": (120, 80, 180),
    "unknown": (200, 200, 80),
}
BOX_PALETTE = (
    (220, 96, 32), (46, 160, 140), (70, 130, 220),
    (200, 80, 160), (120, 80, 180), (210, 170, 40),
    (80, 180, 90), (200, 90, 70),
)


def mapper_from_scene(config, resolution=DEFAULT_RESOLUTION):
    inner = container_usable_dimensions(config)
    _xyz, rpy = container_in_base_link(config)
    return CargoVolumeMapper(
        inner_size=inner,
        center_base=container_usable_center_in_base_link(config),
        yaw=float(rpy[2]),
        resolution=float(resolution),
        hull_local_inside=container_hull_local_inside_fn(config),
    )


def mark_placed_on_mapper(mapper, center_base, size_wdh, yaw=0.0):
    mapper.mark_placed_box(
        [float(v) for v in center_base],
        [float(v) for v in size_wdh],
        yaw=float(yaw),
    )
    return mapper


def fill_unoccupied_as_free(mapper):
    """Geometry GT: empty usable hull is known-free, not unknown."""
    mapper.fill_unoccupied_as_free()


def occupancy_image(surface_map, scale=8):
    """Top-down RGB of the 2.5D surface. Opening (-X) is at the bottom."""
    height = surface_map["height"]
    state = surface_map["state"]
    nx = int(surface_map["nx"])
    ny = int(surface_map["ny"])
    inner_h = float(surface_map.get("inner_size", [0, 0, 1.0])[2] or 1.0)
    rows = []
    for ix in range(nx - 1, -1, -1):
        row = []
        for iy in range(ny):
            cell = state[ix][iy]
            if cell == "occupied":
                t = min(1.0, max(0.0, float(height[ix][iy]) / max(inner_h, 1e-6)))
                row.append(_height_rgb(t))
            elif cell == "free":
                row.append((36, 48, 62))
            else:
                row.append((8, 8, 10))
        rows.append(row)
    if scale > 1:
        scaled = []
        for row in rows:
            fat = []
            for pix in row:
                fat.extend([pix] * int(scale))
            for _ in range(int(scale)):
                scaled.append(list(fat))
        rows = scaled
    # Opening edge: first row after the ix flip is +X; last row is -X.
    if rows:
        n_edge = max(1, int(scale))
        edge = [(240, 240, 240)] * len(rows[-1])
        for k in range(1, min(n_edge, len(rows)) + 1):
            rows[-k] = edge
    return rows


def occupancy_image_array(surface_map, scale=8):
    import numpy as np
    rows = occupancy_image(surface_map, scale=scale)
    return np.asarray(rows, dtype=np.uint8)


def _height_rgb(t):
    """Blue (low) → yellow (high)."""
    r = int(255 * min(1.0, t * 1.4))
    g = int(255 * min(1.0, 0.25 + t))
    b = int(255 * max(0.0, 1.0 - t))
    return (r, g, b)


def _grid_1d(low, high, spacing):
    if high <= low:
        return [0.5 * (low + high)]
    n = max(2, int(math.ceil((high - low) / float(spacing))) + 1)
    return [low + (high - low) * i / float(n - 1) for i in range(n)]


def _container_link_to_world(config, local_xyz):
    origin, rpy = origin_in_world(config)
    world, _ = _compose(origin, rpy, list(local_xyz), [0.0, 0.0, 0.0])
    return world


def _aperture_yz_bounds(config, margin=0.0):
    corners = container_opening_aperture_corners_in_container(config, margin)
    if not corners:
        return None
    ys = [float(c[1]) for c in corners]
    zs = [float(c[2]) for c in corners]
    return min(ys), max(ys), min(zs), max(zs)


def sample_container_inner_cloud(config, spacing=DEFAULT_SPACING):
    """7-face inner hull in world. Opening pentagon has an aperture hole."""
    inner_l, inner_w, _span = container_usable_dimensions(config)
    floor_z = container_inner_floor_z(config)
    ceiling_z = container_inner_ceiling_z(config)
    hx, hy = 0.5 * inner_l, 0.5 * inner_w
    xs = _grid_1d(-hx, hx, spacing)
    zs = _grid_1d(floor_z, ceiling_z, spacing)
    chamfer = container_inner_chamfer(config)
    aper = _aperture_yz_bounds(config, margin=0.5 * float(spacing))
    points = []
    colors = []

    def add(local, color):
        points.append(_container_link_to_world(config, local))
        colors.append(color)

    y_floor_max = container_inner_y_max(floor_z, config)
    for x in xs:
        for y in _grid_1d(-hy, y_floor_max, spacing):
            add([x, y, floor_z], FLOOR_RGB)
    for x in xs:
        for y in _grid_1d(-hy, hy, spacing):
            add([x, y, ceiling_z], WALL_RGB)
    for x in xs:
        for z in zs:
            add([x, -hy, z], WALL_RGB)
    z_plus = chamfer["wall_z"] if chamfer is not None else floor_z
    for x in xs:
        for z in zs:
            if z + 1e-6 < z_plus:
                continue
            add([x, hy, z], WALL_RGB)
    if chamfer is not None:
        n_cut = max(2, int(math.ceil(
            (chamfer["wall_z"] - chamfer["floor_z"]) / float(spacing))) + 1)
        for x in xs:
            for i in range(n_cut):
                t = float(i) / float(n_cut - 1)
                y = chamfer["floor_y"] + t * (
                    chamfer["wall_y"] - chamfer["floor_y"])
                z = chamfer["floor_z"] + t * (
                    chamfer["wall_z"] - chamfer["floor_z"])
                add([x, y, z], CHAMFER_RGB)
    for x, opening in ((-hx, True), (hx, False)):
        for z in zs:
            y_hi = container_inner_y_max(z, config)
            for y in _grid_1d(-hy, y_hi, spacing):
                if opening and aper is not None:
                    y0, y1, z0, z1 = aper
                    if y0 < y < y1 and z0 < z < z1:
                        continue
                add([x, y, z], WALL_RGB)
    rim = _aperture_yz_bounds(config, margin=0.0)
    if rim is not None:
        y0, y1, z0, z1 = rim
        for y in _grid_1d(y0, y1, spacing):
            add([-hx, y, z0], APERTURE_RGB)
            add([-hx, y, z1], APERTURE_RGB)
        for z in _grid_1d(z0, z1, spacing):
            add([-hx, y0, z], APERTURE_RGB)
            add([-hx, y1, z], APERTURE_RGB)
    return points, colors


def sample_box_cloud(center, size_wdh, rpy, spacing=DEFAULT_SPACING):
    """Axis-aligned box in the box frame, posed by world XYZ + RPY."""
    w, d, h = [float(v) for v in size_wdh]
    hx, hy, hz = 0.5 * w, 0.5 * d, 0.5 * h
    xs = _grid_1d(-hx, hx, spacing)
    ys = _grid_1d(-hy, hy, spacing)
    zs = _grid_1d(-hz, hz, spacing)
    locals_ = []
    for x in xs:
        for y in ys:
            locals_.append([x, y, -hz])
            locals_.append([x, y, hz])
        for z in zs:
            locals_.append([x, -hy, z])
            locals_.append([x, hy, z])
    for y in ys:
        for z in zs:
            locals_.append([-hx, y, z])
            locals_.append([hx, y, z])
    rpy = [float(v) for v in (rpy or [0.0, 0.0, 0.0])]
    center = [float(v) for v in center]
    points = []
    for local in locals_:
        world, _ = _compose(center, rpy, local, [0.0, 0.0, 0.0])
        points.append(world)
    colors = [(220, 96, 32)] * len(points)
    return points, colors


def write_ply_xyzrgb(path, points, colors=None):
    points = list(points or [])
    n = len(points)
    has_rgb = colors is not None and len(colors) == n
    with open(path, "wb") as handle:
        header = (
            "ply\nformat binary_little_endian 1.0\n"
            "element vertex %d\n"
            "property float x\nproperty float y\nproperty float z\n"
            % n
        )
        if has_rgb:
            header += (
                "property uchar red\nproperty uchar green\n"
                "property uchar blue\n")
        header += "end_header\n"
        handle.write(header.encode("ascii"))
        for i, p in enumerate(points):
            handle.write(struct.pack("<fff", float(p[0]), float(p[1]), float(p[2])))
            if has_rgb:
                r, g, b = colors[i]
                handle.write(struct.pack("BBB", int(r), int(g), int(b)))
    return n


def build_place_gt(config, box_center_base, box_size, box_yaw,
                   box_center_world=None, box_rpy_world=None,
                   resolution=DEFAULT_RESOLUTION, spacing=DEFAULT_SPACING):
    """Return occupancy image rows, surface map, stats, and labelled GT cloud."""
    mapper = mapper_from_scene(config, resolution=resolution)
    if box_center_base is not None and box_size is not None:
        mark_placed_on_mapper(mapper, box_center_base, box_size, yaw=box_yaw or 0.0)
    fill_unoccupied_as_free(mapper)
    surface = mapper.surface_map_2d()
    image = occupancy_image_array(surface)
    walls, wall_c = sample_container_inner_cloud(config, spacing=spacing)
    points = list(walls)
    colors = list(wall_c)
    if box_center_world is not None and box_size is not None:
        box_pts, box_c = sample_box_cloud(
            box_center_world, box_size, box_rpy_world or [0.0, 0.0, 0.0],
            spacing=spacing)
        points.extend(box_pts)
        colors.extend(box_c)
    stats = mapper.stats()
    stats["n_gt_points"] = len(points)
    stats["n_wall_points"] = len(walls)
    stats["n_box_points"] = len(points) - len(walls)
    return {
        "occupancy_image": image,
        "surface_map": {
            "nx": surface["nx"],
            "ny": surface["ny"],
            "resolution": surface["resolution"],
            "inner_size": surface["inner_size"],
            "origin_local": surface.get("origin_local"),
            "center_base": surface.get("center_base"),
            "yaw": surface.get("yaw"),
            "height": surface["height"],
            "state": surface["state"],
            "occupancy_ratio": stats.get("occupancy_ratio"),
            "occupied_count": stats.get("occupied_count"),
            "committed_box_count": stats.get("committed_box_count"),
        },
        "stats": stats,
        "cloud_xyz": points,
        "cloud_rgb": colors,
    }


def box_rgb(catalog_id, index=0):
    catalog = CATALOG_RGB.get(str(catalog_id or "").lower())
    if catalog is not None:
        return catalog
    return BOX_PALETTE[int(index) % len(BOX_PALETTE)]


def sample_free_voxels_world(mapper, config, color=FREE_VOXEL_RGB):
    """FREE voxel centers in world (remaining interior volume)."""
    points = []
    colors = []
    for iz in range(mapper.nz):
        for iy in range(mapper.ny):
            for ix in range(mapper.nx):
                if mapper._grid[mapper._index(ix, iy, iz)] != FREE:
                    continue
                local = mapper._voxel_center_local(ix, iy, iz)
                base = mapper._local_to_world(*local)
                points.append(xyz_base_link_to_world(config, list(base)))
                colors.append(color)
    return points, colors


def _normalize_box_record(box, index=0):
    size = [float(v) for v in (box.get("size_wdh") or [0.0, 0.0, 0.0])]
    pose_world = box.get("pose_world") or {}
    position = [float(v) for v in (pose_world.get("position") or [0.0, 0.0, 0.0])]
    rpy = pose_world.get("rpy")
    if rpy is None:
        yaw = float(pose_world.get("yaw") or 0.0)
        rpy = [0.0, 0.0, yaw]
    else:
        rpy = [float(v) for v in rpy]
    pose_base = box.get("pose_base_link") or {}
    center_base = pose_base.get("position")
    yaw_base = pose_base.get("yaw")
    if yaw_base is None:
        yaw_base = float(rpy[2])
    return {
        "seq": box.get("seq", index),
        "commit_index": box.get("commit_index", index),
        "catalog_id": box.get("catalog_id") or "unknown",
        "spawn_id": box.get("spawn_id"),
        "gz_model": box.get("gz_model") or box.get("spawn_id"),
        "size_wdh": size,
        "mass_kg": float(box.get("mass_kg") or 0.0),
        "volume_m3": float(box.get("volume_m3") or (size[0] * size[1] * size[2])),
        "pose_world": {"position": position, "rpy": rpy, "yaw": float(rpy[2])},
        "pose_base_link": {
            "position": (
                [float(v) for v in center_base] if center_base is not None
                else None),
            "yaw": float(yaw_base),
        },
        "rgb": list(box_rgb(box.get("catalog_id"), index)),
    }


def build_pack_layout(config, boxes, resolution=DEFAULT_RESOLUTION,
                      spacing=DEFAULT_SPACING):
    """Interior walls + every committed box + remaining FREE voxels.

    ``boxes`` entries need ``size_wdh`` and ``pose_world.position``; mapper
    rasterization uses ``pose_base_link`` when present, else world→base.
    """
    from luggage_description.scene_tf_config_utils import (
        xyz_world_to_base_link, yaw_world_to_base_link,
    )
    records = [_normalize_box_record(box, i) for i, box in enumerate(boxes or [])]
    mapper = mapper_from_scene(config, resolution=resolution)
    for record in records:
        center_base = record["pose_base_link"]["position"]
        if center_base is None:
            center_base = xyz_world_to_base_link(
                config, record["pose_world"]["position"])
            record["pose_base_link"]["position"] = [float(v) for v in center_base]
            record["pose_base_link"]["yaw"] = yaw_world_to_base_link(
                config, record["pose_world"]["yaw"])
        mark_placed_on_mapper(
            mapper, center_base, record["size_wdh"],
            yaw=record["pose_base_link"]["yaw"])
    fill_unoccupied_as_free(mapper)
    surface = mapper.surface_map_2d()
    image = occupancy_image_array(surface)
    walls, wall_c = sample_container_inner_cloud(config, spacing=spacing)
    points = list(walls)
    colors = list(wall_c)
    n_box_points = 0
    for record in records:
        box_pts, _ignored = sample_box_cloud(
            record["pose_world"]["position"], record["size_wdh"],
            record["pose_world"]["rpy"], spacing=spacing)
        rgb = tuple(record["rgb"])
        points.extend(box_pts)
        colors.extend([rgb] * len(box_pts))
        n_box_points += len(box_pts)
    free_xyz, free_rgb = sample_free_voxels_world(mapper, config)
    stats = mapper.stats()
    stats["n_gt_points"] = len(points)
    stats["n_wall_points"] = len(walls)
    stats["n_box_points"] = n_box_points
    stats["n_free_voxels"] = len(free_xyz)
    stats["n_boxes"] = len(records)
    return {
        "boxes": records,
        "occupancy_image": image,
        "surface_map": {
            "nx": surface["nx"],
            "ny": surface["ny"],
            "resolution": surface["resolution"],
            "inner_size": surface["inner_size"],
            "origin_local": surface.get("origin_local"),
            "center_base": surface.get("center_base"),
            "yaw": surface.get("yaw"),
            "height": surface["height"],
            "state": surface["state"],
            "occupancy_ratio": stats.get("occupancy_ratio"),
            "occupied_count": stats.get("occupied_count"),
            "committed_box_count": stats.get("committed_box_count"),
        },
        "stats": stats,
        "cloud_xyz": points,
        "cloud_rgb": colors,
        "free_xyz": free_xyz,
        "free_rgb": free_rgb,
    }


def write_png_rgb(path, arr):
    import numpy as np
    arr = np.asarray(arr)
    if arr.dtype != np.uint8:
        arr = np.clip(arr, 0, 255).astype(np.uint8)
    if arr.ndim == 2:
        arr = arr[:, :, None]
    height, width, channels = arr.shape
    color_type = 2 if channels == 3 else 0
    raw = b"".join(b"\x00" + arr[row].tobytes() for row in range(height))

    def _chunk(tag, data):
        body = tag + data
        return (struct.pack(">I", len(data)) + body
                + struct.pack(">I", zlib.crc32(body) & 0xFFFFFFFF))

    png = (
        b"\x89PNG\r\n\x1a\n"
        + _chunk(b"IHDR", struct.pack(
            ">IIBBBBB", width, height, 8, color_type, 0, 0, 0))
        + _chunk(b"IDAT", zlib.compress(raw, 6))
        + _chunk(b"IEND", b"")
    )
    with open(path, "wb") as handle:
        handle.write(png)


def container_wireframe_world(config):
    """Usable 7-face hull edges in world."""
    return [
        (
            _container_link_to_world(config, a),
            _container_link_to_world(config, b),
        )
        for a, b in container_inner_hull_edges_in_container(config)
    ]


def container_aperture_wireframe_world(config):
    """Opening-aperture rectangle in world."""
    return [
        (
            _container_link_to_world(config, a),
            _container_link_to_world(config, b),
        )
        for a, b in container_aperture_edges_in_container(config)
    ]


_CUBE_TRI = (
    (0, 1, 2), (0, 2, 3),
    (4, 6, 5), (4, 7, 6),
    (0, 4, 5), (0, 5, 1),
    (3, 2, 6), (3, 6, 7),
    (0, 3, 7), (0, 7, 4),
    (1, 5, 6), (1, 6, 2),
)


def _box_mesh(center, size, rpy, rgb):
    w, d, h = [float(v) for v in size]
    hx, hy, hz = 0.5 * w, 0.5 * d, 0.5 * h
    locals_ = [
        [-hx, -hy, -hz], [hx, -hy, -hz], [hx, hy, -hz], [-hx, hy, -hz],
        [-hx, -hy, hz], [hx, -hy, hz], [hx, hy, hz], [-hx, hy, hz],
    ]
    xs, ys, zs = [], [], []
    for local in locals_:
        world, _ = _compose(center, rpy, local, [0.0, 0.0, 0.0])
        xs.append(world[0])
        ys.append(world[1])
        zs.append(world[2])
    i, j, k = zip(*_CUBE_TRI)
    return {
        "x": xs, "y": ys, "z": zs,
        "i": list(i), "j": list(j), "k": list(k),
        "color": "rgb(%d,%d,%d)" % tuple(rgb),
    }


def write_layout_html(path, boxes, edges, free_xyz, meta, aperture_edges=None):
    """Self-contained Plotly viewer (CDN). Open the HTML in a browser."""
    stride = max(1, int(math.ceil(len(free_xyz or []) / 2500.0)))
    free = list(free_xyz or [])[::stride]
    traces_boxes = []
    for record in boxes:
        mesh = _box_mesh(
            record["pose_world"]["position"], record["size_wdh"],
            record["pose_world"]["rpy"], record["rgb"])
        traces_boxes.append({
            "type": "mesh3d",
            "x": mesh["x"], "y": mesh["y"], "z": mesh["z"],
            "i": mesh["i"], "j": mesh["j"], "k": mesh["k"],
            "color": mesh["color"],
            "opacity": 0.88,
            "name": "seq%s %s" % (record.get("seq"), record.get("catalog_id")),
            "hovertext": "seq=%s %s %s" % (
                record.get("seq"), record.get("catalog_id"),
                record.get("size_wdh")),
            "showscale": False,
        })

    def _edge_xyz(segments):
        ex, ey, ez = [], [], []
        for a, b in segments or []:
            ex.extend([a[0], b[0], None])
            ey.extend([a[1], b[1], None])
            ez.extend([a[2], b[2], None])
        return {"x": ex, "y": ey, "z": ez}

    payload = {
        "boxes": traces_boxes,
        "edges": _edge_xyz(edges),
        "aperture": _edge_xyz(aperture_edges),
        "free": {
            "x": [p[0] for p in free],
            "y": [p[1] for p in free],
            "z": [p[2] for p in free],
        },
        "meta": meta,
    }
    html = """<!DOCTYPE html>
<html><head><meta charset="utf-8"/>
<title>Pack layout — %(title)s</title>
<script src="https://cdn.plot.ly/plotly-2.27.0.min.js"></script>
<style>
 body { margin:0; font-family: sans-serif; background:#111; color:#eee; }
 #plot { width:100vw; height:100vh; }
 .bar { position:absolute; top:8px; left:12px; z-index:2;
        background:rgba(0,0,0,.55); padding:8px 12px; border-radius:6px;
        font-size:13px; }
</style></head>
<body>
<div class="bar">%(title)s<br/>%(subtitle)s<br/>
Open <code>container_and_boxes.ply</code> / <code>interior_free.ply</code>
in CloudCompare if the CDN is blocked.</div>
<div id="plot"></div>
<script>
const DATA = %(payload)s;
const traces = DATA.boxes.slice();
traces.push({
  type: 'scatter3d', mode: 'lines',
  x: DATA.edges.x, y: DATA.edges.y, z: DATA.edges.z,
  line: {color: '#cccccc', width: 4}, name: 'container hull',
  hoverinfo: 'skip'
});
if (DATA.aperture.x.length) {
  traces.push({
    type: 'scatter3d', mode: 'lines',
    x: DATA.aperture.x, y: DATA.aperture.y, z: DATA.aperture.z,
    line: {color: '#f0d240', width: 6}, name: 'aperture',
    hoverinfo: 'skip'
  });
}
if (DATA.free.x.length) {
  traces.push({
    type: 'scatter3d', mode: 'markers',
    x: DATA.free.x, y: DATA.free.y, z: DATA.free.z,
    marker: {size: 2, color: '#48a8d2', opacity: 0.22},
    name: 'free interior',
    hoverinfo: 'skip'
  });
}
Plotly.newPlot('plot', traces, {
  paper_bgcolor: '#111', plot_bgcolor: '#111',
  scene: {
    aspectmode: 'data',
    xaxis: {title: 'x world', color: '#bbb', gridcolor: '#333'},
    yaxis: {title: 'y world', color: '#bbb', gridcolor: '#333'},
    zaxis: {title: 'z world', color: '#bbb', gridcolor: '#333'},
    camera: {eye: {x: 1.6, y: -1.8, z: 1.1}}
  },
  legend: {font: {color: '#eee'}},
  margin: {l: 0, r: 0, t: 0, b: 0}
}, {responsive: true});
</script>
</body></html>
""" % {
        "title": meta.get("termination") or "pack layout",
        "subtitle": "%s boxes · occupancy %.1f%% · free voxels %s" % (
            meta.get("n_boxes", len(boxes)),
            100.0 * float(meta.get("occupancy_ratio") or 0.0),
            meta.get("n_free_voxels", len(free_xyz or []))),
        "payload": json.dumps(payload),
    }
    with open(path, "w", encoding="utf-8") as handle:
        handle.write(html)


def write_pack_layout_dump(dest, config, boxes, termination="", extra_meta=None,
                           resolution=DEFAULT_RESOLUTION, spacing=DEFAULT_SPACING):
    """Write visualization artifacts when packing stops."""
    os.makedirs(dest, exist_ok=True)
    layout = build_pack_layout(
        config, boxes, resolution=resolution, spacing=spacing)
    boxes_out = layout["boxes"]
    stats = layout["stats"]
    meta = {
        "frame": "world",
        "termination": termination,
        "n_boxes": len(boxes_out),
        "occupancy_ratio": stats.get("occupancy_ratio"),
        "occupied_count": stats.get("occupied_count"),
        "free_count": stats.get("free_count"),
        "n_wall_points": stats.get("n_wall_points"),
        "n_box_points": stats.get("n_box_points"),
        "n_free_voxels": stats.get("n_free_voxels"),
        "files": [
            "boxes.json", "occupancy_gt.json", "occupancy_gt.png",
            "container_and_boxes.ply", "interior_free.ply", "layout.ply",
            "layout.html", "meta.json",
        ],
    }
    if extra_meta:
        meta.update(extra_meta)
    with open(os.path.join(dest, "boxes.json"), "w", encoding="utf-8") as handle:
        json.dump({"boxes": boxes_out, "termination": termination},
                  handle, indent=2, sort_keys=True)
        handle.write("\n")
    with open(os.path.join(dest, "occupancy_gt.json"), "w",
              encoding="utf-8") as handle:
        json.dump(layout["surface_map"], handle, indent=2, sort_keys=True)
        handle.write("\n")
    write_png_rgb(os.path.join(dest, "occupancy_gt.png"),
                  layout["occupancy_image"])
    write_ply_xyzrgb(
        os.path.join(dest, "container_and_boxes.ply"),
        layout["cloud_xyz"], layout["cloud_rgb"])
    write_ply_xyzrgb(
        os.path.join(dest, "interior_free.ply"),
        layout["free_xyz"], layout["free_rgb"])
    write_ply_xyzrgb(
        os.path.join(dest, "layout.ply"),
        list(layout["cloud_xyz"]) + list(layout["free_xyz"]),
        list(layout["cloud_rgb"]) + list(layout["free_rgb"]))
    write_layout_html(
        os.path.join(dest, "layout.html"),
        boxes_out, container_wireframe_world(config),
        layout["free_xyz"], meta,
        aperture_edges=container_aperture_wireframe_world(config))
    with open(os.path.join(dest, "meta.json"), "w", encoding="utf-8") as handle:
        json.dump(meta, handle, indent=2, sort_keys=True)
        handle.write("\n")
    return meta

