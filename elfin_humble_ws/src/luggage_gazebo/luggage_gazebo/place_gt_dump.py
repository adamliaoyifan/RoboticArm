#!/usr/bin/env python3
"""Geometry GT dumps after a successful place (no ROS).

Live cargo-map integration is slice B. This module rasterizes the same
``CargoVolumeMapper`` from scene_tf + the placed box AABB, then samples a
world-frame point cloud of the inner surfaces plus the box. That cloud is the
occupancy target: complete, labelled, not limited by the wrist camera FOV.
"""

from __future__ import division

import math
import struct

from luggage_description.scene_tf_config_utils import (
    _compose,
    container_in_base_link,
    container_inner_ceiling_z,
    container_inner_dimensions,
    container_inner_floor_z,
    container_usable_center_in_base_link,
    container_usable_dimensions,
    origin_in_world,
)
from luggage_perception.cargo_volume_mapper import (
    FREE,
    OCCUPIED,
    CargoVolumeMapper,
)


DEFAULT_RESOLUTION = 0.05
DEFAULT_SPACING = 0.03


def mapper_from_scene(config, resolution=DEFAULT_RESOLUTION):
    inner = container_usable_dimensions(config)
    _xyz, rpy = container_in_base_link(config)
    return CargoVolumeMapper(
        inner_size=inner,
        center_base=container_usable_center_in_base_link(config),
        yaw=float(rpy[2]),
        resolution=float(resolution),
    )


def mark_placed_on_mapper(mapper, center_base, size_wdh, yaw=0.0):
    mapper.mark_placed_box(
        [float(v) for v in center_base],
        [float(v) for v in size_wdh],
        yaw=float(yaw),
    )
    return mapper


def fill_unoccupied_as_free(mapper):
    """Geometry GT: empty usable volume is known-free, not unknown."""
    grid = mapper._grid
    for i, cell in enumerate(grid):
        if cell != OCCUPIED:
            grid[i] = FREE


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


def sample_container_inner_cloud(config, spacing=DEFAULT_SPACING):
    """Floor + four inner walls in world. Opening face (-X) included."""
    inner_l, inner_w, _span = container_usable_dimensions(config)
    floor_z = container_inner_floor_z(config)
    ceiling_z = container_inner_ceiling_z(config)
    hx, hy = 0.5 * inner_l, 0.5 * inner_w
    xs = _grid_1d(-hx, hx, spacing)
    ys = _grid_1d(-hy, hy, spacing)
    zs = _grid_1d(floor_z, ceiling_z, spacing)
    points = []
    colors = []
    wall = (160, 160, 170)
    floor_c = (90, 90, 100)
    for x in xs:
        for y in ys:
            points.append(_container_link_to_world(config, [x, y, floor_z]))
            colors.append(floor_c)
    for z in zs:
        for y in ys:
            points.append(_container_link_to_world(config, [-hx, y, z]))
            colors.append(wall)
            points.append(_container_link_to_world(config, [hx, y, z]))
            colors.append(wall)
        for x in xs:
            points.append(_container_link_to_world(config, [x, -hy, z]))
            colors.append(wall)
            points.append(_container_link_to_world(config, [x, hy, z]))
            colors.append(wall)
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
