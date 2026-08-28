"""Build watertight visual mesh and collision primitives from parametric model."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import open3d as o3d


@dataclass
class CadResult:
    visual_path: Path
    collision_path: Path
    collision_boxes: list[dict[str, Any]]
    debug_path: Path


def _box_mesh(center: np.ndarray, size: np.ndarray) -> o3d.geometry.TriangleMesh:
    return o3d.geometry.TriangleMesh.create_box(
        width=float(size[0]), height=float(size[1]), depth=float(size[2])
    ).translate(center - size / 2)


def _extrude_polygon_xz(
    polygon_xz: list[list[float]],
    y_min: float,
    depth_y: float,
    opening: dict[str, Any] | None,
) -> o3d.geometry.TriangleMesh:
    poly = np.asarray(polygon_xz, dtype=np.float64)
    if poly.shape[0] < 3:
        return o3d.geometry.TriangleMesh()

    n = len(poly)
    y0, y1 = float(y_min), float(y_min + depth_y)
    bottom = np.column_stack([poly[:, 0], np.full(n, y0), poly[:, 1]])
    top = np.column_stack([poly[:, 0], np.full(n, y1), poly[:, 1]])
    verts = np.vstack([bottom, top])
    tris = []
    for i in range(n):
        j = (i + 1) % n
        bi, bj, ti, tj = i, j, i + n, j + n
        tris.append([bi, bj, tj])
        tris.append([bi, tj, ti])
    for i in range(1, n - 1):
        tris.append([0, i, i + 1])
        tris.append([n, n + i + 1, n + i])

    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(verts)
    mesh.triangles = o3d.utility.Vector3iVector(np.asarray(tris, dtype=np.int32))
    mesh.compute_vertex_normals()

    if opening and opening.get("enabled"):
        mesh = _apply_opening_cut(mesh, opening, y1)
    return mesh


def _apply_opening_cut(
    mesh: o3d.geometry.TriangleMesh,
    opening: dict[str, Any],
    y_front: float,
) -> o3d.geometry.TriangleMesh:
    from cargo_mesh_opening import apply_opening_cut

    return apply_opening_cut(mesh, opening, y_front)


def _leg_meshes(legs: list[dict[str, Any]]) -> list[o3d.geometry.TriangleMesh]:
    meshes = []
    for leg in legs:
        c = np.asarray(leg["center"], dtype=np.float64)
        s = np.asarray(leg["size"], dtype=np.float64)
        h = float(leg.get("height", s[2]))
        size = np.array([max(s[0], 0.05), max(s[1], 0.05), max(h, 0.05)])
        center = c.copy()
        center[2] = size[2] / 2
        meshes.append(_box_mesh(center, size))
    return meshes


def _collision_boxes_from_model(model: dict[str, Any]) -> list[dict[str, Any]]:
    body = model["body"]
    poly = np.asarray(body["polygon_xz"], dtype=np.float64)
    mn = poly.min(axis=0)
    mx = poly.max(axis=0)
    z_mid = 0.5 * (mn[1] + mx[1])
    y_min = float(body["y_min"])
    depth = float(body["depth_y"])
    y_mid = y_min + depth / 2
    boxes = [
        {
            "name": "body_main",
            "center": [0.5 * (mn[0] + mx[0]), y_mid, z_mid],
            "size": [mx[0] - mn[0], depth, mx[1] - mn[1]],
        }
    ]
    opening = model.get("opening", {})
    if opening.get("enabled"):
        ox0, ox1 = float(opening["x_min"]), float(opening["x_max"])
        oz0, oz1 = float(opening["z_min"]), float(opening["z_max"])
        ow = ox1 - ox0
        oh = oz1 - oz0
        y_front = y_min + depth
        boxes.extend(
            [
                {
                    "name": "lintel",
                    "center": [0.5 * (ox0 + ox1), y_front - 0.03, oz1 + 0.05],
                    "size": [ow, 0.06, 0.1],
                },
                {
                    "name": "sill",
                    "center": [0.5 * (ox0 + ox1), y_front - 0.03, oz0 - 0.05],
                    "size": [ow, 0.06, 0.1],
                },
                {
                    "name": "left_pillar",
                    "center": [ox0 - 0.05, y_front - 0.03, 0.5 * (oz0 + oz1)],
                    "size": [0.1, 0.06, oh],
                },
                {
                    "name": "right_pillar",
                    "center": [ox1 + 0.05, y_front - 0.03, 0.5 * (oz0 + oz1)],
                    "size": [0.1, 0.06, oh],
                },
            ]
        )
    for leg in model.get("legs", []):
        c = leg["center"]
        s = leg["size"]
        boxes.append(
            {
                "name": f"leg_{leg['id']}",
                "center": c,
                "size": s,
                "inferred": leg.get("inferred", False),
            }
        )
    return boxes


def _mesh_from_boxes(boxes: list[dict[str, Any]]) -> o3d.geometry.TriangleMesh:
    meshes = []
    for b in boxes:
        c = np.asarray(b["center"], dtype=np.float64)
        s = np.asarray(b["size"], dtype=np.float64)
        meshes.append(_box_mesh(c, s))
    return _merge_meshes(meshes)


def _merge_meshes(meshes: list[o3d.geometry.TriangleMesh]) -> o3d.geometry.TriangleMesh:
    valid = [m for m in meshes if len(m.triangles) > 0]
    if not valid:
        return o3d.geometry.TriangleMesh()
    out = valid[0]
    for m in valid[1:]:
        out += m
    out.compute_vertex_normals()
    return out


def _parametric_debug_pointcloud(model: dict[str, Any]) -> np.ndarray:
    pts = []
    for p in model["body"]["polygon_xz"]:
        pts.append([p[0], model["body"]["y_min"], p[1]])
        pts.append([p[0], model["body"]["y_min"] + model["body"]["depth_y"], p[1]])
    for leg in model.get("legs", []):
        c = np.asarray(leg["center"])
        s = np.asarray(leg["size"]) * 0.5
        for dx in (-1, 1):
            for dy in (-1, 1):
                for dz in (-1, 1):
                    pts.append(c + np.array([dx, dy, dz]) * s)
    return np.asarray(pts, dtype=np.float64)


def build_cad_model(model: dict[str, Any], output_dir: Path) -> CadResult:
    body_mesh = _extrude_polygon_xz(
        model["body"]["polygon_xz"],
        float(model["body"]["y_min"]),
        float(model["body"]["depth_y"]),
        model.get("opening"),
    )
    leg_mesh_list = _leg_meshes(model.get("legs", []))
    visual = _merge_meshes([body_mesh, *leg_mesh_list])

    collision_boxes = _collision_boxes_from_model(model)
    collision = _mesh_from_boxes(collision_boxes)

    mesh_dir = output_dir / "meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)
    visual_path = mesh_dir / "container_visual.stl"
    collision_path = mesh_dir / "container_collision.stl"
    debug_path = output_dir / "parametric_debug.ply"

    o3d.io.write_triangle_mesh(str(visual_path), visual)
    o3d.io.write_triangle_mesh(str(collision_path), collision)

    dbg = _parametric_debug_pointcloud(model)
    o3d.io.write_point_cloud(
        str(debug_path), o3d.geometry.PointCloud(o3d.utility.Vector3dVector(dbg))
    )

    import json

    with (output_dir / "collision_boxes.json").open("w", encoding="utf-8") as fh:
        json.dump(collision_boxes, fh, indent=2)

    model["collision_boxes"] = collision_boxes
    return CadResult(
        visual_path=visual_path,
        collision_path=collision_path,
        collision_boxes=collision_boxes,
        debug_path=debug_path,
    )
