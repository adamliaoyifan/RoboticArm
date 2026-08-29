"""Opening cut on triangle meshes (+Y facade) shared by CAD and surface reconstruction."""

from __future__ import annotations

from typing import Any

import numpy as np
import open3d as o3d


def y_front_from_body(body_y_min: float, body_depth_y: float) -> float:
    return float(body_y_min) + float(body_depth_y)


def resolve_opening(
    opening_cfg: dict[str, Any],
    parametric_model: dict[str, Any] | None,
) -> dict[str, Any] | None:
    """Return opening dict with x_min/x_max/z_min/z_max and enabled, or None."""
    manual = opening_cfg.get("manual")
    if manual:
        out = {**manual, "enabled": True}
        for key in ("x_min", "x_max", "z_min", "z_max"):
            if key not in out:
                raise KeyError(f"opening.manual must include {key}")
        return out

    if not opening_cfg.get("use_parametric", True) or parametric_model is None:
        return None

    opening = parametric_model.get("opening") or {}
    if not opening.get("enabled"):
        return None
    for key in ("x_min", "x_max", "z_min", "z_max"):
        if key not in opening:
            return None
    return opening


def apply_opening_cut(
    mesh: o3d.geometry.TriangleMesh,
    opening: dict[str, Any],
    y_front: float,
    margin_y: float = 0.08,
) -> o3d.geometry.TriangleMesh:
    """Remove triangles whose centroid falls in the opening box on the +Y face."""
    verts = np.asarray(mesh.vertices)
    tris = np.asarray(mesh.triangles)
    if tris.size == 0:
        return mesh
    centroids = verts[tris].mean(axis=1)
    x0, x1 = float(opening["x_min"]), float(opening["x_max"])
    z0, z1 = float(opening["z_min"]), float(opening["z_max"])
    in_hole = (
        (centroids[:, 0] >= x0)
        & (centroids[:, 0] <= x1)
        & (centroids[:, 2] >= z0)
        & (centroids[:, 2] <= z1)
        & (centroids[:, 1] > y_front - margin_y)
    )
    keep = ~in_hole
    if keep.sum() < 10:
        return mesh
    mesh.remove_triangles_by_mask(~keep)
    mesh.remove_unreferenced_vertices()
    mesh.compute_vertex_normals()
    return mesh
