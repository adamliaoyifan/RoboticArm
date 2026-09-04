"""Quality checks before exporting cargo mesh to STL / Gazebo."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np
import open3d as o3d


@dataclass
class MeshQAResult:
    passed: bool
    messages: list[str]


@dataclass
class CoverageQAResult:
    passed: bool
    messages: list[str]
    coverage_ratio: float
    median_distance_m: float


def validate_mesh(
    mesh: o3d.geometry.TriangleMesh,
    cfg: dict[str, Any],
    outer: dict[str, float] | None = None,
) -> MeshQAResult:
    msgs: list[str] = []
    verts = np.asarray(mesh.vertices)
    if verts.shape[0] < 100:
        return MeshQAResult(False, ["mesh has too few vertices"])

    mn, mx = verts.min(axis=0), verts.max(axis=0)
    span = mx - mn
    min_h = float(cfg.get("min_height_span", 1.0))
    if float(span[2]) < min_h:
        msgs.append(f"mesh Z span {span[2]:.3f} m < {min_h} m (flat sheet?)")

    if outer:
        ref = np.array([outer.get("length", 2.4), outer.get("width", 2.0), outer.get("height", 2.2)])
        meas = np.array([span[0], span[1], span[2]])
        max_scale = float(cfg.get("max_outer_scale", 1.75))
        min_scale = float(cfg.get("min_outer_scale", 0.4))
        if np.any(meas > ref * max_scale) or np.any(meas < ref * min_scale):
            msgs.append(
                f"mesh AABB {meas.round(3).tolist()} outside "
                f"[{min_scale}x, {max_scale}x] of outer {ref.round(3).tolist()}"
            )

    mesh.compute_triangle_normals()
    normals = np.asarray(mesh.triangle_normals)
    if normals.shape[0]:
        vertical = np.abs(normals[:, 2]) < 0.35
        vert_ratio = float(vertical.mean())
        if vert_ratio < float(cfg.get("min_vertical_face_ratio", 0.08)):
            msgs.append(
                f"vertical face ratio {vert_ratio:.2%} too low (mostly horizontal — carpet mesh?)"
            )

    if float(span[2]) > 0 and float(span[0]) / float(span[2]) > 8:
        msgs.append("mesh is very wide vs tall — check alignment")

    passed = len(msgs) == 0
    if passed:
        msgs.append(
            f"OK: AABB min={mn.round(3).tolist()} max={mx.round(3).tolist()} "
            f"span={span.round(3).tolist()}"
        )
    return MeshQAResult(passed, msgs)


def validate_point_coverage(
    mesh: o3d.geometry.TriangleMesh,
    reference_xyz: np.ndarray,
    cfg: dict[str, Any],
) -> CoverageQAResult:
    """Fraction of reference points within max distance of the reconstructed surface."""
    msgs: list[str] = []
    if reference_xyz.shape[0] < 50:
        return CoverageQAResult(
            True,
            ["skip coverage: too few reference points"],
            1.0,
            0.0,
        )

    max_dist = float(cfg.get("max_point_to_mesh_m", 0.04))
    min_ratio = float(cfg.get("min_coverage_ratio", 0.85))
    sample_n = int(cfg.get("coverage_sample_points", 8000))
    band = float(cfg.get("coverage_exterior_band", 0.10))

    xyz = reference_xyz
    if band > 0:
        mn, mx = xyz.min(axis=0), xyz.max(axis=0)
        on_shell = (
            (xyz[:, 0] <= mn[0] + band)
            | (xyz[:, 0] >= mx[0] - band)
            | (xyz[:, 1] <= mn[1] + band)
            | (xyz[:, 1] >= mx[1] - band)
            | (xyz[:, 2] <= mn[2] + band)
            | (xyz[:, 2] >= mx[2] - band)
        )
        shell = xyz[on_shell]
        if shell.shape[0] >= 200:
            xyz = shell

    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))
    if xyz.shape[0] > sample_n:
        pcd = pcd.random_down_sample(min(1.0, sample_n / xyz.shape[0]))

    n_mesh_samples = int(cfg.get("coverage_mesh_samples", 50000))
    mesh_pcd = mesh.sample_points_uniformly(
        number_of_points=min(n_mesh_samples, max(5000, len(mesh.triangles) * 3))
    )
    dists = np.asarray(pcd.compute_point_cloud_distance(mesh_pcd), dtype=np.float64)

    ratio = float((dists < max_dist).mean())
    median_d = float(np.median(dists))
    passed = ratio >= min_ratio
    if passed:
        msgs.append(
            f"Coverage OK: {ratio:.1%} of points within {max_dist:.3f} m "
            f"(median dist {median_d:.4f} m)"
        )
    else:
        msgs.append(
            f"Coverage FAIL: {ratio:.1%} within {max_dist:.3f} m "
            f"(need >= {min_ratio:.0%}, median {median_d:.4f} m)"
        )
    return CoverageQAResult(passed, msgs, ratio, median_d)


def log_stl_aabb(path: Any) -> tuple[np.ndarray, np.ndarray]:
    mesh = o3d.io.read_triangle_mesh(str(path))
    verts = np.asarray(mesh.vertices)
    if verts.size == 0:
        raise ValueError(f"empty STL: {path}")
    mn, mx = verts.min(axis=0), verts.max(axis=0)
    print(f"STL AABB {path.name}: min={mn.round(4).tolist()} max={mx.round(4).tolist()}")
    return mn, mx
