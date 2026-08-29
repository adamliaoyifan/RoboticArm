"""Legacy: BPA/Poisson surface mesh. Superseded by cargo_cad_model.py (not used by measure_cargo_from_las.py)."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import open3d as o3d

from cargo_io import numpy_to_pcd
from cargo_mesh_qa import MeshQAResult, validate_mesh


@dataclass
class MeshResult:
    visual_path: Path
    collision_path: Path
    visual_triangles: int
    collision_triangles: int
    qa: MeshQAResult


def _estimate_normals(pcd: o3d.geometry.PointCloud, cfg: dict[str, Any]) -> o3d.geometry.PointCloud:
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=float(cfg.get("normal_radius", 0.08)),
            max_nn=int(cfg.get("normal_max_nn", 30)),
        )
    )
    try:
        pcd.orient_normals_consistent_tangent_plane(50)
    except RuntimeError:
        pcd.orient_normals_towards_camera_location(
            camera_location=np.array([0.0, 0.0, 10.0])
        )
    return pcd


def _mask_under_deck(
    body_xyz: np.ndarray,
    floor_deck_z: float | None,
    leg_margin: float,
) -> np.ndarray:
    if floor_deck_z is None:
        return body_xyz
    z_lo = float(floor_deck_z) + leg_margin
    keep = body_xyz[:, 2] >= z_lo
    if keep.sum() < 100:
        return body_xyz
    return body_xyz[keep]


def _cluster_by_planes(
    xyz: np.ndarray,
    planes: list[dict[str, Any]],
    dist: float,
) -> list[np.ndarray]:
    if not planes or xyz.shape[0] < 200:
        return [xyz]
    remaining = np.ones(xyz.shape[0], dtype=bool)
    clusters: list[np.ndarray] = []
    for plane in planes:
        if remaining.sum() < 100:
            break
        n = np.asarray(plane["normal"], dtype=np.float64)
        n /= np.linalg.norm(n) + 1e-12
        d = float(plane["d"])
        pts = xyz[remaining]
        sd = np.abs(pts @ n + d)
        inlier = sd < dist
        if inlier.sum() < 80:
            continue
        idx = np.where(remaining)[0][inlier]
        clusters.append(xyz[idx])
        rem_idx = np.where(remaining)[0]
        remaining[rem_idx[inlier]] = False
    if remaining.sum() >= 100:
        clusters.append(xyz[remaining])
    return clusters if clusters else [xyz]


def _reconstruct_poisson(pcd: o3d.geometry.PointCloud, depth: int, quantile: float) -> o3d.geometry.TriangleMesh:
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=depth)
    densities = np.asarray(densities)
    if densities.size:
        thresh = np.quantile(densities, quantile)
        mesh.remove_vertices_by_mask(densities < thresh)
    mesh.compute_vertex_normals()
    return mesh


def _reconstruct_bpa(pcd: o3d.geometry.PointCloud) -> o3d.geometry.TriangleMesh:
    dists = pcd.compute_nearest_neighbor_distance()
    avg = float(np.mean(dists)) if len(dists) else 0.02
    avg = max(avg, 0.005)
    radii = [avg, avg * 2, avg * 4]
    return o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        pcd, o3d.utility.DoubleVector(radii)
    )


def _reconstruct_cloud(
    xyz: np.ndarray,
    cfg: dict[str, Any],
) -> o3d.geometry.TriangleMesh:
    if xyz.shape[0] < 50:
        return o3d.geometry.TriangleMesh()
    pcd = _estimate_normals(numpy_to_pcd(xyz), cfg)
    method = cfg.get("reconstruction", "bpa")
    if method == "poisson" and cfg.get("fill_closed_body", False):
        mesh = _reconstruct_poisson(
            pcd,
            int(cfg.get("poisson_depth", 9)),
            float(cfg.get("poisson_density_quantile", 0.05)),
        )
    else:
        mesh = _reconstruct_bpa(pcd)
        if len(mesh.triangles) == 0 and method != "bpa":
            mesh = _reconstruct_poisson(pcd, int(cfg.get("poisson_depth", 9)), 0.05)
    mesh.compute_vertex_normals()
    return mesh


def _merge_meshes(meshes: list[o3d.geometry.TriangleMesh]) -> o3d.geometry.TriangleMesh:
    valid = [m for m in meshes if len(m.triangles) > 0]
    if not valid:
        return o3d.geometry.TriangleMesh()
    combined = valid[0]
    for m in valid[1:]:
        combined += m
    combined.compute_vertex_normals()
    return combined


def _decimate(mesh: o3d.geometry.TriangleMesh, target: int) -> o3d.geometry.TriangleMesh:
    n = len(mesh.triangles)
    if n <= target or n == 0:
        return mesh
    try:
        return mesh.simplify_quadric_decimation(target_number_of_triangles=target)
    except Exception:
        span = float(mesh.get_max_bound().max() - mesh.get_min_bound().min())
        return mesh.simplify_vertex_clustering(voxel_size=max(span * 0.01, 0.01))


def build_mesh(
    body_xyz: np.ndarray,
    legs_xyz: np.ndarray,
    features: dict[str, Any] | None,
    cfg: dict[str, Any],
    output_dir: Path,
    outer: dict[str, float] | None = None,
) -> MeshResult:
    floor_deck_z = None
    if features:
        floor_deck_z = features.get("floor_deck_z")
    body_use = body_xyz
    if cfg.get("mask_under_deck", True):
        body_use = _mask_under_deck(
            body_xyz,
            floor_deck_z,
            float(cfg.get("under_deck_margin", 0.03)),
        )

    meshes: list[o3d.geometry.TriangleMesh] = []
    planes = (features or {}).get("planes", [])
    if cfg.get("segment_by_planes", True) and len(planes) >= 2:
        clusters = _cluster_by_planes(
            body_use, planes, float(cfg.get("plane_cluster_distance", 0.04))
        )
        for cluster in clusters:
            m = _reconstruct_cloud(cluster, cfg)
            if len(m.triangles):
                meshes.append(m)
    else:
        m = _reconstruct_cloud(body_use, cfg)
        if len(m.triangles):
            meshes.append(m)

    if legs_xyz.shape[0] >= 50:
        m_leg = _reconstruct_cloud(legs_xyz, cfg)
        if len(m_leg.triangles):
            meshes.append(m_leg)

    mesh = _merge_meshes(meshes)
    verts = np.asarray(mesh.vertices)
    if verts.size:
        z0 = float(verts[:, 2].min())
        if z0 < -0.001:
            verts = verts.copy()
            verts[:, 2] -= z0
            mesh.vertices = o3d.utility.Vector3dVector(verts)
    qa = validate_mesh(mesh, cfg, outer)
    if not qa.passed:
        raise RuntimeError("Mesh QA failed:\n  " + "\n  ".join(qa.messages))

    mesh_dir = output_dir / "meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)
    visual = _decimate(mesh, int(cfg.get("visual_triangles", 80000)))
    collision = _decimate(mesh, int(cfg.get("collision_triangles", 4000)))

    visual_path = mesh_dir / "container_visual.stl"
    collision_path = mesh_dir / "container_collision.stl"
    o3d.io.write_triangle_mesh(str(visual_path), visual)
    o3d.io.write_triangle_mesh(str(collision_path), collision)

    return MeshResult(
        visual_path=visual_path,
        collision_path=collision_path,
        visual_triangles=len(visual.triangles),
        collision_triangles=len(collision.triangles),
        qa=qa,
    )
