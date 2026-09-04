"""Point-cloud surface reconstruction for Gazebo visual STL (standalone pipeline)."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import open3d as o3d

from cargo_io import load_point_cloud, numpy_to_pcd
from cargo_mesh_opening import apply_opening_cut, resolve_opening, y_front_from_body
from cargo_mesh_qa import MeshQAResult, validate_mesh, validate_point_coverage


@dataclass
class SurfaceMeshResult:
    visual_path: Path
    debug_path: Path | None
    visual_triangles: int
    qa: MeshQAResult
    coverage: dict[str, float]


def _load_ply_xyz(path: Path) -> np.ndarray:
    xyz, _, _ = load_point_cloud(path)
    return xyz


def _voxel_downsample(xyz: np.ndarray, voxel_size: float) -> np.ndarray:
    if voxel_size <= 0 or xyz.shape[0] < 100:
        return xyz
    pcd = numpy_to_pcd(xyz)
    ds = pcd.voxel_down_sample(voxel_size)
    return np.asarray(ds.points, dtype=np.float64)


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


def _planes_list(planes_data: dict[str, Any]) -> list[dict[str, Any]]:
    planes: list[dict[str, Any]] = []
    for key in ("vertical", "horizontal"):
        for p in planes_data.get(key, []) or []:
            if "normal" in p and "d" in p:
                planes.append(p)
    return planes


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


def _estimate_normals(pcd: o3d.geometry.PointCloud, cfg: dict[str, Any]) -> o3d.geometry.PointCloud:
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=float(cfg.get("normal_radius", 0.08)),
            max_nn=int(cfg.get("normal_max_nn", 30)),
        )
    )
    try:
        pcd.orient_normals_consistent_tangent_plane(int(cfg.get("tangent_plane_k", 50)))
    except RuntimeError:
        pcd.orient_normals_towards_camera_location(
            camera_location=np.array([0.0, 0.0, 10.0])
        )
    return pcd


def _median_nn_distance(pcd: o3d.geometry.PointCloud) -> float:
    dists = pcd.compute_nearest_neighbor_distance()
    if len(dists) == 0:
        return 0.02
    return float(np.median(dists))


def _reconstruct_bpa(pcd: o3d.geometry.PointCloud, cfg: dict[str, Any]) -> o3d.geometry.TriangleMesh:
    dists = pcd.compute_nearest_neighbor_distance()
    avg = float(np.mean(dists)) if len(dists) else 0.02
    avg = max(avg, 0.005)
    scale = float(cfg.get("bpa_radius_scale", 1.0))
    radii = [avg * scale, avg * 2 * scale, avg * 4 * scale]
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        pcd, o3d.utility.DoubleVector(radii)
    )
    mesh.compute_vertex_normals()
    return mesh


def _reconstruct_alpha(
    pcd: o3d.geometry.PointCloud, cfg: dict[str, Any]
) -> o3d.geometry.TriangleMesh:
    alpha_scale = float(cfg.get("alpha_scale", 2.5))
    alpha = max(_median_nn_distance(pcd) * alpha_scale, 0.02)
    try:
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
    except RuntimeError:
        return o3d.geometry.TriangleMesh()
    mesh.compute_vertex_normals()
    return mesh


def _reconstruct_cluster(
    xyz: np.ndarray,
    cfg: dict[str, Any],
) -> o3d.geometry.TriangleMesh:
    if xyz.shape[0] < 50:
        return o3d.geometry.TriangleMesh()
    pcd = _estimate_normals(numpy_to_pcd(xyz), cfg)
    method = cfg.get("reconstruction", "bpa_then_alpha")
    mesh = o3d.geometry.TriangleMesh()
    if method in ("bpa", "bpa_then_alpha"):
        mesh = _reconstruct_bpa(pcd, cfg)
    if len(mesh.triangles) == 0 and method in ("alpha", "bpa_then_alpha"):
        mesh = _reconstruct_alpha(pcd, cfg)
    return mesh


def _merge_meshes(meshes: list[o3d.geometry.TriangleMesh]) -> o3d.geometry.TriangleMesh:
    valid = [m for m in meshes if len(m.triangles) > 0]
    if not valid:
        return o3d.geometry.TriangleMesh()
    out = valid[0]
    for m in valid[1:]:
        out += m
    out.remove_duplicated_vertices()
    out.remove_degenerate_triangles()
    out.compute_vertex_normals()
    return out


def _crop_to_aabb(
    mesh: o3d.geometry.TriangleMesh,
    mn: np.ndarray,
    mx: np.ndarray,
    margin: float,
) -> o3d.geometry.TriangleMesh:
    lo = mn - margin
    hi = mx + margin
    verts = np.asarray(mesh.vertices)
    tris = np.asarray(mesh.triangles)
    if tris.size == 0:
        return mesh
    centroids = verts[tris].mean(axis=1)
    inside = np.all(centroids >= lo, axis=1) & np.all(centroids <= hi, axis=1)
    if inside.sum() < 10:
        return mesh
    mesh.remove_triangles_by_mask(~inside)
    mesh.remove_unreferenced_vertices()
    mesh.compute_vertex_normals()
    return mesh


def _decimate(mesh: o3d.geometry.TriangleMesh, target: int) -> o3d.geometry.TriangleMesh:
    n = len(mesh.triangles)
    if n <= target or n == 0:
        return mesh
    try:
        return mesh.simplify_quadric_decimation(target_number_of_triangles=target)
    except Exception:
        span = float(mesh.get_max_bound().max() - mesh.get_min_bound().min())
        return mesh.simplify_vertex_clustering(voxel_size=max(span * 0.01, 0.01))


def _colored_cluster_meshes(
    clusters: list[np.ndarray],
    cfg: dict[str, Any],
) -> list[o3d.geometry.TriangleMesh]:
    meshes: list[o3d.geometry.TriangleMesh] = []
    palette = [
        [0.9, 0.2, 0.2],
        [0.2, 0.7, 0.3],
        [0.2, 0.4, 0.9],
        [0.9, 0.7, 0.1],
        [0.7, 0.2, 0.8],
        [0.3, 0.8, 0.8],
    ]
    for i, cluster in enumerate(clusters):
        m = _reconstruct_cluster(cluster, cfg)
        if len(m.triangles) == 0:
            continue
        color = palette[i % len(palette)]
        m.paint_uniform_color(color)
        meshes.append(m)
    return meshes


def _floor_deck_z(z_layers_path: Path | None) -> float | None:
    if z_layers_path is None or not z_layers_path.exists():
        return None
    data = json.loads(z_layers_path.read_text(encoding="utf-8"))
    return data.get("deck_bottom_z")


def build_surface_visual_mesh(
    output_dir: Path,
    cfg: dict[str, Any],
) -> SurfaceMeshResult:
    out = Path(output_dir)
    mesh_cfg = cfg.get("mesh", {})
    qa_cfg = cfg.get("qa", {})
    opening_cfg = cfg.get("opening", {})

    body_path = out / cfg.get("body_ply", "body_points.ply")
    leg_path = out / cfg.get("leg_ply", "leg_points.ply")
    object_path = out / cfg.get("object_ply", "object_points.ply")
    planes_path = out / cfg.get("planes_json", "planes.json")
    param_path = out / cfg.get("parametric_model_json", "parametric_model.json")
    z_layers_path = out / cfg.get("z_layers_json", "z_layers.json")

    if not body_path.exists():
        raise FileNotFoundError(f"Missing body point cloud: {body_path}")

    body_xyz = _load_ply_xyz(body_path)
    reference_xyz = body_xyz.copy()

    if mesh_cfg.get("mask_under_deck", False):
        body_xyz = _mask_under_deck(
            body_xyz,
            _floor_deck_z(z_layers_path if z_layers_path.exists() else None),
            float(mesh_cfg.get("under_deck_margin", 0.03)),
        )

    voxel_size = float(mesh_cfg.get("voxel_size", 0.02))
    body_xyz = _voxel_downsample(body_xyz, voxel_size)

    leg_xyz = np.empty((0, 3))
    if mesh_cfg.get("include_legs", True) and leg_path.exists():
        leg_xyz = _voxel_downsample(_load_ply_xyz(leg_path), voxel_size)

    if mesh_cfg.get("include_object_points", False) and object_path.exists():
        obj_xyz = _voxel_downsample(_load_ply_xyz(object_path), voxel_size)
        body_xyz = np.vstack([body_xyz, obj_xyz]) if body_xyz.size else obj_xyz

    ref_mn, ref_mx = reference_xyz.min(axis=0), reference_xyz.max(axis=0)
    outer = {
        "length": float(ref_mx[0] - ref_mn[0]),
        "width": float(ref_mx[1] - ref_mn[1]),
        "height": float(ref_mx[2] - ref_mn[2]),
    }

    planes_data: dict[str, Any] = {}
    if planes_path.exists():
        planes_data = json.loads(planes_path.read_text(encoding="utf-8"))

    parametric_model = None
    if param_path.exists():
        parametric_model = json.loads(param_path.read_text(encoding="utf-8"))

    cluster_meshes: list[o3d.geometry.TriangleMesh] = []
    debug_clusters: list[np.ndarray] = []

    if mesh_cfg.get("segment_by_planes", True):
        planes = _planes_list(planes_data)
        body_clusters = _cluster_by_planes(
            body_xyz,
            planes,
            float(mesh_cfg.get("plane_cluster_distance", 0.04)),
        )
        debug_clusters.extend(body_clusters)
        cluster_meshes.extend(_colored_cluster_meshes(body_clusters, mesh_cfg))
        if mesh_cfg.get("full_body_fallback", True):
            m_full = _reconstruct_cluster(body_xyz, mesh_cfg)
            if len(m_full.triangles):
                cluster_meshes.append(m_full)
    else:
        m = _reconstruct_cluster(body_xyz, mesh_cfg)
        if len(m.triangles):
            cluster_meshes.append(m)

    if leg_xyz.shape[0] >= 50:
        m_leg = _reconstruct_cluster(leg_xyz, mesh_cfg)
        if len(m_leg.triangles):
            cluster_meshes.append(m_leg)

    mesh = _merge_meshes(cluster_meshes)
    if len(mesh.triangles) == 0:
        raise RuntimeError(
            "Surface reconstruction produced empty mesh; check body_points.ply and mesh config"
        )

    pts_mn, pts_mx = body_xyz.min(axis=0), body_xyz.max(axis=0)
    if leg_xyz.shape[0]:
        pts_mn = np.minimum(pts_mn, leg_xyz.min(axis=0))
        pts_mx = np.maximum(pts_mx, leg_xyz.max(axis=0))

    mesh = _crop_to_aabb(mesh, pts_mn, pts_mx, float(mesh_cfg.get("crop_margin", 0.05)))

    opening = resolve_opening(opening_cfg, parametric_model)
    if mesh_cfg.get("opening_cut", True) and opening is not None:
        body = (parametric_model or {}).get("body", {})
        y_min = float(body.get("y_min", pts_mn[1]))
        depth_y = float(body.get("depth_y", pts_mx[1] - pts_mn[1]))
        y_front = y_front_from_body(y_min, depth_y)
        mesh = apply_opening_cut(
            mesh,
            opening,
            y_front,
            margin_y=float(mesh_cfg.get("opening_margin_y", 0.08)),
        )

    z_shift = 0.0
    verts = np.asarray(mesh.vertices)
    if verts.size:
        z0 = float(verts[:, 2].min())
        if z0 < -0.001:
            z_shift = z0
            verts = verts.copy()
            verts[:, 2] -= z_shift
            mesh.vertices = o3d.utility.Vector3dVector(verts)

    reference_for_qa = reference_xyz
    if z_shift != 0.0:
        reference_for_qa = reference_xyz.copy()
        reference_for_qa[:, 2] -= z_shift

    qa = validate_mesh(mesh, qa_cfg, outer)
    coverage_stats = validate_point_coverage(
        mesh,
        reference_for_qa,
        qa_cfg,
    )
    if not coverage_stats.passed:
        qa = MeshQAResult(False, qa.messages + coverage_stats.messages)
    else:
        qa = MeshQAResult(qa.passed, qa.messages + coverage_stats.messages)

    if not qa.passed:
        raise RuntimeError("Surface mesh QA failed:\n  " + "\n  ".join(qa.messages))

    visual = _decimate(mesh, int(mesh_cfg.get("visual_triangles", 80000)))

    mesh_dir = out / "meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)
    visual_path = mesh_dir / "container_visual.stl"
    o3d.io.write_triangle_mesh(str(visual_path), visual)

    debug_path = None
    if mesh_cfg.get("write_debug_ply", True) and debug_clusters:
        debug_path = mesh_dir / "surface_debug.ply"
        debug_mesh = _merge_meshes(_colored_cluster_meshes(debug_clusters, mesh_cfg))
        if len(debug_mesh.triangles):
            o3d.io.write_triangle_mesh(str(debug_path), debug_mesh)

    return SurfaceMeshResult(
        visual_path=visual_path,
        debug_path=debug_path,
        visual_triangles=len(visual.triangles),
        qa=qa,
        coverage={
            "ratio": coverage_stats.coverage_ratio,
            "median_m": coverage_stats.median_distance_m,
        },
    )
