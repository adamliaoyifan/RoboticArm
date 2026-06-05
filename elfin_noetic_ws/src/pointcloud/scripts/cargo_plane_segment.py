"""Plane-based body/leg segmentation and vertical leg OBB fitting."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
import open3d as o3d

from cargo_io import numpy_to_pcd


def _mask_optional(arr: np.ndarray | None, mask: np.ndarray) -> np.ndarray | None:
    return None if arr is None else arr[mask]


def _normalize(v: np.ndarray) -> np.ndarray:
    v = np.asarray(v, dtype=np.float64).reshape(3)
    return v / (np.linalg.norm(v) + 1e-12)


def _signed_distance(xyz: np.ndarray, plane: np.ndarray) -> np.ndarray:
    n = _normalize(plane[:3])
    return xyz @ n + float(plane[3])


def _fit_planes_iterative(
    xyz: np.ndarray,
    cfg: dict,
    horizontal_only: bool = False,
    vertical_only: bool = False,
) -> list[dict[str, Any]]:
    dist = float(cfg.get("plane_distance", 0.03))
    max_planes = int(cfg.get("max_planes", 10))
    max_tilt_deg = float(cfg.get("max_tilt_deg", 15.0))
    min_hz = float(np.cos(np.radians(max_tilt_deg)))
    min_vz = float(np.sin(np.radians(30.0)))

    work = numpy_to_pcd(xyz.copy())
    planes: list[dict[str, Any]] = []
    for _ in range(max_planes):
        if len(work.points) < 200:
            break
        model, inliers = work.segment_plane(
            distance_threshold=dist, ransac_n=3, num_iterations=2500
        )
        if len(inliers) < 50:
            break
        normal = _normalize(np.array(model[:3], dtype=np.float64))
        if normal[2] < 0:
            normal = -normal
        is_horizontal = abs(normal[2]) >= min_hz
        is_vertical = abs(normal[2]) <= min_vz
        if horizontal_only and not is_horizontal:
            work = work.select_by_index(inliers, invert=True)
            continue
        if vertical_only and not is_vertical:
            work = work.select_by_index(inliers, invert=True)
            continue
        inlier_xyz = np.asarray(work.points)[inliers]
        planes.append(
            {
                "normal": normal.tolist(),
                "d": float(model[3]),
                "model": [float(normal[0]), float(normal[1]), float(normal[2]), float(model[3])],
                "count": int(len(inliers)),
                "z_center": float(inlier_xyz[:, 2].mean()) if inlier_xyz.size else 0.0,
                "horizontal": bool(is_horizontal),
                "vertical": bool(is_vertical),
            }
        )
        work = work.select_by_index(inliers, invert=True)
    return planes


def _is_flat_cluster(pts: np.ndarray, cfg: dict) -> bool:
    mn, mx = pts.min(axis=0), pts.max(axis=0)
    z_span = float(mx[2] - mn[2])
    xy_extent = float(max(mx[0] - mn[0], mx[1] - mn[1]))
    return z_span < float(cfg.get("deck_flat_max_span", 0.08)) and xy_extent > float(
        cfg.get("deck_flat_min_xy", 0.25)
    )


def _cluster_vertical_legs(
    leg_candidates: np.ndarray,
    seg_cfg: dict,
) -> tuple[np.ndarray, list[dict[str, Any]]]:
    obb_cfg = {**seg_cfg.get("leg_obb", {}), **seg_cfg}
    eps = float(seg_cfg.get("leg_dbscan_eps", 0.08))
    min_pts = int(seg_cfg.get("leg_dbscan_min_points", 12))
    min_aspect = float(obb_cfg.get("min_aspect_ratio", 3.0))
    min_height = float(obb_cfg.get("min_height", 0.08))

    n = leg_candidates.shape[0]
    leg_mask = np.zeros(n, dtype=bool)
    leg_fits: list[dict[str, Any]] = []

    if n < min_pts:
        return leg_mask, leg_fits

    labels = np.array(
        o3d.geometry.PointCloud(o3d.utility.Vector3dVector(leg_candidates)).cluster_dbscan(
            eps=eps, min_points=min_pts, print_progress=False
        )
    )
    if labels.max() < 0:
        return leg_mask, leg_fits

    for lab in range(labels.max() + 1):
        local = labels == lab
        if local.sum() < min_pts:
            continue
        pts = leg_candidates[local]
        if _is_flat_cluster(pts, obb_cfg):
            continue
        mn, mx = pts.min(axis=0), pts.max(axis=0)
        z_span = float(mx[2] - mn[2])
        xy_extent = float(max(mx[0] - mn[0], mx[1] - mn[1]))
        if z_span < min_height:
            continue
        if z_span / (xy_extent + 1e-6) < min_aspect:
            continue
        pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts))
        obb = pcd.get_oriented_bounding_box()
        extents = np.sort(np.asarray(obb.extent))
        if float(extents[0]) > float(obb_cfg.get("max_cross_section", 0.25)):
            continue
        axis = _normalize(np.asarray(obb.R)[:, 2])
        max_tilt = float(obb_cfg.get("max_tilt_deg", 20.0))
        if abs(axis[2]) < float(np.cos(np.radians(max_tilt))):
            continue
        leg_mask[local] = True
        leg_fits.append(
            {
                "center": (0.5 * (mn + mx)).tolist(),
                "size": (mx - mn).tolist(),
                "height": z_span,
                "z_min": float(mn[2]),
                "z_max": float(mx[2]),
                "point_count": int(local.sum()),
            }
        )

    return leg_mask, leg_fits


def segment_body_legs_planes(
    xyz: np.ndarray,
    rgb: np.ndarray | None,
    seg_cfg: dict,
    layers: dict[str, Any],
) -> tuple[np.ndarray, np.ndarray, np.ndarray | None, np.ndarray | None, float, dict[str, Any]]:
    floor_top = float(layers["floor_top_z"])
    deck_bottom = float(seg_cfg.get("body_bottom_z") or layers["deck_bottom_z"])
    deck_band = float(seg_cfg.get("deck_exclude_band", 0.04))
    deck_z_lo = deck_bottom - deck_band
    wall_th = float(seg_cfg.get("wall_thickness", 0.05))
    plane_dist = float(seg_cfg.get("plane_distance", 0.03))

    meta: dict[str, Any] = {"method": "plane_and_vertical_obb", "deck_bottom_z": deck_bottom}

    h_planes = _fit_planes_iterative(xyz, seg_cfg, horizontal_only=True)
    meta["horizontal_planes"] = h_planes

    deck_inlier_mask = np.zeros(xyz.shape[0], dtype=bool)
    deck_plane = None
    best_count = 0
    for pl in h_planes:
        if not pl["horizontal"]:
            continue
        if abs(pl["z_center"] - deck_bottom) > 0.06:
            continue
        if pl["count"] > best_count:
            best_count = pl["count"]
            deck_plane = pl

    if deck_plane is not None:
        model = np.array(deck_plane["model"], dtype=np.float64)
        deck_inlier_mask = np.abs(_signed_distance(xyz, model)) < plane_dist * 1.5
        meta["deck_plane_inliers"] = int(deck_inlier_mask.sum())

    deck_z_mask = xyz[:, 2] >= deck_z_lo

    v_planes = _fit_planes_iterative(xyz, seg_cfg, vertical_only=True)
    meta["vertical_planes"] = v_planes
    wall_mask = np.zeros(xyz.shape[0], dtype=bool)
    for pl in v_planes:
        if not pl.get("vertical"):
            continue
        model = np.array(pl["model"], dtype=np.float64)
        wall_mask |= np.abs(_signed_distance(xyz, model)) < wall_th

    leg_z_hi = deck_bottom - deck_band
    leg_candidate_mask = (
        (xyz[:, 2] > floor_top) & (xyz[:, 2] < leg_z_hi) & ~deck_inlier_mask
    )
    leg_candidates = xyz[leg_candidate_mask]
    leg_local_mask, leg_fits = _cluster_vertical_legs(leg_candidates, seg_cfg)

    leg_mask = np.zeros(xyz.shape[0], dtype=bool)
    cand_idx = np.where(leg_candidate_mask)[0]
    leg_mask[cand_idx[leg_local_mask]] = True

    body_mask = (deck_inlier_mask | deck_z_mask | wall_mask) & ~leg_mask
    body_mask |= (xyz[:, 2] >= deck_z_lo) & ~leg_mask

    meta["leg_fits"] = leg_fits
    meta["leg_points"] = int(leg_mask.sum())
    meta["body_points"] = int(body_mask.sum())

    return (
        xyz[body_mask],
        xyz[leg_mask],
        _mask_optional(rgb, body_mask),
        _mask_optional(rgb, leg_mask),
        deck_bottom,
        meta,
    )


def segment_body_legs_z_band(
    xyz: np.ndarray,
    rgb: np.ndarray | None,
    seg_cfg: dict,
    layers: dict[str, Any],
) -> tuple[np.ndarray, np.ndarray, np.ndarray | None, np.ndarray | None, float, dict[str, Any]]:
    floor_top = float(layers["floor_top_z"])
    deck_bottom = float(seg_cfg.get("body_bottom_z") or layers["deck_bottom_z"])
    deck_band = float(seg_cfg.get("deck_exclude_band", 0.04))
    leg_z_hi = deck_bottom - deck_band

    candidate_mask = (xyz[:, 2] > floor_top) & (xyz[:, 2] < leg_z_hi)
    leg_candidates = xyz[candidate_mask]
    leg_local, leg_fits = _cluster_vertical_legs(leg_candidates, seg_cfg)

    leg_mask = np.zeros(xyz.shape[0], dtype=bool)
    cand_idx = np.where(candidate_mask)[0]
    leg_mask[cand_idx[leg_local]] = True

    body_mask = xyz[:, 2] >= (deck_bottom - deck_band)
    body_mask &= ~leg_mask

    meta = {
        "method": "z_band_only",
        "leg_fits": leg_fits,
        "deck_bottom_z": deck_bottom,
        "leg_points": int(leg_mask.sum()),
        "body_points": int(body_mask.sum()),
    }
    return (
        xyz[body_mask],
        xyz[leg_mask],
        _mask_optional(rgb, body_mask),
        _mask_optional(rgb, leg_mask),
        deck_bottom,
        meta,
    )


def write_leg_obb_debug(path: Path, leg_fits: list[dict[str, Any]]) -> None:
    pts = []
    for leg in leg_fits:
        c = np.asarray(leg["center"], dtype=np.float64)
        s = np.asarray(leg["size"], dtype=np.float64) * 0.5
        for dx in (-1, 1):
            for dy in (-1, 1):
                for dz in (-1, 1):
                    pts.append(c + np.array([dx, dy, dz]) * s)
    if not pts:
        return
    o3d.io.write_point_cloud(
        str(path), o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.asarray(pts)))
    )
