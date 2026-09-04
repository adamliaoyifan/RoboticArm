"""Light preprocess: up-axis alignment, horizontal ground, leg columns, opening yaw."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import open3d as o3d

from cargo_io import numpy_to_pcd, save_ply
from cargo_plane_segment import (
    segment_body_legs_planes,
    segment_body_legs_z_band,
    write_leg_obb_debug,
)


@dataclass
class PreprocessResult:
    object_xyz: np.ndarray
    ground_xyz: np.ndarray
    body_xyz: np.ndarray
    legs_xyz: np.ndarray
    object_rgb: np.ndarray | None
    transform: np.ndarray
    stats: dict[str, Any]
    body_bottom_z: float


def _mask_optional(arr: np.ndarray | None, mask: np.ndarray) -> np.ndarray | None:
    return None if arr is None else arr[mask]


def light_outliers(xyz: np.ndarray, rgb: np.ndarray | None, cfg: dict) -> tuple[np.ndarray, np.ndarray | None]:
    pcd = numpy_to_pcd(xyz, rgb)
    sor = cfg.get("sor", {})
    _, ind = pcd.remove_statistical_outlier(
        nb_neighbors=int(sor.get("nb_neighbors", 20)),
        std_ratio=float(sor.get("std_ratio", 3.0)),
    )
    mask = np.zeros(xyz.shape[0], dtype=bool)
    mask[np.asarray(ind, dtype=np.int64)] = True
    xyz, rgb = xyz[mask], _mask_optional(rgb, mask)

    db = cfg.get("dbscan", {})
    if db.get("enabled", True) and xyz.shape[0] > 100:
        labels = np.array(
            numpy_to_pcd(xyz).cluster_dbscan(
                eps=float(db.get("eps", 0.12)),
                min_points=int(db.get("min_points", 20)),
                print_progress=False,
            )
        )
        if labels.max() >= 0:
            keep_labels = []
            for lab in range(labels.max() + 1):
                count = int((labels == lab).sum())
                keep_labels.append((count, lab))
            keep_labels.sort(reverse=True)
            n_keep = int(db.get("keep_largest_components", 2))
            allowed = {lab for _, lab in keep_labels[:n_keep]}
            mask = np.isin(labels, list(allowed))
            xyz, rgb = xyz[mask], _mask_optional(rgb, mask)
    return xyz, rgb


def _normalize(v: np.ndarray) -> np.ndarray:
    v = np.asarray(v, dtype=np.float64).reshape(3)
    n = np.linalg.norm(v)
    return v / (n + 1e-12)


def detect_up_axis(xyz: np.ndarray, cfg: dict) -> tuple[np.ndarray, dict[str, Any]]:
    mode = cfg.get("up_axis", "auto")
    meta: dict[str, Any] = {"mode": mode}

    if mode == "z" or mode == "scan_z":
        up = np.array([0.0, 0.0, 1.0])
        meta["height_span"] = float(np.percentile(xyz[:, 2], 99) - np.percentile(xyz[:, 2], 1))
        return up, meta
    if mode == "y":
        up = np.array([0.0, 1.0, 0.0])
        meta["height_span"] = float(np.percentile(xyz[:, 1], 99) - np.percentile(xyz[:, 1], 1))
        return up, meta
    if isinstance(mode, (list, tuple)) and len(mode) == 3:
        up = _normalize(mode)
        h = xyz @ up
        meta["height_span"] = float(np.percentile(h, 99) - np.percentile(h, 1))
        return up, meta

    min_span = float(cfg.get("min_height_span", 1.0))
    best_up, best_score, best_span = np.array([0.0, 0.0, 1.0]), -1.0, 0.0
    for i in range(3):
        for sign in (1.0, -1.0):
            up = sign * np.eye(3)[i]
            h = xyz @ up
            span = float(np.percentile(h, 99) - np.percentile(h, 1))
            if span < min_span:
                continue
            floor_band = h < np.percentile(h, 5) + 0.05
            density = float(floor_band.mean())
            score = span * density
            if score > best_score:
                best_score, best_up, best_span = score, up, span

    if abs(best_up[2]) >= 0.9 and best_up[2] < 0:
        best_up = -best_up
    elif abs(best_up[1]) >= 0.9 and best_up[1] < 0:
        best_up = -best_up
    elif abs(best_up[0]) >= 0.9 and best_up[0] < 0:
        best_up = -best_up

    meta["detected_up"] = best_up.tolist()
    meta["height_span"] = best_span
    meta["auto_score"] = best_score
    return best_up, meta


def _rotation_to_z(normal: np.ndarray) -> np.ndarray:
    normal = _normalize(normal)
    z = np.array([0.0, 0.0, 1.0])
    if np.linalg.norm(np.cross(normal, z)) < 1e-8:
        if normal[2] < 0:
            return np.diag([1.0, 1.0, -1.0])
        return np.eye(3)
    axis = np.cross(normal, z)
    axis /= np.linalg.norm(axis)
    angle = np.arccos(np.clip(normal @ z, -1.0, 1.0))
    c, s = np.cos(angle), np.sin(angle)
    x, y, zc = axis
    return np.array(
        [
            [c + x * x * (1 - c), x * y * (1 - c) - zc * s, x * zc * (1 - c) + y * s],
            [y * x * (1 - c) + zc * s, c + y * y * (1 - c), y * zc * (1 - c) - x * s],
            [zc * x * (1 - c) - y * s, zc * y * (1 - c) + x * s, c + zc * zc * (1 - c)],
        ]
    )


def _make_transform(rot: np.ndarray, origin: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = rot
    T[:3, 3] = -rot @ origin
    return T


def _apply_transform(xyz: np.ndarray, T: np.ndarray) -> np.ndarray:
    if xyz.shape[0] == 0:
        return xyz
    hom = np.hstack((xyz, np.ones((xyz.shape[0], 1))))
    return hom @ T.T[:, :3]


def align_gravity_to_z(
    xyz: np.ndarray,
    rgb: np.ndarray | None,
    up: np.ndarray,
    align_cfg: dict,
) -> tuple[np.ndarray, np.ndarray | None, np.ndarray]:
    method = align_cfg.get("method", "floor_plane")
    if method == "scan_z" and np.allclose(up, [0, 0, 1], atol=0.01):
        rot = np.eye(3)
    else:
        rot = _rotation_to_z(up)

    aligned = xyz @ rot.T
    if float(np.percentile(aligned[:, 2], 50)) < float(np.percentile(aligned[:, 2], 5)):
        flip = np.diag([1.0, 1.0, -1.0])
        rot = flip @ rot
        aligned = xyz @ rot.T
    z0 = float(np.percentile(aligned[:, 2], 1))
    origin = np.array([aligned[:, 0].mean(), aligned[:, 1].mean(), z0])
    T = _make_transform(rot, origin)
    return _apply_transform(xyz, T), rgb, T


def detect_floor_and_deck_z(z: np.ndarray, cfg: dict) -> dict[str, Any]:
    """Find true floor (lowest Z peak) and container deck (next major peak above gap)."""
    z = np.asarray(z, dtype=np.float64)
    n = z.shape[0]
    z_min = float(z.min())
    z_max = float(z.max())

    bin_w = float(cfg.get("bin_width", 0.02))
    floor_max_th = float(cfg.get("floor_max_thickness", 0.05))
    floor_clearance = float(cfg.get("floor_clearance", 0.03))
    min_gap = float(cfg.get("min_gap_to_deck", 0.12))
    drop_ratio = float(cfg.get("floor_peak_drop_ratio", 0.15))
    deck_ratio = float(cfg.get("deck_peak_min_ratio", 0.25))
    min_leg_h = float(cfg.get("min_leg_height", 0.08))
    min_count = int(cfg.get("floor_peak_min_count", max(500, int(0.002 * n))))

    z_hi = min(z_min + 0.85 * (z_max - z_min), z_max)
    edges = np.arange(z_min, z_hi + bin_w, bin_w)
    hist, edges = np.histogram(z, bins=edges)
    centers = 0.5 * (edges[:-1] + edges[1:])
    h_max = int(hist.max()) if hist.size else 0

    floor_top_z = z_min + floor_max_th
    method = "fallback_cap"

    if hist.size > 0:
        peak0_idx = 0
        for i in range(len(hist)):
            if hist[i] >= min_count:
                peak0_idx = i
                break
        peak0 = int(hist[peak0_idx])
        top_idx = peak0_idx
        low_run = 0
        for j in range(peak0_idx + 1, len(hist)):
            if hist[j] >= peak0 * drop_ratio:
                top_idx = j
                low_run = 0
            else:
                low_run += 1
                if low_run >= 3:
                    break
        floor_top_z = float(edges[top_idx + 1])
        floor_top_z = min(floor_top_z, z_min + floor_max_th)
        method = "lowest_peak"

    p2 = float(np.percentile(z, 2))
    p5 = float(np.percentile(z, 5))
    if p5 - p2 > min_gap:
        floor_top_z = min(floor_top_z, p2 + floor_clearance)
        method = "gap_percentile"

    deck_bottom_z = float(z_min + 0.12 * (z_max - z_min))
    search_start = floor_top_z + min_leg_h
    if hist.size and h_max > 0:
        for i in range(len(hist)):
            if centers[i] < search_start:
                continue
            if hist[i] >= hist.max() * deck_ratio:
                if i == 0 or hist[i] >= hist[i - 1]:
                    if i == len(hist) - 1 or hist[i] >= hist[i + 1]:
                        deck_bottom_z = float(centers[i])
                        break
        else:
            above = centers[centers >= search_start]
            if above.size:
                sub = hist[centers >= search_start]
                deck_bottom_z = float(above[int(np.argmax(sub))])

    gap_m = float(deck_bottom_z - floor_top_z)
    return {
        "z_min": z_min,
        "z_max": z_max,
        "floor_top_z": float(floor_top_z),
        "deck_bottom_z": float(deck_bottom_z),
        "gap_m": gap_m,
        "method": method,
        "p2": p2,
        "p5": p5,
    }


def anchor_z_to_floor(
    xyz: np.ndarray,
    rgb: np.ndarray | None,
    floor_top_z: float,
    T_prev: np.ndarray,
) -> tuple[np.ndarray, np.ndarray | None, np.ndarray, float]:
    """Shift Z so floor_top sits at z=0."""
    shift = np.eye(4)
    shift[2, 3] = -float(floor_top_z)
    T_new = shift @ T_prev
    xyz_out = _apply_transform(xyz, shift)
    return xyz_out, rgb, T_new, float(floor_top_z)


def split_ground(
    xyz: np.ndarray,
    rgb: np.ndarray | None,
    cfg: dict,
    layers: dict[str, Any],
) -> tuple[np.ndarray, np.ndarray, np.ndarray | None, np.ndarray | None, dict[str, Any]]:
    """Ground = lowest Z layer only (not container deck)."""
    mode = cfg.get("mode", "z_min_gap")
    floor_top_z = float(layers["floor_top_z"])
    meta = {
        "ground_method": mode,
        "floor_top_z": floor_top_z,
        "deck_bottom_z": layers.get("deck_bottom_z"),
    }

    if mode == "z_band":
        clearance = float(cfg.get("ground_clearance", 0.03))
        ground_mask = xyz[:, 2] < float(np.percentile(xyz[:, 2], 5)) + clearance
        meta["ground_method"] = "z_band_legacy"
    else:
        ground_mask = xyz[:, 2] <= floor_top_z
        if cfg.get("refine_plane", False):
            dist = float(cfg.get("distance_threshold", 0.03))
            idx = np.where(ground_mask)[0]
            if idx.shape[0] >= 50:
                pcd = numpy_to_pcd(xyz[idx])
                plane, inliers = pcd.segment_plane(
                    distance_threshold=dist, ransac_n=3, num_iterations=1000
                )
                normal = _normalize(np.array(plane[:3], dtype=np.float64))
                if normal[2] < 0:
                    normal = -normal
                if abs(normal[2]) > 0.9:
                    signed = xyz @ normal + float(plane[3])
                    refined = (np.abs(signed) < dist) & (xyz[:, 2] <= floor_top_z)
                    if refined.sum() > 0.3 * ground_mask.sum():
                        ground_mask = refined
                        meta["ground_method"] = "z_min_gap_plane_refine"

    meta["ground_points"] = int(ground_mask.sum())
    obj_mask = ~ground_mask
    return (
        xyz[obj_mask],
        xyz[ground_mask],
        _mask_optional(rgb, obj_mask),
        _mask_optional(rgb, ground_mask),
        meta,
    )


def split_body_legs(
    xyz: np.ndarray,
    rgb: np.ndarray | None,
    seg_cfg: dict,
    layers: dict[str, Any],
) -> tuple[np.ndarray, np.ndarray, np.ndarray | None, np.ndarray | None, float, dict[str, Any]]:
    """Split object cloud into body (incl. deck) and vertical leg columns."""
    if seg_cfg.get("body_bottom_z") is not None:
        layers = {**layers, "deck_bottom_z": float(seg_cfg["body_bottom_z"])}
    method = seg_cfg.get("method", "plane_and_vertical_obb")
    if method == "z_band_only":
        return segment_body_legs_z_band(xyz, rgb, seg_cfg, layers)
    return segment_body_legs_planes(xyz, rgb, seg_cfg, layers)


def align_opening_yaw(xyz: np.ndarray, cfg: dict) -> tuple[np.ndarray, np.ndarray]:
    yaw = _best_yaw_for_opening(xyz, cfg.get("opening_side", "positive_y"))
    c, s = np.cos(yaw), np.sin(yaw)
    rot_yaw = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    origin = np.array([xyz[:, 0].mean(), xyz[:, 1].mean(), float(np.percentile(xyz[:, 2], 1))])
    T = _make_transform(rot_yaw, origin)
    return _apply_transform(xyz, T), T


def _best_yaw_for_opening(xyz: np.ndarray, side: str) -> float:
    del side
    best_yaw, best_score = 0.0, -np.inf
    for k in range(4):
        yaw = k * np.pi / 2
        c, s = np.cos(yaw), np.sin(yaw)
        rot = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        pts = xyz @ rot.T
        y_max = pts[:, 1].max()
        band = pts[:, 1] > y_max - 0.1
        if band.sum() < 30:
            continue
        facade = pts[band]
        grid, _, _ = np.histogram2d(facade[:, 0], facade[:, 2], bins=(16, 16))
        occ = (grid > 0).mean()
        score = facade.shape[0] * (1.0 - occ)
        if score > best_score:
            best_score, best_yaw = score, yaw
    return best_yaw


def preprocess(
    xyz: np.ndarray,
    rgb: np.ndarray | None,
    cfg: dict,
    output_dir: Any | None = None,
) -> PreprocessResult:
    n0 = xyz.shape[0]
    xyz, rgb = light_outliers(xyz, rgb, cfg)

    up, up_meta = detect_up_axis(xyz, cfg)
    xyz_g, rgb_g, T_grav = align_gravity_to_z(xyz, rgb, up, cfg.get("alignment", {}))

    ground_cfg = cfg.get("ground", {})
    layers = detect_floor_and_deck_z(xyz_g[:, 2], ground_cfg)

    z_shift = 0.0
    if ground_cfg.get("anchor_floor_to_z0", True):
        xyz_g, rgb_g, T_grav, z_shift = anchor_z_to_floor(xyz_g, rgb_g, layers["floor_top_z"], T_grav)
        layers["floor_top_z"] = 0.0
        layers["deck_bottom_z"] = float(layers["deck_bottom_z"] - z_shift)
        layers["z_shift_applied"] = z_shift

    if output_dir is not None:
        out = Path(output_dir)
        save_ply(out / "aligned_pre_split.ply", xyz_g, rgb_g)
        with (out / "up_axis.json").open("w", encoding="utf-8") as fh:
            json.dump(up_meta, fh, indent=2)
        layers["ground_points_est"] = int((xyz_g[:, 2] <= layers["floor_top_z"]).sum())
        layers["leg_points_est"] = int(
            ((xyz_g[:, 2] > layers["floor_top_z"]) & (xyz_g[:, 2] <= layers["deck_bottom_z"] + 0.05)).sum()
        )
        layers["body_points_est"] = int((xyz_g[:, 2] > layers["deck_bottom_z"] + 0.05).sum())
        with (out / "z_layers.json").open("w", encoding="utf-8") as fh:
            json.dump(layers, fh, indent=2)

    obj, ground, rgb_obj, rgb_ground, ground_meta = split_ground(
        xyz_g, rgb_g, ground_cfg, layers
    )
    seg_cfg = cfg.get("segmentation", {})
    body, legs, rgb_body, rgb_legs, body_bottom, seg_meta = split_body_legs(
        obj, rgb_obj, seg_cfg, layers
    )

    obj_al, T_yaw = align_opening_yaw(obj, cfg.get("alignment", {}))
    T_total = T_yaw @ T_grav

    body_al = _apply_transform(body, T_yaw)
    legs_al = _apply_transform(legs, T_yaw)
    ground_al = _apply_transform(ground, T_yaw)

    if output_dir is not None:
        out = Path(output_dir)
        save_ply(out / "object_points.ply", obj_al)
        save_ply(out / "ground_points.ply", ground_al, rgb_ground)
        save_ply(out / "body_points.ply", body_al, rgb_body)
        save_ply(out / "leg_points.ply", legs_al, rgb_legs)
        planes_out = {
            "horizontal": seg_meta.get("horizontal_planes", []),
            "vertical": seg_meta.get("vertical_planes", []),
            "deck_plane_inliers": seg_meta.get("deck_plane_inliers"),
        }
        with (out / "planes.json").open("w", encoding="utf-8") as fh:
            json.dump(planes_out, fh, indent=2)
        leg_fits = seg_meta.get("leg_fits", [])
        if leg_fits:
            write_leg_obb_debug(out / "leg_obb_debug.ply", leg_fits)
        layers["segmentation"] = {
            "method": seg_meta.get("method"),
            "body_points": int(body_al.shape[0]),
            "leg_points": int(legs_al.shape[0]),
            "leg_fits": leg_fits,
        }
        with (out / "z_layers.json").open("w", encoding="utf-8") as fh:
            json.dump(layers, fh, indent=2)

    stats = {
        "input_points": n0,
        "object_points": int(obj_al.shape[0]),
        "body_points": int(body_al.shape[0]),
        "leg_points": int(legs_al.shape[0]),
        "ground_points": int(ground_al.shape[0]),
        "body_bottom_z": body_bottom,
        "up_axis": up_meta,
        "z_layers": layers,
        "ground_split": ground_meta,
        "segmentation": seg_meta,
        "leg_fits": seg_meta.get("leg_fits", []),
    }

    return PreprocessResult(
        object_xyz=obj_al,
        ground_xyz=ground_al,
        body_xyz=body_al,
        legs_xyz=legs_al,
        object_rgb=rgb_obj,
        transform=T_total,
        body_bottom_z=body_bottom,
        stats=stats,
    )
