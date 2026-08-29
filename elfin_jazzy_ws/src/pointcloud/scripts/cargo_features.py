"""Legacy: planes/lines/corners. Superseded by cargo_parametric_fit.py (not used by measure_cargo_from_las.py)."""

from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Any

import numpy as np
import yaml
from scipy.spatial import ConvexHull
from cargo_io import save_ply


@dataclass
class FeatureResult:
    body_xyz: np.ndarray
    legs_xyz: np.ndarray
    transform: np.ndarray
    features: dict[str, Any]
    container_yaml: dict[str, Any]
    report_lines: list[str]


def _rotation_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = axis / (np.linalg.norm(axis) + 1e-12)
    x, y, z = axis
    c, s = np.cos(angle), np.sin(angle)
    return np.array(
        [
            [c + x * x * (1 - c), x * y * (1 - c) - z * s, x * z * (1 - c) + y * s],
            [y * x * (1 - c) + z * s, c + y * y * (1 - c), y * z * (1 - c) - x * s],
            [z * x * (1 - c) - y * s, z * y * (1 - c) + x * s, c + z * z * (1 - c)],
        ]
    )


def _normal_to_z_rotation(normal: np.ndarray) -> np.ndarray:
    normal = normal / (np.linalg.norm(normal) + 1e-12)
    z_axis = np.array([0.0, 0.0, 1.0])
    if np.linalg.norm(np.cross(normal, z_axis)) < 1e-6:
        return np.eye(3)
    v = np.cross(normal, z_axis)
    angle = np.arccos(np.clip(normal @ z_axis, -1.0, 1.0))
    return _rotation_matrix(v, angle)


def fit_ground_plane_rotation(xyz: np.ndarray, dist: float = 0.03) -> np.ndarray | None:
    import open3d as o3d

    if xyz.shape[0] < 200:
        return None
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))
    model, inliers = pcd.segment_plane(
        distance_threshold=dist, ransac_n=3, num_iterations=3000
    )
    if len(inliers) < max(200, xyz.shape[0] // 20):
        return None
    normal = np.array(model[:3], dtype=np.float64)
    if normal[2] < 0:
        normal = -normal
    if abs(normal[2]) < 0.7:
        return None
    return _normal_to_z_rotation(normal)


def pca_max_extent_vertical(xyz: np.ndarray) -> np.ndarray:
    """Tallest box axis (max variance) as vertical — not argmin."""
    centered = xyz - xyz.mean(axis=0)
    cov = np.cov(centered.T)
    _, evecs = np.linalg.eigh(cov)
    up = evecs[:, -1]
    if up[2] < 0:
        up = -up
    return _normal_to_z_rotation(up)


def align_vertical(xyz: np.ndarray, cfg: dict[str, Any]) -> tuple[np.ndarray, str]:
    method = cfg.get("method", "floor_plane")
    dist = float(cfg.get("plane_distance", 0.03))
    world_z = np.array([0.0, 0.0, 1.0])

    if method == "scan_z":
        return np.eye(3), "scan_z"

    if method in ("floor_plane", "auto"):
        rot = fit_ground_plane_rotation(xyz, dist)
        if rot is not None:
            return rot, "floor_plane"

    if method in ("pca_max_extent", "auto", "floor_plane"):
        return pca_max_extent_vertical(xyz), "pca_max_extent"

    return np.eye(3), "identity"


def align_opening_plus_y(xyz: np.ndarray) -> np.ndarray:
    best_yaw = 0.0
    best_score = -np.inf
    for k in range(4):
        yaw = k * np.pi / 2
        c, s = np.cos(yaw), np.sin(yaw)
        rot = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        pts = xyz @ rot.T
        y_max = pts[:, 1].max()
        band = pts[:, 1] > y_max - 0.08
        if band.sum() < 20:
            continue
        facade = pts[band]
        xz = facade[:, [0, 2]]
        span_x = xz[:, 0].max() - xz[:, 0].min()
        span_z = xz[:, 1].max() - xz[:, 1].min()
        grid, _, _ = np.histogram2d(
            facade[:, 0],
            facade[:, 2],
            bins=(20, 20),
        )
        occupancy = (grid > 0).mean()
        score = span_x * span_z * (1.0 - occupancy)
        if score > best_score:
            best_score = score
            best_yaw = yaw
    c, s = np.cos(best_yaw), np.sin(best_yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _origin_bottom_center(
    aligned: np.ndarray,
    legs_xyz: np.ndarray,
    preprocess_stats: dict[str, Any] | None,
) -> np.ndarray:
    mn, mx = aligned.min(axis=0), aligned.max(axis=0)
    xy = np.array([(mn[0] + mx[0]) / 2, (mn[1] + mx[1]) / 2])
    z0 = float(mn[2])
    if legs_xyz.shape[0] >= 10:
        z0 = float(np.percentile(legs_xyz[:, 2], 2))
    elif preprocess_stats and preprocess_stats.get("z_floor_deck") is not None:
        z_band = float(preprocess_stats["z_floor_deck"]) * 0.05 + 0.05
        near = aligned[aligned[:, 2] < float(preprocess_stats["z_floor_deck"]) + z_band]
        if near.shape[0] > 20:
            z0 = float(np.percentile(near[:, 2], 5))
    return np.array([xy[0], xy[1], z0])


def build_transform(
    xyz: np.ndarray,
    align_cfg: dict[str, Any],
    legs_xyz: np.ndarray | None = None,
    preprocess_stats: dict[str, Any] | None = None,
) -> tuple[np.ndarray, np.ndarray, str, np.ndarray]:
    combined = xyz if legs_xyz is None or legs_xyz.shape[0] == 0 else np.vstack((xyz, legs_xyz))
    rot_v, align_method = align_vertical(combined, align_cfg)
    aligned = combined @ rot_v.T
    legs_aligned = (
        legs_xyz @ rot_v.T if legs_xyz is not None and legs_xyz.shape[0] else legs_xyz
    )
    rot_y = align_opening_plus_y(aligned)
    aligned = aligned @ rot_y.T
    if legs_aligned is not None and legs_aligned.shape[0]:
        legs_aligned = legs_aligned @ rot_y.T
    rot = rot_y @ rot_v

    body_aligned = xyz @ rot.T
    origin = _origin_bottom_center(
        np.vstack((body_aligned, legs_aligned))
        if legs_aligned is not None and legs_aligned.shape[0]
        else body_aligned,
        legs_aligned if legs_aligned is not None else np.empty((0, 3)),
        preprocess_stats,
    )
    T = np.eye(4)
    T[:3, :3] = rot
    T[:3, 3] = -rot @ origin
    shifted = body_aligned - origin
    return T, shifted, align_method, origin


def apply_transform(xyz: np.ndarray, T: np.ndarray) -> np.ndarray:
    hom = np.hstack((xyz, np.ones((xyz.shape[0], 1))))
    return (hom @ T.T)[:, :3]


def percentile_bounds(xyz: np.ndarray, lo: float, hi: float) -> tuple[np.ndarray, np.ndarray]:
    mn = np.percentile(xyz, lo, axis=0)
    mx = np.percentile(xyz, hi, axis=0)
    return mn, mx


def fit_planes(xyz: np.ndarray, cfg: dict[str, Any]) -> list[dict[str, Any]]:
    import open3d as o3d

    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))
    planes = []
    work = pcd
    dist = float(cfg.get("plane_distance", 0.03))
    max_planes = int(cfg.get("max_planes", 8))
    for _ in range(max_planes):
        if len(work.points) < 200:
            break
        model, inliers = work.segment_plane(
            distance_threshold=dist, ransac_n=3, num_iterations=2000
        )
        if len(inliers) < 200:
            break
        normal = np.array(model[:3], dtype=np.float64)
        normal /= np.linalg.norm(normal) + 1e-12
        d = float(model[3])
        name = "horizontal"
        if abs(normal[2]) < 0.2:
            name = "vertical"
        planes.append(
            {
                "normal": normal.tolist(),
                "d": d,
                "name": name,
                "point_count": len(inliers),
            }
        )
        work = work.select_by_index(inliers, invert=True)
    return planes


def detect_opening(xyz: np.ndarray, mn: np.ndarray, mx: np.ndarray) -> dict[str, Any]:
    y_face = mx[1] - 0.04
    facade = xyz[xyz[:, 1] >= y_face - 0.06]
    if facade.shape[0] < 50:
        w = mx[0] - mn[0]
        h = mx[2] - mn[2]
        return {
            "width": float(w * 0.75),
            "height": float(h * 0.9),
            "side": "positive_y",
            "corners": [],
        }
    xs = facade[:, 0]
    zs = facade[:, 2]
    x_bins = np.linspace(mn[0], mx[0], 30)
    z_bins = np.linspace(mn[2], mx[2], 30)
    grid, _, _ = np.histogram2d(xs, zs, bins=[x_bins, z_bins])
    occupied = grid > 0
    best = None
    best_area = 0
    for ix in range(occupied.shape[0]):
        for iz in range(occupied.shape[1]):
            if occupied[ix, iz]:
                continue
            for iw in range(1, occupied.shape[0] - ix):
                if occupied[ix + iw - 1, iz]:
                    break
                for ih in range(1, occupied.shape[1] - iz):
                    hole = occupied[ix : ix + iw, iz : iz + ih]
                    if hole.any():
                        break
                    area = iw * ih
                    if area > best_area:
                        best_area = area
                        best = (ix, iz, iw, ih)
    if best is None:
        w = (mx[0] - mn[0]) * 0.75
        h = (mx[2] - mn[2]) * 0.9
        x0 = mn[0] + 0.15 * (mx[0] - mn[0])
        z0 = mn[2] + 0.05 * (mx[2] - mn[2])
    else:
        ix, iz, iw, ih = best
        x0 = x_bins[ix]
        z0 = z_bins[iz]
        w = x_bins[min(ix + iw, len(x_bins) - 1)] - x0
        h = z_bins[min(iz + ih, len(z_bins) - 1)] - z0
    y = mx[1]
    corners = [
        [x0, y, z0],
        [x0 + w, y, z0],
        [x0 + w, y, z0 + h],
        [x0, y, z0 + h],
    ]
    return {
        "width": float(w),
        "height": float(h),
        "side": "positive_y",
        "corners": corners,
    }


def slice_corners_2d(xyz: np.ndarray, z: float, band: float = 0.06) -> list[list[float]]:
    layer = xyz[np.abs(xyz[:, 2] - z) < band]
    if layer.shape[0] < 30:
        return []
    xy = layer[:, :2]
    try:
        hull = ConvexHull(xy)
        poly = xy[hull.vertices]
    except Exception:
        return []
    corners = []
    n = len(poly)
    for i in range(n):
        p0 = poly[i]
        p1 = poly[(i + 1) % n]
        edge = p1 - p0
        if np.linalg.norm(edge) < 0.15:
            continue
        corners.append([float(p0[0]), float(p0[1]), float(z)])
    return corners


def fit_slice_edges(xyz: np.ndarray, heights: list[float]) -> tuple[list[dict], list[dict]]:
    all_corners: list[dict] = []
    edges: list[dict] = []
    for zi, z in enumerate(heights):
        layer = xyz[np.abs(xyz[:, 2] - z) < 0.06]
        if layer.shape[0] < 40:
            continue
        xy = layer[:, :2]
        try:
            hull = ConvexHull(xy)
            poly = xy[hull.vertices]
        except Exception:
            continue
        n = len(poly)
        for i in range(n):
            p0 = poly[i]
            p1 = poly[(i + 1) % n]
            if np.linalg.norm(p1 - p0) < 0.2:
                continue
            cid = f"z{z:.2f}_c{i}"
            all_corners.append({"id": cid, "xyz": [float(p0[0]), float(p0[1]), float(z)]})
            edges.append(
                {
                    "id": f"z{z:.2f}_e{i}",
                    "p0": [float(p0[0]), float(p0[1]), float(z)],
                    "p1": [float(p1[0]), float(p1[1]), float(z)],
                }
            )
    return all_corners, edges


def build_container_yaml(
    mn: np.ndarray,
    mx: np.ndarray,
    opening: dict[str, Any],
    legs_meta: list[dict],
    wall_t: float,
) -> dict[str, Any]:
    outer = {
        "length": float(mx[0] - mn[0]),
        "width": float(mx[1] - mn[1]),
        "height": float(mx[2] - mn[2]),
    }
    inner = {
        "length": float(max(outer["length"] - 2 * wall_t, 0.5)),
        "width": float(max(outer["width"] - 2 * wall_t, 0.5)),
        "height": float(max(outer["height"] - wall_t, 0.5)),
    }
    ox = (mn[0] + mx[0]) / 2
    oz = (mn[2] + mx[2]) / 2
    if opening.get("corners"):
        oc = np.array(opening["corners"])
        ox = float(oc[:, 0].mean())
        oz = float(oc[:, 2].mean())
    return {
        "outer": outer,
        "inner": inner,
        "opening": {
            "side": opening.get("side", "positive_y"),
            "width": opening.get("width", outer["width"] * 0.75),
            "height": opening.get("height", outer["height"] * 0.9),
            "frame": {"xyz": [ox, float(mx[1] - 0.02), oz], "rpy": [0.0, 0.0, 0.0]},
        },
        "legs": legs_meta,
        "meta": {"name": "measured_from_pointcloud", "version": "1.0"},
    }


def write_report(
    container: dict[str, Any],
    cfg: dict[str, Any],
    stats: dict[str, Any],
) -> list[str]:
    ref = np.asarray(cfg.get("reference_outer", [2.4, 2.0, 2.2]))
    outer = container["outer"]
    meas = np.array([outer["length"], outer["width"], outer["height"]])
    rel_err = np.abs(meas - ref) / ref
    lines = [
        "Cargo measurement report",
        "========================",
        f"Input points: {stats.get('input_points', 'n/a')}",
        f"Body points after preprocess: {stats.get('body_points', 'n/a')}",
        f"Leg clusters: {len(container.get('legs', []))}",
        "",
        "Outer (m):",
        f"  length (X): {outer['length']:.4f}",
        f"  width  (Y): {outer['width']:.4f}",
        f"  height (Z): {outer['height']:.4f}",
        "",
        "Opening:",
        f"  width:  {container['opening']['width']:.4f}",
        f"  height: {container['opening']['height']:.4f}",
        "",
        f"Reference placeholder: {ref.tolist()}",
    ]
    warn_th = float(cfg.get("warn_relative_error", 0.1))
    for i, label in enumerate(["length", "width", "height"]):
        if rel_err[i] > warn_th:
            lines.append(f"WARN: {label} differs from reference by {100*rel_err[i]:.1f}%")
    return lines


def validate_aligned_span(
    body: np.ndarray,
    min_height_span: float,
) -> tuple[bool, str]:
    if body.shape[0] < 100:
        return False, "too few body points after alignment"
    span = body.max(axis=0) - body.min(axis=0)
    if float(span[2]) < min_height_span:
        return (
            False,
            f"aligned height span {span[2]:.3f} m < min_height_span {min_height_span} m "
            "(likely wrong vertical alignment — check aligned_debug.ply)",
        )
    return True, ""


def extract_features(
    body_xyz: np.ndarray,
    legs_xyz: np.ndarray,
    legs_meta: list[dict],
    preprocess_stats: dict[str, Any],
    cfg: dict[str, Any],
    output_dir: Any,
    align_cfg: dict[str, Any] | None = None,
) -> FeatureResult:
    from pathlib import Path

    out = Path(output_dir)
    acfg = align_cfg or {}
    min_h = float(acfg.get("min_height_span", cfg.get("min_height_span", 1.0)))
    T, body, align_method, origin = build_transform(
        body_xyz, acfg, legs_xyz, preprocess_stats
    )
    legs = apply_transform(legs_xyz, T) if legs_xyz.shape[0] else legs_xyz

    ok, err = validate_aligned_span(body, min_h)
    if not ok:
        save_ply(out / "aligned_debug.ply", body)
        raise RuntimeError(err)

    lo = float(cfg.get("percentile_low", 1.0))
    hi = float(cfg.get("percentile_high", 99.0))
    mn, mx = percentile_bounds(body, lo, hi)
    wall_t = float(cfg.get("wall_thickness", 0.05))

    planes = fit_planes(body, cfg)
    opening = detect_opening(body, mn, mx)
    heights = list(cfg.get("slice_heights", [0.05, 0.5, 1.0, 1.5, 2.0]))
    if mx[2] - mn[2] > 0.2:
        heights.append(float(mx[2] - 0.05))
    corners, edges = fit_slice_edges(body, heights)

    if legs.shape[0]:
        floor_deck_z_aligned = float(np.percentile(legs[:, 2], 90))
    else:
        floor_deck_z_aligned = float(np.percentile(body[:, 2], 8))

    container = build_container_yaml(mn, mx, opening, legs_meta, wall_t)
    features = {
        "transform_scan_to_container": T.tolist(),
        "alignment_method": align_method,
        "origin_rule": "floor_deck_bottom_center",
        "origin_xyz": origin.tolist(),
        "mesh_origin_offset": [0.0, 0.0, 0.0],
        "floor_deck_z": floor_deck_z_aligned,
        "floor_deck_z_scan": preprocess_stats.get("z_floor_deck"),
        "planes": planes,
        "corners_3d": corners,
        "edges": edges,
        "opening": opening,
        "legs": legs_meta,
        "bounds": {"min": mn.tolist(), "max": mx.tolist()},
        "aligned_span": (mx - mn).tolist(),
    }
    report = write_report(container, cfg, preprocess_stats)
    report.append(f"Alignment method: {align_method}")
    report.append(f"Aligned span (m): {(mx - mn).tolist()}")

    save_ply(out / "aligned_debug.ply", np.vstack((body, legs)) if legs.shape[0] else body)
    save_ply(out / "cleaned.ply", np.vstack((body, legs)) if legs.shape[0] else body)
    if corners:
        corner_xyz = np.array([c["xyz"] for c in corners])
        save_ply(out / "corners.ply", corner_xyz)

    return FeatureResult(
        body_xyz=body,
        legs_xyz=legs,
        transform=T,
        features=features,
        container_yaml=container,
        report_lines=report,
    )


def write_outputs(result: FeatureResult, output_dir: Any) -> None:
    from pathlib import Path

    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    with (out / "features.json").open("w", encoding="utf-8") as fh:
        json.dump(result.features, fh, indent=2)
    with (out / "measured_container.yaml").open("w", encoding="utf-8") as fh:
        yaml.safe_dump(result.container_yaml, fh, sort_keys=False, default_flow_style=False)
    with (out / "report.txt").open("w", encoding="utf-8") as fh:
        fh.write("\n".join(result.report_lines) + "\n")
