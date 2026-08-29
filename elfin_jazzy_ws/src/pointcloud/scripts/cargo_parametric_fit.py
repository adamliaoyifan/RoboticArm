"""Fit parametric hexagonal body, opening, and legs from segmented point cloud."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np
from scipy.spatial import ConvexHull


@dataclass
class ParametricModel:
    model: dict[str, Any]
    report_lines: list[str]


def _fit_hexagon_xz(body: np.ndarray, cfg: dict) -> list[list[float]]:
    xz = body[:, [0, 2]]
    if xz.shape[0] < 50:
        mn, mx = body.min(axis=0), body.max(axis=0)
        return [
            [mn[0], mx[2]],
            [mx[0], mx[2]],
            [mx[0], mn[2] + 0.1 * (mx[2] - mn[2])],
            [0.5 * (mn[0] + mx[0]), mn[2]],
            [mn[0], mn[2] + 0.15 * (mx[2] - mn[2])],
            [mn[0], mn[2] + 0.75 * (mx[2] - mn[2])],
        ]

    hull = ConvexHull(xz)
    pts = xz[hull.vertices]
    center = pts.mean(axis=0)
    angles = np.arctan2(pts[:, 1] - center[1], pts[:, 0] - center[0])
    order = np.argsort(angles)
    pts = pts[order]

    n_bins = int(cfg.get("hex_vertices", 6))
    bins = np.linspace(-np.pi, np.pi, n_bins + 1)
    verts = []
    for i in range(n_bins):
        mask = (angles[order] >= bins[i]) & (angles[order] < bins[i + 1])
        if mask.any():
            verts.append(pts[order][mask].mean(axis=0))
        else:
            verts.append(pts[i % len(pts)])

    poly = np.array(verts)
    mn, mx = poly.min(axis=0), poly.max(axis=0)
    chamfer = float(cfg.get("chamfer_x_ratio", 0.08))
    z_span = mx[1] - mn[1]
    x_span = mx[0] - mn[0]
    refined = [
        [mn[0], mx[1]],
        [mx[0], mx[1]],
        [mx[0], mn[1] + 0.12 * z_span],
        [mx[0] - 0.35 * x_span, mn[1]],
        [mn[0] + chamfer * x_span, mn[1] + 0.18 * z_span],
        [mn[0], mn[1] + 0.72 * z_span],
    ]
    return [[float(x), float(z)] for x, z in refined]


def _detect_opening(body: np.ndarray, cfg: dict) -> dict[str, Any]:
    manual = cfg.get("manual")
    if manual:
        return {**manual, "enabled": True, "inferred": False}

    y_face = body[:, 1].max() - 0.05
    facade = body[body[:, 1] >= y_face - 0.08]
    if facade.shape[0] < 80:
        mn, mx = body.min(axis=0), body.max(axis=0)
        w = 0.55 * (mx[0] - mn[0])
        h = 0.75 * (mx[2] - mn[2])
        return {
            "enabled": True,
            "plane": "positive_y",
            "x_min": float(mn[0] + 0.2 * (mx[0] - mn[0])),
            "x_max": float(mn[0] + 0.2 * (mx[0] - mn[0]) + w),
            "z_min": float(mn[2] + 0.1 * (mx[2] - mn[2])),
            "z_max": float(mn[2] + 0.1 * (mx[2] - mn[2]) + h),
            "inferred": True,
        }

    xs, zs = facade[:, 0], facade[:, 2]
    x_bins = np.linspace(xs.min(), xs.max(), 24)
    z_bins = np.linspace(zs.min(), zs.max(), 24)
    grid, _, _ = np.histogram2d(xs, zs, bins=[x_bins, z_bins])
    occupied = grid > 0
    best, best_area = None, 0
    for ix in range(occupied.shape[0] - 2):
        for iz in range(occupied.shape[1] - 2):
            for iw in range(2, min(12, occupied.shape[0] - ix)):
                for ih in range(2, min(12, occupied.shape[1] - iz)):
                    hole = occupied[ix : ix + iw, iz : iz + ih]
                    if hole.any():
                        continue
                    area = iw * ih
                    if area > best_area:
                        best_area = area
                        best = (ix, iz, iw, ih)
    if best is None:
        mn, mx = body.min(axis=0), body.max(axis=0)
        return {
            "enabled": True,
            "plane": "positive_y",
            "x_min": float(0.25 * mn[0] + 0.75 * mx[0]),
            "x_max": float(0.75 * mn[0] + 0.25 * mx[0]),
            "z_min": float(mn[2] + 0.15 * (mx[2] - mn[2])),
            "z_max": float(mn[2] + 0.85 * (mx[2] - mn[2])),
            "inferred": True,
        }
    ix, iz, iw, ih = best
    return {
        "enabled": True,
        "plane": "positive_y",
        "x_min": float(x_bins[ix]),
        "x_max": float(x_bins[min(ix + iw, len(x_bins) - 1)]),
        "z_min": float(z_bins[iz]),
        "z_max": float(z_bins[min(iz + ih, len(z_bins) - 1)]),
        "inferred": False,
    }


def _cluster_legs_from_points(legs: np.ndarray, leg_cfg: dict) -> list[dict[str, Any]]:
    if legs.shape[0] < 20:
        return []
    import open3d as o3d

    labels = np.array(
        o3d.geometry.PointCloud(o3d.utility.Vector3dVector(legs)).cluster_dbscan(
            eps=float(leg_cfg.get("dbscan_eps", 0.08)),
            min_points=int(leg_cfg.get("dbscan_min_points", 12)),
            print_progress=False,
        )
    )
    detected: list[dict[str, Any]] = []
    if labels.max() < 0:
        return detected
    min_pts = int(leg_cfg.get("dbscan_min_points", 12))
    for lab in range(labels.max() + 1):
        idx = labels == lab
        if idx.sum() < min_pts:
            continue
        pts = legs[idx]
        mn, mx = pts.min(axis=0), pts.max(axis=0)
        detected.append(
            {
                "center": (0.5 * (mn + mx)).tolist(),
                "size": (mx - mn).tolist(),
                "height": float(mx[2] - mn[2]),
                "point_count": int(idx.sum()),
            }
        )
    return detected


def _median_same_side(
    name: str,
    detected: list[dict[str, Any]],
    body_center_x: float,
) -> float:
    if not detected:
        return 0.12
    is_left = "left" in name
    same_side = [
        d["height"]
        for d in detected
        if (d["center"][0] < body_center_x) == is_left
    ]
    if same_side:
        return float(np.median(same_side))
    return float(np.median([d["height"] for d in detected]))


def _fit_legs(
    legs: np.ndarray,
    body: np.ndarray,
    cfg: dict,
    preprocess_leg_fits: list[dict[str, Any]] | None = None,
) -> list[dict[str, Any]]:
    leg_cfg = cfg.get("legs", {})
    if preprocess_leg_fits:
        detected = [
            {
                "center": f["center"],
                "size": f["size"],
                "height": float(f["height"]),
                "point_count": int(f.get("point_count", 0)),
            }
            for f in preprocess_leg_fits
        ]
    else:
        detected = _cluster_legs_from_points(legs, leg_cfg)

    mn, mx = body.min(axis=0), body.max(axis=0)
    y_min, y_max = float(mn[1]), float(mx[1])
    x_min, x_max = float(mn[0]), float(mx[0])
    margin_x = 0.12 * (x_max - x_min)
    margin_y = 0.12 * (y_max - y_min)
    corners = {
        "front_left": [x_min + margin_x, y_min + margin_y],
        "front_right": [x_max - margin_x, y_min + margin_y],
        "back_left": [x_min + margin_x, y_max - margin_y],
        "back_right": [x_max - margin_x, y_max - margin_y],
    }

    out: list[dict[str, Any]] = []
    used = set()
    for name, xy in corners.items():
        best, best_d = None, np.inf
        for i, leg in enumerate(detected):
            if i in used:
                continue
            c = leg["center"]
            d = (c[0] - xy[0]) ** 2 + (c[1] - xy[1]) ** 2
            if d < best_d:
                best_d, best = d, i
        if best is not None and best_d < (0.35 * (x_max - x_min)) ** 2:
            leg = detected[best]
            used.add(best)
            out.append(
                {
                    "id": name,
                    "center": leg["center"],
                    "size": leg["size"],
                    "height": leg["height"],
                    "inferred": False,
                }
            )
        else:
            sizes = [np.array(d["size"]) for d in detected] if detected else []
            heights = [d["height"] for d in detected] if detected else []
            med_size = np.median(sizes, axis=0) if sizes else np.array([0.12, 0.12, 0.12])
            med_h = _median_same_side(name, detected, 0.5 * (x_min + x_max))
            out.append(
                {
                    "id": name,
                    "center": [xy[0], xy[1], med_h / 2],
                    "size": med_size.tolist(),
                    "height": med_h,
                    "inferred": True,
                }
            )
    return out


def fit_parametric_model(
    body_xyz: np.ndarray,
    legs_xyz: np.ndarray,
    transform: np.ndarray,
    stats: dict[str, Any],
    cfg: dict[str, Any],
) -> ParametricModel:
    body_cfg = cfg.get("body", {})
    mn, mx = body_xyz.min(axis=0), body_xyz.max(axis=0)
    depth_y = float(body_cfg.get("depth_y") or (mx[1] - mn[1]))
    depth_y = max(depth_y, 0.5)
    polygon_xz = _fit_hexagon_xz(body_xyz, body_cfg)
    opening = _detect_opening(body_xyz, cfg.get("opening", {}))
    leg_fits = stats.get("leg_fits") or stats.get("segmentation", {}).get("leg_fits")
    legs = _fit_legs(legs_xyz, body_xyz, cfg, preprocess_leg_fits=leg_fits)

    model = {
        "meta": {"pipeline": "parametric_cad_v2", "version": "2.0"},
        "transform_scan_to_container": transform.tolist(),
        "body": {
            "type": "extruded_polygon_xz",
            "depth_y": depth_y,
            "y_min": float(mn[1]),
            "polygon_xz": polygon_xz,
        },
        "opening": opening,
        "legs": legs,
        "outer": {
            "length": float(mx[0] - mn[0]),
            "width": float(mx[1] - mn[1]),
            "height": float(mx[2] - mn[2]),
        },
        "preprocess_stats": stats,
    }

    lines = [
        "Parametric cargo model",
        "====================",
        f"Body points: {stats.get('body_points', 'n/a')}",
        f"Leg points: {stats.get('leg_points', 'n/a')}",
        f"Outer LxWxH: {model['outer']['length']:.3f} x {model['outer']['width']:.3f} x {model['outer']['height']:.3f}",
        f"Body depth_y: {depth_y:.3f}",
        f"Hex vertices: {len(polygon_xz)}",
        f"Legs fitted: {len(legs)} (inferred: {sum(1 for l in legs if l.get('inferred'))})",
    ]
    if opening.get("enabled"):
        lines.append(
            f"Opening: x[{opening['x_min']:.2f},{opening['x_max']:.2f}] "
            f"z[{opening['z_min']:.2f},{opening['z_max']:.2f}]"
        )
    return ParametricModel(model=model, report_lines=lines)


def write_model_outputs(result: ParametricModel, output_dir: Any) -> None:
    import json
    from pathlib import Path

    import yaml

    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    with (out / "parametric_model.json").open("w", encoding="utf-8") as fh:
        json.dump(result.model, fh, indent=2)
    container_yaml = {
        "outer": result.model["outer"],
        "opening": {
            "side": result.model["opening"].get("plane", "positive_y"),
            "width": float(
                result.model["opening"].get("x_max", 0) - result.model["opening"].get("x_min", 0)
            ),
            "height": float(
                result.model["opening"].get("z_max", 0) - result.model["opening"].get("z_min", 0)
            ),
            "frame": {"xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]},
        },
        "legs": result.model["legs"],
        "body": result.model["body"],
        "meta": result.model["meta"],
    }
    with (out / "measured_container.yaml").open("w", encoding="utf-8") as fh:
        yaml.safe_dump(container_yaml, fh, sort_keys=False)
    with (out / "report.txt").open("w", encoding="utf-8") as fh:
        fh.write("\n".join(result.report_lines) + "\n")
