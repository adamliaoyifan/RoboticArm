#!/usr/bin/env python3
"""Extract Cargo container points from a room-scale LAS point cloud.

Pipeline (mirrors CloudCompare workflow in README.md):
  load/inspect -> oriented box crop -> RANSAC plane strip -> DBSCAN cluster pick -> SOR -> export
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import numpy as np
import open3d as o3d
import yaml

try:
    import laspy
except ImportError as exc:
    raise SystemExit(
        "laspy is required. Install: pip install -r requirements.txt"
    ) from exc


def load_config(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def load_las(path: Path) -> tuple[np.ndarray, np.ndarray | None, np.ndarray | None]:
    las = laspy.read(str(path))
    xyz = np.vstack((las.x, las.y, las.z)).T.astype(np.float64)
    rgb = None
    intensity = None
    if hasattr(las, "red") and hasattr(las, "green") and hasattr(las, "blue"):
        rgb = np.vstack(
            (las.red, las.green, las.blue)
        ).T.astype(np.float64)
        if rgb.max() > 1.0:
            rgb = rgb / 65535.0
    if hasattr(las, "intensity"):
        intensity = np.asarray(las.intensity, dtype=np.float64)
    return xyz, rgb, intensity


def inspect_cloud(
    xyz: np.ndarray, rgb: np.ndarray | None, intensity: np.ndarray | None
) -> dict[str, Any]:
    mins = xyz.min(axis=0)
    maxs = xyz.max(axis=0)
    span = maxs - mins
    center = (mins + maxs) / 2.0
    info: dict[str, Any] = {
        "point_count": int(xyz.shape[0]),
        "bbox_min": mins.tolist(),
        "bbox_max": maxs.tolist(),
        "bbox_span": span.tolist(),
        "bbox_center": center.tolist(),
    }
    if rgb is not None:
        info["has_rgb"] = True
    if intensity is not None:
        info["intensity_min"] = float(intensity.min())
        info["intensity_max"] = float(intensity.max())
        info["intensity_mean"] = float(intensity.mean())
    return info


def subsample_spatial(
    xyz: np.ndarray,
    rgb: np.ndarray | None,
    intensity: np.ndarray | None,
    spacing: float,
) -> tuple[np.ndarray, np.ndarray | None, np.ndarray | None]:
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))
    if rgb is not None:
        pcd.colors = o3d.utility.Vector3dVector(rgb)
    down = pcd.voxel_down_sample(spacing)
    xyz_out = np.asarray(down.points)
    rgb_out = np.asarray(down.colors) if down.has_colors() else None
    if intensity is not None and xyz_out.shape[0] < xyz.shape[0]:
        tree = o3d.geometry.KDTreeFlann(pcd)
        idx = []
        for pt in xyz_out:
            _, i, _ = tree.search_knn_vector_3d(pt, 1)
            idx.append(i[0])
        intensity = intensity[np.asarray(idx, dtype=np.int64)]
    return xyz_out, rgb_out, intensity


def rpy_to_rotation(rpy: list[float]) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def crop_oriented_box(
    xyz: np.ndarray,
    rgb: np.ndarray | None,
    intensity: np.ndarray | None,
    center: np.ndarray,
    rpy: list[float],
    length: float,
    width: float,
    height: float,
    margin: float,
) -> tuple[np.ndarray, np.ndarray | None, np.ndarray | None]:
    rot = rpy_to_rotation(rpy)
    local = (xyz - center) @ rot
    half = np.array(
        [length / 2 + margin, width / 2 + margin, height / 2 + margin]
    )
    mask = np.all(np.abs(local) <= half, axis=1)
    return xyz[mask], _mask_optional(rgb, mask), _mask_optional(intensity, mask)


def _mask_optional(arr: np.ndarray | None, mask: np.ndarray) -> np.ndarray | None:
    return None if arr is None else arr[mask]


def strip_room_planes(
    xyz: np.ndarray,
    rgb: np.ndarray | None,
    intensity: np.ndarray | None,
    cfg: dict[str, Any],
) -> tuple[np.ndarray, np.ndarray | None, np.ndarray | None]:
    """Remove floor/ceiling only — keep vertical faces (Cargo walls)."""
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz.copy()))
    if rgb is not None:
        pcd.colors = o3d.utility.Vector3dVector(rgb.copy())

    dist_th = float(cfg["distance_threshold"])
    min_pts = int(cfg["min_plane_points"])
    max_planes = int(cfg["max_planes"])
    floor_z = float(cfg["floor_normal_z_min"])
    edge_band = float(cfg.get("edge_band", 0.12))

    z_min, z_max = float(xyz[:, 2].min()), float(xyz[:, 2].max())
    keep = np.ones(xyz.shape[0], dtype=bool)
    work = pcd
    for _ in range(max_planes):
        if len(work.points) < min_pts * 2:
            break
        model, inliers = work.segment_plane(
            distance_threshold=dist_th,
            ransac_n=3,
            num_iterations=2000,
        )
        if len(inliers) < min_pts:
            break
        normal = np.array(model[:3], dtype=np.float64)
        normal /= np.linalg.norm(normal) + 1e-12
        nz = abs(normal[2])
        if nz < floor_z:
            break
        active_idx = np.where(keep)[0]
        inlier_idx = active_idx[np.asarray(inliers, dtype=np.int64)]
        inlier_z = xyz[inlier_idx, 2]
        near_floor = np.mean(inlier_z) < z_min + edge_band
        near_ceil = np.mean(inlier_z) > z_max - edge_band
        if near_floor or near_ceil:
            keep[inlier_idx] = False
        work = work.select_by_index(inliers, invert=True)

    return xyz[keep], _mask_optional(rgb, keep), _mask_optional(intensity, keep)


def pick_cargo_cluster(
    xyz: np.ndarray,
    rgb: np.ndarray | None,
    intensity: np.ndarray | None,
    cfg: dict[str, Any],
) -> tuple[np.ndarray, np.ndarray | None, np.ndarray | None, int]:
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))
    labels = np.array(
        pcd.cluster_dbscan(
            eps=float(cfg["eps"]),
            min_points=int(cfg["min_points"]),
            print_progress=False,
        )
    )
    if labels.size == 0 or labels.max() < 0:
        return xyz, rgb, intensity, -1

    target = np.sort(np.asarray(cfg["target_dims"], dtype=np.float64))[::-1]
    best_label = -1
    best_score = -np.inf
    for label in range(labels.max() + 1):
        mask = labels == label
        if mask.sum() < cfg["min_points"]:
            continue
        pts = xyz[mask]
        span = np.sort((pts.max(axis=0) - pts.min(axis=0)))[::-1]
        size_err = np.linalg.norm(span - target)
        score = mask.sum() - 500.0 * size_err
        if score > best_score:
            best_score = score
            best_label = label

    if best_label < 0:
        return xyz, rgb, intensity, -1

    mask = labels == best_label
    return (
        xyz[mask],
        _mask_optional(rgb, mask),
        _mask_optional(intensity, mask),
        best_label,
    )


def sor_filter(
    xyz: np.ndarray,
    rgb: np.ndarray | None,
    intensity: np.ndarray | None,
    cfg: dict[str, Any],
) -> tuple[np.ndarray, np.ndarray | None, np.ndarray | None]:
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))
    if rgb is not None:
        pcd.colors = o3d.utility.Vector3dVector(rgb)
    filtered, ind = pcd.remove_statistical_outlier(
        nb_neighbors=int(cfg["nb_neighbors"]),
        std_ratio=float(cfg["std_ratio"]),
    )
    mask = np.zeros(xyz.shape[0], dtype=bool)
    mask[np.asarray(ind, dtype=np.int64)] = True
    return xyz[mask], _mask_optional(rgb, mask), _mask_optional(intensity, mask)


def save_ply(
    path: Path,
    xyz: np.ndarray,
    rgb: np.ndarray | None,
) -> None:
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))
    if rgb is not None:
        pcd.colors = o3d.utility.Vector3dVector(np.clip(rgb, 0.0, 1.0))
    o3d.io.write_point_cloud(str(path), pcd)


def save_las(
    path: Path,
    xyz: np.ndarray,
    rgb: np.ndarray | None,
    intensity: np.ndarray | None,
    template: Path | None,
) -> None:
    las = laspy.create(point_format=3, file_version="1.4")
    las.x = xyz[:, 0]
    las.y = xyz[:, 1]
    las.z = xyz[:, 2]
    if rgb is not None:
        scale = 65535 if rgb.max() <= 1.0 else 1.0
        las.red = (rgb[:, 0] * scale).astype(np.uint16)
        las.green = (rgb[:, 1] * scale).astype(np.uint16)
        las.blue = (rgb[:, 2] * scale).astype(np.uint16)
    if intensity is not None:
        las.intensity = intensity.astype(np.uint16)
    las.write(str(path))


def run_pipeline(cfg: dict[str, Any], inspect_only: bool) -> int:
    input_path = Path(cfg["input_las"]).expanduser()
    if not input_path.exists():
        print(f"ERROR: input LAS not found: {input_path}", file=sys.stderr)
        return 1

    xyz, rgb, intensity = load_las(input_path)
    info = inspect_cloud(xyz, rgb, intensity)
    print(json.dumps({"stage": "load_inspect", **info}, indent=2))

    if inspect_only:
        print(
            "\nSuggested crop center (bbox center):",
            info["bbox_center"],
            "\nEdit config cargo.center / cargo.rpy then re-run without --inspect-only.",
        )
        return 0

    spacing = cfg.get("subsample_spacing")
    if spacing:
        xyz, rgb, intensity = subsample_spatial(xyz, rgb, intensity, float(spacing))
        print(f"Subsampled to {xyz.shape[0]} points (spacing={spacing} m)")

    cargo = cfg["cargo"]
    center = np.asarray(cargo["center"], dtype=np.float64)
    xyz, rgb, intensity = crop_oriented_box(
        xyz,
        rgb,
        intensity,
        center,
        cargo["rpy"],
        float(cargo["length"]),
        float(cargo["width"]),
        float(cargo["height"]),
        float(cargo["margin"]),
    )
    print(f"After oriented crop: {xyz.shape[0]} points")
    if xyz.shape[0] == 0:
        print("ERROR: crop removed all points; adjust cargo.center/rpy", file=sys.stderr)
        return 1

    xyz, rgb, intensity = strip_room_planes(xyz, rgb, intensity, cfg["planes"])
    print(f"After plane strip: {xyz.shape[0]} points")

    xyz, rgb, intensity, label = pick_cargo_cluster(xyz, rgb, intensity, cfg["cluster"])
    print(f"After cluster pick (label={label}): {xyz.shape[0]} points")

    xyz, rgb, intensity = sor_filter(xyz, rgb, intensity, cfg["sor"])
    print(f"After SOR: {xyz.shape[0]} points")

    out_dir = Path(cfg.get("output_dir", "./output"))
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = input_path.stem + "_cargo"
    ply_path = out_dir / f"{stem}.ply"
    las_path = out_dir / f"{stem}.las"
    save_ply(ply_path, xyz, rgb)
    save_las(las_path, xyz, rgb, intensity, template=input_path)
    print(f"Exported:\n  {ply_path}\n  {las_path}")

    final = inspect_cloud(xyz, rgb, intensity)
    print(json.dumps({"stage": "export", **final}, indent=2))
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "-c",
        "--config",
        type=Path,
        default=Path(__file__).resolve().parent.parent / "config" / "extract_cargo.yaml",
        help="YAML config path",
    )
    parser.add_argument(
        "--inspect-only",
        action="store_true",
        help="Only load LAS and print bbox stats (step 0)",
    )
    args = parser.parse_args()
    if not args.config.exists():
        example = args.config.parent / "extract_cargo.yaml.example"
        print(
            f"Config not found: {args.config}\n"
            f"Copy example and edit:\n  cp {example} {args.config}",
            file=sys.stderr,
        )
        return 1
    cfg = load_config(args.config)
    return run_pipeline(cfg, args.inspect_only)


if __name__ == "__main__":
    raise SystemExit(main())
