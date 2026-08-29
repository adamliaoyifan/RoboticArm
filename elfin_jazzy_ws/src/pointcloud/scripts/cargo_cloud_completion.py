"""Geometric point-cloud completion: planar wall patches and leg column extension."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from cargo_io import load_point_cloud, save_ply
from cargo_mesh_opening import resolve_opening, y_front_from_body


@dataclass
class CompletionResult:
    body_path: Path
    leg_path: Path
    debug_path: Path | None
    body_original: int
    body_synthetic: int
    leg_original: int
    leg_synthetic: int
    messages: list[str] = field(default_factory=list)


def _unit(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    if n < 1e-12:
        raise ValueError("zero-length vector")
    return v / n


def _orthonormal_basis(
    plane_normal: np.ndarray,
    u_axis: np.ndarray | None,
    v_axis: np.ndarray | None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    n = _unit(np.asarray(plane_normal, dtype=np.float64))
    if u_axis is not None:
        u = np.asarray(u_axis, dtype=np.float64)
        u = u - n * np.dot(u, n)
        u = _unit(u)
    else:
        ref = np.array([0.0, 0.0, 1.0]) if abs(n[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
        u = _unit(np.cross(n, ref))
    if v_axis is not None:
        v = np.asarray(v_axis, dtype=np.float64)
        v = v - n * np.dot(v, n) - u * np.dot(v, u)
        v = _unit(v)
    else:
        v = _unit(np.cross(n, u))
    return u, v, n


def _axis_normal(axis: str, sign: int = 1) -> np.ndarray:
    axes = {
        "x": np.array([1.0, 0.0, 0.0]),
        "y": np.array([0.0, 1.0, 0.0]),
        "z": np.array([0.0, 0.0, 1.0]),
    }
    if axis not in axes:
        raise ValueError(f"unknown axis: {axis}")
    return axes[axis] * float(sign)


def _normal_from_target_face(face: str) -> np.ndarray:
    normals = {
        "positive_x": np.array([1.0, 0.0, 0.0]),
        "negative_x": np.array([-1.0, 0.0, 0.0]),
        "positive_y": np.array([0.0, 1.0, 0.0]),
        "negative_y": np.array([0.0, -1.0, 0.0]),
        "positive_z": np.array([0.0, 0.0, 1.0]),
        "negative_z": np.array([0.0, 0.0, -1.0]),
    }
    if face not in normals:
        raise ValueError(f"unknown target_face: {face}")
    return normals[face]


def _default_axes_for_normal(n: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    idx = int(np.argmax(np.abs(n)))
    if idx == 0:
        return np.array([0.0, 1.0, 0.0]), np.array([0.0, 0.0, 1.0])
    if idx == 1:
        return np.array([1.0, 0.0, 0.0]), np.array([0.0, 0.0, 1.0])
    return np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0])


def _normal_from_patch_cfg(patch: dict[str, Any]) -> np.ndarray:
    if "target_face" in patch:
        return _normal_from_target_face(str(patch["target_face"]))
    if "axis" in patch:
        return _axis_normal(str(patch["axis"]), int(patch.get("sign", 1)))
    return np.asarray(patch["plane_normal"], dtype=np.float64)


def _plane_from_patch_cfg(patch: dict[str, Any]) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    n = _normal_from_patch_cfg(patch)
    if "center_xyz" in patch:
        plane_point = np.asarray(patch["center_xyz"], dtype=np.float64)
    elif "plane_point" in patch:
        plane_point = np.asarray(patch["plane_point"], dtype=np.float64)
    else:
        plane_point = np.asarray([0.0, 0.0, 0.0], dtype=np.float64)
    u_default, v_default = _default_axes_for_normal(_unit(n))
    u, v, n = _orthonormal_basis(
        n,
        patch.get("u_axis", u_default),
        patch.get("v_axis", v_default),
    )
    return plane_point, u, v, n


def _opening_mask_for_points(
    pts: np.ndarray,
    opening: dict[str, Any] | None,
    parametric_model: dict[str, Any] | None,
    margin_y: float,
) -> np.ndarray:
    if opening is None or pts.size == 0:
        return np.zeros(pts.shape[0], dtype=bool)
    body = (parametric_model or {}).get("body", {})
    y_min = float(body.get("y_min", pts[:, 1].min()))
    depth_y = float(body.get("depth_y", pts[:, 1].max() - y_min))
    y_front = y_front_from_body(y_min, depth_y)
    x0, x1 = float(opening["x_min"]), float(opening["x_max"])
    z0, z1 = float(opening["z_min"]), float(opening["z_max"])
    return (
        (pts[:, 0] >= x0)
        & (pts[:, 0] <= x1)
        & (pts[:, 2] >= z0)
        & (pts[:, 2] <= z1)
        & (pts[:, 1] > y_front - margin_y)
    )


def _filter_opening_points(
    pts: np.ndarray,
    opening: dict[str, Any] | None,
    parametric_model: dict[str, Any] | None,
    margin_y: float,
) -> np.ndarray:
    return pts[~_opening_mask_for_points(pts, opening, parametric_model, margin_y)]


def _face_points_for_patch(
    body_xyz: np.ndarray,
    patch: dict[str, Any],
    n: np.ndarray,
) -> tuple[np.ndarray, float]:
    signed = body_xyz @ n
    face_value = float(np.quantile(signed, float(patch.get("face_quantile", 0.995))))
    band = float(patch.get("face_band", 0.08))
    face = body_xyz[np.abs(signed - face_value) <= band]
    if face.shape[0] < 100:
        face = body_xyz[signed >= face_value - band * 2.0]
    return face, face_value


def _detect_planar_hole_center(
    body_xyz: np.ndarray,
    patch: dict[str, Any],
    opening: dict[str, Any] | None,
    parametric_model: dict[str, Any] | None,
    opening_margin_y: float,
) -> np.ndarray:
    n = _unit(_normal_from_patch_cfg(patch))
    u_default, v_default = _default_axes_for_normal(n)
    u, v, n = _orthonormal_basis(
        n,
        patch.get("u_axis", u_default),
        patch.get("v_axis", v_default),
    )
    face, face_value = _face_points_for_patch(body_xyz, patch, n)
    if face.shape[0] < 100:
        raise RuntimeError("auto_hole could not find enough points on target face")

    plane_point = n * face_value
    local = face - plane_point
    uv = np.column_stack([local @ u, local @ v])

    valid = ~_opening_mask_for_points(face, opening, parametric_model, opening_margin_y)
    if "detect_u_min" in patch:
        valid &= uv[:, 0] >= float(patch["detect_u_min"])
    if "detect_u_max" in patch:
        valid &= uv[:, 0] <= float(patch["detect_u_max"])
    if "detect_v_min" in patch:
        valid &= uv[:, 1] >= float(patch["detect_v_min"])
    if "detect_v_max" in patch:
        valid &= uv[:, 1] <= float(patch["detect_v_max"])
    uv = uv[valid]
    face = face[valid]
    if uv.shape[0] < 100:
        raise RuntimeError("auto_hole has too few non-opening face points")

    cell = float(patch.get("detect_grid_spacing", max(float(patch.get("grid_spacing", 0.015)) * 2.0, 0.03)))
    u_half = float(patch["u_half"])
    v_half = float(patch["v_half"])
    pad = float(patch.get("detect_margin", max(u_half, v_half)))
    u_min, v_min = uv.min(axis=0) - pad
    u_max, v_max = uv.max(axis=0) + pad
    u_bins = np.arange(u_min, u_max + cell, cell)
    v_bins = np.arange(v_min, v_max + cell, cell)
    hist, _, _ = np.histogram2d(uv[:, 0], uv[:, 1], bins=[u_bins, v_bins])
    occupied = hist >= int(patch.get("detect_min_cell_points", 1))

    win_u = max(1, int(np.ceil((2.0 * u_half) / cell)))
    win_v = max(1, int(np.ceil((2.0 * v_half) / cell)))
    ring = max(1, int(np.ceil(float(patch.get("detect_ring_m", 0.12)) / cell)))
    min_ring_density = float(patch.get("min_ring_density", 0.08))
    max_inside_density = float(patch.get("max_inside_density", 0.20))
    prefer_high_v = bool(patch.get("prefer_high_v", True))

    best_score = -np.inf
    best_center: tuple[float, float] | None = None
    nu, nv = occupied.shape
    for i in range(ring, max(ring, nu - win_u - ring)):
        for j in range(ring, max(ring, nv - win_v - ring)):
            inside = occupied[i : i + win_u, j : j + win_v]
            inside_density = float(inside.mean())
            if inside_density > max_inside_density:
                continue
            outer = occupied[i - ring : i + win_u + ring, j - ring : j + win_v + ring]
            ring_cells = outer.size - inside.size
            if ring_cells <= 0:
                continue
            ring_density = float((outer.sum() - inside.sum()) / ring_cells)
            if ring_density < min_ring_density:
                continue
            center_u = u_bins[i] + 0.5 * win_u * cell
            center_v = v_bins[j] + 0.5 * win_v * cell
            v_norm = (center_v - v_min) / max(v_max - v_min, 1e-6)
            score = ring_density - 2.0 * inside_density
            if prefer_high_v:
                score += 0.15 * v_norm
            if score > best_score:
                best_score = score
                best_center = (center_u, center_v)

    if best_center is None:
        raise RuntimeError("auto_hole did not find an empty patch with dense surroundings")
    return plane_point + best_center[0] * u + best_center[1] * v


def fill_planar_patch(
    patch: dict[str, Any],
    body_xyz: np.ndarray,
    opening: dict[str, Any] | None,
    parametric_model: dict[str, Any] | None,
    opening_margin_y: float,
) -> tuple[np.ndarray, str]:
    mode = str(patch.get("mode", "manual_center")).lower()
    if mode == "auto_hole":
        if patch.get("target_face") == "auto_vertical":
            last_error: Exception | None = None
            candidates = ["positive_y", "negative_y", "positive_x", "negative_x"]
            for face in candidates:
                trial = {**patch, "target_face": face}
                trial.pop("u_axis", None)
                trial.pop("v_axis", None)
                try:
                    center = _detect_planar_hole_center(
                        body_xyz, trial, opening, parametric_model, opening_margin_y
                    )
                    patch = trial
                    break
                except RuntimeError as exc:
                    last_error = exc
            else:
                raise RuntimeError(f"auto_hole failed on all vertical faces: {last_error}")
        else:
            center = _detect_planar_hole_center(
                body_xyz, patch, opening, parametric_model, opening_margin_y
            )
        plane_point, u, v, n = _plane_from_patch_cfg({**patch, "center_xyz": center})
        plane_point = center
        u_center = 0.0
        v_center = 0.0
        mode_label = "auto_hole"
    elif "center_xyz" in patch:
        plane_point, u, v, n = _plane_from_patch_cfg(patch)
        plane_point = np.asarray(patch["center_xyz"], dtype=np.float64)
        u_center = 0.0
        v_center = 0.0
        mode_label = "manual_center"
    else:
        plane_point, u, v, n = _plane_from_patch_cfg(patch)
        u_center = float(patch.get("u_center", 0.0))
        v_center = float(patch.get("v_center", 0.0))
        mode_label = "legacy_uv"
    u_half = float(patch["u_half"])
    v_half = float(patch["v_half"])
    spacing = float(patch.get("grid_spacing", 0.015))
    thickness = float(patch.get("thickness", 0.02))

    us = np.arange(u_center - u_half, u_center + u_half + spacing * 0.5, spacing)
    vs = np.arange(v_center - v_half, v_center + v_half + spacing * 0.5, spacing)
    uu, vv = np.meshgrid(us, vs, indexing="xy")
    base = plane_point + np.outer(uu.ravel(), u) + np.outer(vv.ravel(), v)

    layers = [0.0]
    if thickness > 0:
        layers = [-0.5 * thickness, 0.5 * thickness]
    pts = np.vstack([base + layer * n for layer in layers])
    return _filter_opening_points(pts, opening, parametric_model, opening_margin_y), mode_label


def _load_measured_legs(path: Path) -> dict[str, dict[str, Any]]:
    if not path.exists():
        return {}
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    return {leg["id"]: leg for leg in data.get("legs", [])}


def _leg_cluster_near(
    legs_xyz: np.ndarray,
    center_xy: np.ndarray,
    radius: float,
) -> np.ndarray:
    if legs_xyz.shape[0] == 0:
        return np.empty((0, 3))
    d = np.linalg.norm(legs_xyz[:, :2] - center_xy, axis=1)
    return legs_xyz[d < radius]


def _cross_section_from_cluster(
    cluster: np.ndarray,
    fallback: np.ndarray,
    center_xy: np.ndarray,
) -> tuple[np.ndarray, float, np.ndarray, np.ndarray]:
    if cluster.shape[0] < 10:
        base_center = np.array([center_xy[0], center_xy[1], 0.0])
        return fallback[:2], float(fallback[2]), np.array([0.0, 0.0, 1.0]), base_center
    mn, mx = cluster.min(axis=0), cluster.max(axis=0)
    size_xy = (mx[:2] - mn[:2]).clip(min=0.05)
    z0, z1 = float(mn[2]), float(mx[2])
    axis = np.array([0.0, 0.0, 1.0])
    base_center = np.array([0.5 * (mn[0] + mx[0]), 0.5 * (mn[1] + mx[1]), z0])
    return size_xy, z1 - z0, axis, base_center


def _sibling_leg_size(legs_meta: dict[str, dict[str, Any]], leg_id: str) -> np.ndarray:
    pairs = {
        "front_left": "front_right",
        "front_right": "front_left",
        "back_left": "back_right",
        "back_right": "back_left",
    }
    sib = pairs.get(leg_id)
    if sib and sib in legs_meta:
        return np.asarray(legs_meta[sib]["size"], dtype=np.float64)[:2]
    return np.array([0.12, 0.12])


def extend_leg_column(
    leg_id: str,
    extend_cfg: dict[str, Any],
    legs_xyz: np.ndarray,
    legs_meta: dict[str, dict[str, Any]],
) -> np.ndarray:
    if leg_id not in legs_meta:
        raise KeyError(f"leg id {leg_id} not in measured_container.yaml")

    meta = legs_meta[leg_id]
    center = np.asarray(meta["center"], dtype=np.float64)
    default_size = np.asarray(meta.get("size", [0.12, 0.12, 0.12]), dtype=np.float64)

    cluster = _leg_cluster_near(legs_xyz, center[:2], float(extend_cfg.get("cluster_radius", 0.18)))
    fallback_xy = default_size[:2]
    if extend_cfg.get("mirror_sibling_cross_section", True) and cluster.shape[0] < 10:
        fallback_xy = _sibling_leg_size(legs_meta, leg_id)

    cs = extend_cfg.get("cross_section", "auto")
    if cs != "auto" and cs is not None:
        size_xy = np.asarray(cs, dtype=np.float64)[:2]
        axis = np.array([0.0, 0.0, 1.0])
        if cluster.shape[0] >= 10:
            base_z = float(cluster[:, 2].min())
            base_center = np.array([center[0], center[1], base_z])
        else:
            base_center = center.copy()
            base_center[2] = float(meta.get("height", default_size[2]) / 2) - default_size[2] / 2
    else:
        size_xy, _h, _axis, base_center = _cross_section_from_cluster(
            cluster, default_size, center[:2]
        )
        if cluster.shape[0] < 10:
            size_xy = fallback_xy
            base_center = center.copy()
            base_center[2] = 0.0

    length_m = float(extend_cfg["length_m"])
    spacing = float(extend_cfg.get("grid_spacing", 0.01))
    direction = str(extend_cfg.get("direction", "down")).lower()
    if direction == "down":
        axis_vec = -np.array([0.0, 0.0, 1.0])
        start = base_center.copy()
        if cluster.shape[0] >= 10:
            start[2] = float(cluster[:, 2].min())
        else:
            start[2] = float(center[2]) - float(meta.get("height", default_size[2]) / 2)
    elif direction == "up":
        axis_vec = np.array([0.0, 0.0, 1.0])
        start = base_center.copy()
        if cluster.shape[0] >= 10:
            start[2] = float(cluster[:, 2].max())
        else:
            start[2] = float(center[2]) + float(meta.get("height", default_size[2]) / 2)
    else:
        raise ValueError(f"unknown leg direction: {direction}")

    ts = np.arange(spacing, length_m + spacing * 0.5, spacing)
    half_u, half_v = 0.5 * size_xy[0], 0.5 * size_xy[1]
    us = np.arange(-half_u, half_u + spacing * 0.5, spacing)
    vs = np.arange(-half_v, half_v + spacing * 0.5, spacing)
    synth = []
    for t in ts:
        origin = start + axis_vec * t
        for du in us:
            for dv in vs:
                synth.append(origin + np.array([du, dv, 0.0]))
    if not synth:
        return np.empty((0, 3))
    return np.asarray(synth, dtype=np.float64)


def _load_xyz(path: Path) -> np.ndarray:
    xyz, _, _ = load_point_cloud(path)
    return xyz


def complete_cargo_cloud(output_dir: Path, cfg: dict[str, Any]) -> CompletionResult:
    out = Path(output_dir)
    messages: list[str] = []

    body_in = out / cfg.get("body_ply", "body_points.ply")
    leg_in = out / cfg.get("leg_ply", "leg_points.ply")
    if not body_in.exists():
        raise FileNotFoundError(f"Missing {body_in}")

    body_xyz = _load_xyz(body_in)
    leg_xyz = _load_xyz(leg_in) if leg_in.exists() else np.empty((0, 3))

    parametric_model = None
    param_path = out / cfg.get("parametric_model_json", "parametric_model.json")
    if param_path.exists():
        parametric_model = json.loads(param_path.read_text(encoding="utf-8"))

    opening = resolve_opening(cfg.get("opening", {}), parametric_model)
    opening_margin_y = float(cfg.get("opening_margin_y", 0.08))

    synth_body: list[np.ndarray] = []
    synth_body_colors: list[np.ndarray] = []
    for patch in cfg.get("patches") or []:
        if not patch.get("enabled", True):
            continue
        pts, mode_label = fill_planar_patch(
            patch, body_xyz, opening, parametric_model, opening_margin_y
        )
        synth_body.append(pts)
        color = [1.0, 0.15, 0.15] if mode_label == "manual_center" else [1.0, 0.0, 0.8]
        if mode_label == "legacy_uv":
            color = [1.0, 0.35, 0.0]
        synth_body_colors.append(np.tile(color, (pts.shape[0], 1)))
        messages.append(
            f"Patch {patch.get('name', '?')} ({mode_label}): +{pts.shape[0]} synthetic points"
        )

    body_synth = np.vstack(synth_body) if synth_body else np.empty((0, 3))
    body_synth_rgb = (
        np.vstack(synth_body_colors) if synth_body_colors else np.empty((0, 3))
    )
    body_completed = body_xyz if body_synth.size == 0 else np.vstack([body_xyz, body_synth])

    measured_path = out / cfg.get("measured_container_yaml", "measured_container.yaml")
    legs_meta = _load_measured_legs(measured_path)

    synth_leg: list[np.ndarray] = []
    for ext in cfg.get("leg_extends") or []:
        if not ext.get("enabled", True):
            continue
        leg_id = str(ext["id"])
        pts = extend_leg_column(leg_id, ext, leg_xyz, legs_meta)
        synth_leg.append(pts)
        messages.append(f"Leg extend {leg_id}: +{pts.shape[0]} synthetic points")

    leg_synth = np.vstack(synth_leg) if synth_leg else np.empty((0, 3))
    leg_completed = leg_xyz if leg_synth.size == 0 else np.vstack([leg_xyz, leg_synth])

    body_out = out / cfg.get("body_out_ply", "body_points_completed.ply")
    leg_out = out / cfg.get("leg_out_ply", "leg_points_completed.ply")
    save_ply(body_out, body_completed)
    save_ply(leg_out, leg_completed)

    debug_path = None
    if cfg.get("write_debug", True):
        debug_path = out / cfg.get("debug_ply", "completion_debug.ply")
        dbg_parts: list[np.ndarray] = []
        rgb_parts: list[np.ndarray] = []
        if body_xyz.size:
            dbg_parts.append(body_xyz)
            rgb_parts.append(np.full((body_xyz.shape[0], 3), 0.55))
        if body_synth.size:
            dbg_parts.append(body_synth)
            rgb_parts.append(body_synth_rgb)
        if leg_xyz.size:
            dbg_parts.append(leg_xyz)
            rgb_parts.append(np.full((leg_xyz.shape[0], 3), 0.35))
        if leg_synth.size:
            dbg_parts.append(leg_synth)
            rgb_parts.append(np.tile([1.0, 0.5, 0.1], (leg_synth.shape[0], 1)))
        if dbg_parts:
            save_ply(debug_path, np.vstack(dbg_parts), np.vstack(rgb_parts))

    messages.append(
        f"body {body_xyz.shape[0]} + {body_synth.shape[0]} -> {body_out.name}; "
        f"leg {leg_xyz.shape[0]} + {leg_synth.shape[0]} -> {leg_out.name}"
    )

    return CompletionResult(
        body_path=body_out,
        leg_path=leg_out,
        debug_path=debug_path,
        body_original=int(body_xyz.shape[0]),
        body_synthetic=int(body_synth.shape[0]),
        leg_original=int(leg_xyz.shape[0]),
        leg_synthetic=int(leg_synth.shape[0]),
        messages=messages,
    )
