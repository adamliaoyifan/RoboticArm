"""Deterministic synthetic RGB-D-like observations from sized suitcase STLs.

Generation may use eval-only mesh identity and visibility ratio. The objects
returned to inference contain only points and an observation-derived ROI.
"""

from __future__ import division

import hashlib
import math
import os
import struct

import numpy as np

from luggage_description.suitcase_visual import (
    VISUAL_IDS,
    SIZE_TIERS,
    mesh_observable_reference,
    sized_stl_path,
    stl_sha256,
)

PLATFORM_Z = 0.86
MODELS_ROOT_REL = os.path.join("src", "luggage_gazebo", "models")


def default_models_root(workspace):
    return os.path.join(workspace, MODELS_ROOT_REL)


_TRIANGLE_CACHE = {}


def load_triangles(path):
    cached = _TRIANGLE_CACHE.get(path)
    if cached is not None:
        return cached
    tris = []
    with open(path, "rb") as handle:
        header = handle.read(80)
        if len(header) < 80:
            raise IOError("truncated STL header: %s" % path)
        n_raw = handle.read(4)
        if len(n_raw) < 4:
            raise IOError("truncated STL triangle count: %s" % path)
        count = struct.unpack("<I", n_raw)[0]
        for _ in range(count):
            rec = handle.read(50)
            if len(rec) < 50:
                raise IOError("truncated STL triangles: %s" % path)
            nums = struct.unpack("<12fH", rec)
            tris.append((nums[3:6], nums[6:9], nums[9:12]))
    stacked = np.asarray(tris, dtype=np.float64)
    _TRIANGLE_CACHE[path] = stacked
    return stacked


def sample_surface(triangles, n_points, rng):
    verts = np.asarray(triangles, dtype=np.float64)
    if verts.ndim != 3 or verts.shape[1:] != (3, 3):
        raise ValueError("triangles must be (N,3,3)")
    a, b, c = verts[:, 0], verts[:, 1], verts[:, 2]
    areas = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
    total = float(areas.sum())
    if total <= 0.0:
        raise ValueError("zero-area mesh")
    idx = rng.choice(len(verts), size=int(n_points), p=areas / total)
    u = rng.random(int(n_points))
    v = rng.random(int(n_points))
    swap = u + v > 1.0
    u = np.where(swap, 1.0 - u, u)
    v = np.where(swap, 1.0 - v, v)
    w = 1.0 - u - v
    chosen = verts[idx]
    return (w[:, None] * chosen[:, 0]
            + u[:, None] * chosen[:, 1]
            + v[:, None] * chosen[:, 2])


def _rot_z(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def place_points(local_points, yaw, xy, platform_z):
    pts = np.asarray(local_points, dtype=np.float64)
    zmin = float(pts[:, 2].min())
    rotated = pts.dot(_rot_z(yaw).T)
    rotated[:, 0] += float(xy[0])
    rotated[:, 1] += float(xy[1])
    rotated[:, 2] += float(platform_z) - zmin
    center = np.array([
        float(xy[0]),
        float(xy[1]),
        float(rotated[:, 2].mean()),
    ])
    return rotated, center


def visible_subset(points, center, camera, keep_frac, rng):
    pts = np.asarray(points, dtype=np.float64)
    view = np.asarray(camera, dtype=np.float64) - pts
    outward = pts - np.asarray(center, dtype=np.float64)
    front = (outward * view).sum(axis=1) > 0.0
    kept = pts[front]
    if len(kept) == 0:
        return kept, 0.0
    keep_frac = min(1.0, max(0.05, float(keep_frac)))
    n_keep = max(1, int(round(len(kept) * keep_frac)))
    if n_keep >= len(kept):
        ratio = float(len(kept)) / float(max(1, len(pts)))
        return kept, ratio
    order = rng.permutation(len(kept))[:n_keep]
    subset = kept[order]
    ratio = float(len(subset)) / float(max(1, len(pts)))
    return subset, ratio


def add_sensor_noise(points, sigma, rng):
    if sigma <= 0.0:
        return np.asarray(points, dtype=np.float64).copy()
    noise = rng.normal(0.0, float(sigma), size=np.asarray(points).shape)
    return np.asarray(points, dtype=np.float64) + noise


def occlusion_bucket(visible_ratio):
    ratio = float(visible_ratio)
    if ratio > 0.70:
        return "light"
    if ratio >= 0.40:
        return "medium"
    if ratio >= 0.20:
        return "heavy"
    return "degenerate"


def observation_id(fields):
    blob = "|".join("%s=%s" % (k, fields[k]) for k in sorted(fields))
    return hashlib.sha256(blob.encode("utf-8")).hexdigest()[:16]


def enumerate_specs():
    """Frozen generation grid. Mesh identity is for splitting only."""
    yaws = (0.0, 0.7, 1.57)
    xys = ((-1.0, 0.0), (-1.05, 0.05))
    azimuths = (1.57, 0.7, 2.5)
    keep_fracs = (1.0, 0.55, 0.28)
    elevations = (0.55, 0.35)
    specs = []
    for visual_id in VISUAL_IDS:
        for tier, _size, _mass, _cat in SIZE_TIERS:
            for yaw in yaws:
                for xy in xys:
                    for az in azimuths:
                        for keep in keep_fracs:
                            elev = elevations[0] if keep >= 0.99 else elevations[1]
                            control = bool(keep >= 0.99 and abs(az - 1.57) < 1e-9)
                            specs.append({
                                "visual_id": visual_id,
                                "tier": tier,
                                "yaw": yaw,
                                "xy": xy,
                                "azimuth": az,
                                "keep_frac": keep,
                                "elevation": elev,
                                "unoccluded_control": control,
                            })
    return specs


def split_name(visual_id, tier):
    if visual_id == "suitcase_loafbrr":
        return "train"
    if visual_id == "suitcase_vintage" and tier == "small":
        return "val"
    return "test"


def build_sample(workspace, spec, rng, n_surface=900, noise_sigma=0.003):
    models_root = default_models_root(workspace)
    stl = sized_stl_path(spec["visual_id"], spec["tier"], models_root)
    triangles = load_triangles(stl)
    local = sample_surface(triangles, n_surface, rng)
    placed, center = place_points(
        local, spec["yaw"], spec["xy"], PLATFORM_Z)
    dist = 0.85
    camera = np.array([
        center[0] + dist * math.cos(spec["azimuth"]),
        center[1] + dist * math.sin(spec["azimuth"]),
        PLATFORM_Z + spec["elevation"] + 0.35,
    ])
    if spec["keep_frac"] >= 0.99:
        visible = placed
        ratio = 1.0
    else:
        visible, ratio = visible_subset(
            placed, center, camera, spec["keep_frac"], rng)
        # Bucket by the kept fraction of the camera-visible set, not of the
        # full watertight surface (the hidden back is UNKNOWN, not a second
        # occlusion process).
        ratio = float(spec["keep_frac"])
    noisy = add_sensor_noise(visible, noise_sigma, rng)
    ref = mesh_observable_reference(stl)
    gt_w, gt_d, lid, full_h = ref
    gt_h = full_h - lid
    gt_center = np.array([
        float(spec["xy"][0]),
        float(spec["xy"][1]),
        PLATFORM_Z + gt_h * 0.5,
    ])
    sample_fields = {
        "visual_id": spec["visual_id"],
        "tier": spec["tier"],
        "yaw": "%.5f" % spec["yaw"],
        "xy": "%.3f,%.3f" % spec["xy"],
        "azimuth": "%.3f" % spec["azimuth"],
        "keep": "%.3f" % spec["keep_frac"],
    }
    sid = observation_id(sample_fields)
    eval_record = {
        "sample_id": sid,
        "split": split_name(spec["visual_id"], spec["tier"]),
        "mesh_id": "%s_%s" % (spec["visual_id"], spec["tier"]),
        "visual_id": spec["visual_id"],
        "tier": spec["tier"],
        "stl_sha256": stl_sha256(stl),
        "visible_surface_ratio": ratio,
        "occlusion_bucket": occlusion_bucket(ratio),
        "unoccluded_control": bool(spec["unoccluded_control"]),
        "gt": {
            "width": float(gt_w),
            "depth": float(gt_d),
            "height": float(gt_h),
            "center_xyz": gt_center.tolist(),
            "yaw": float(spec["yaw"]),
        },
    }
    observation = {
        "points": noisy,
        "roi_center_xy": (
            float(np.median(noisy[:, 0])) if len(noisy) else 0.0,
            float(np.median(noisy[:, 1])) if len(noisy) else 0.0,
        ),
        "frame_id": "world",
        "stamp": 0.0,
    }
    extra_views = []
    if spec["keep_frac"] < 0.99:
        cam2 = np.array([
            center[0] + dist * math.cos(spec["azimuth"] + 0.7),
            center[1] + dist * math.sin(spec["azimuth"] + 0.7),
            camera[2],
        ])
        vis2, _ratio2 = visible_subset(
            placed, center, cam2, min(1.0, spec["keep_frac"] + 0.15), rng)
        extra_views.append(add_sensor_noise(vis2, noise_sigma, rng))
    return observation, eval_record, extra_views
