"""Align Mid-360S to D555: camera Layer 3 is the reference.

CAD ``mid360_mount_*`` on eef_mount_adapter is the only joint we freeze.
Handbook ``livox_optical_*`` (47 mm) and IMU stay untouched.
"""

from __future__ import division

import math
import os
import re

import numpy as np
from scipy.spatial import cKDTree

from luggage_description.handeye_layer3 import R_to_rpy, T_xyz_rpy, parse_xacro_xyz_rpy, rpy_to_R

_CONFIG = os.path.join(os.path.dirname(__file__), "..", "config")
HANDBOOK_OPTICAL_XYZ = (0.0, 0.0, 0.047)
HANDBOOK_OPTICAL_RPY = (0.0, 0.0, 0.0)
CAD_MOUNT_XYZ = (0.022, 0.103, 0.038)
CAD_MOUNT_RPY = (0.0, 1.57079633, 1.57079633)


def invert_T(T):
    out = np.eye(4)
    Rt = np.asarray(T[:3, :3], dtype=np.float64).T
    out[:3, :3] = Rt
    out[:3, 3] = -Rt.dot(T[:3, 3])
    return out


def transform_points(T, points):
    pts = np.asarray(points, dtype=np.float64)
    if pts.size == 0:
        return pts.reshape(0, 3)
    return pts.dot(T[:3, :3].T) + T[:3, 3]


def finite_xyz(points, min_range=0.05, max_range=20.0):
    pts = np.asarray(points, dtype=np.float64)
    if pts.size == 0:
        return pts.reshape(0, 3)
    ok = np.isfinite(pts).all(axis=1)
    rad = np.linalg.norm(pts, axis=1)
    ok &= rad > float(min_range)
    ok &= rad < float(max_range)
    return pts[ok]


def voxel_downsample(points, voxel):
    pts = np.asarray(points, dtype=np.float64)
    if pts.shape[0] == 0 or voxel <= 0:
        return pts
    keys = np.floor(pts / float(voxel)).astype(np.int64)
    # Pack so np.unique is 1-D.
    span = keys.max(axis=0) - keys.min(axis=0) + 1
    span = np.maximum(span, 1)
    packed = (
        (keys[:, 0] - keys[:, 0].min())
        + (keys[:, 1] - keys[:, 1].min()) * span[0]
        + (keys[:, 2] - keys[:, 2].min()) * span[0] * span[1]
    )
    _, idx = np.unique(packed, return_index=True)
    return pts[np.sort(idx)]


def frustum_mask(points, fx, fy, cx, cy, width, height, z_min, z_max):
    """Keep points in a pinhole image. points are in the camera optical frame."""
    pts = np.asarray(points, dtype=np.float64)
    if pts.size == 0:
        return np.zeros((0,), dtype=bool)
    z = pts[:, 2]
    ok = (z > float(z_min)) & (z < float(z_max)) & np.isfinite(z)
    u = fx * pts[:, 0] / np.clip(z, 1e-6, None) + cx
    v = fy * pts[:, 1] / np.clip(z, 1e-6, None) + cy
    ok &= (u >= 0.0) & (u < float(width)) & (v >= 0.0) & (v < float(height))
    return ok


def estimate_normals(points, k=12):
    pts = np.asarray(points, dtype=np.float64)
    n_pts = pts.shape[0]
    normals = np.zeros_like(pts)
    if n_pts < k:
        return normals
    tree = cKDTree(pts)
    _dist, idx = tree.query(pts, k=k)
    for i in range(n_pts):
        neigh = pts[idx[i]]
        centered = neigh - neigh.mean(axis=0)
        _u, _s, vh = np.linalg.svd(centered, full_matrices=False)
        normal = vh[-1]
        if np.dot(normal, pts[i]) > 0.0:
            normal = -normal
        normals[i] = normal
    return normals


def _exp_se3(omega, trans):
    angle = float(np.linalg.norm(omega))
    T = np.eye(4)
    if angle < 1e-12:
        T[:3, 3] = trans
        return T
    axis = omega / angle
    kx, ky, kz = axis
    K = np.array([[0.0, -kz, ky], [kz, 0.0, -kx], [-ky, kx, 0.0]])
    rot = np.eye(3) + math.sin(angle) * K + (1.0 - math.cos(angle)) * K.dot(K)
    T[:3, :3] = rot
    T[:3, 3] = trans
    return T


def icp_point_to_plane(
    source,
    target,
    max_iter=25,
    max_dist=0.20,
    voxel=0.025,
    min_pairs=80,
):
    """Return T such that T @ source ≈ target (point-to-plane)."""
    src0 = voxel_downsample(finite_xyz(source), voxel)
    dst = voxel_downsample(finite_xyz(target), voxel)
    if src0.shape[0] < min_pairs or dst.shape[0] < min_pairs:
        raise RuntimeError(
            "not enough overlap for ICP (src=%d dst=%d)"
            % (src0.shape[0], dst.shape[0])
        )
    normals = estimate_normals(dst)
    T = np.eye(4)
    last = None
    tree = cKDTree(dst)
    thresh = float(max_dist)
    for it in range(int(max_iter)):
        src = transform_points(T, src0)
        dist, idx = tree.query(src, k=1)
        keep = dist < thresh
        if int(keep.sum()) < min_pairs:
            if last is not None:
                break
            thresh *= 1.5
            continue
        p = src[keep]
        q = dst[idx[keep]]
        n = normals[idx[keep]]
        n = n / np.clip(np.linalg.norm(n, axis=1, keepdims=True), 1e-9, None)
        cross = np.cross(p, n)
        A = np.hstack([cross, n])
        b = np.sum(n * (q - p), axis=1)
        delta, _resid, _rank, _s = np.linalg.lstsq(A, b, rcond=None)
        step = _exp_se3(delta[:3], delta[3:])
        T = step.dot(T)
        rmse = float(np.sqrt(np.mean(dist[keep] ** 2)))
        last = {
            "iter": it + 1,
            "pairs": int(keep.sum()),
            "rmse": rmse,
            "median": float(np.median(dist[keep])),
        }
        if np.linalg.norm(delta) < 1e-5:
            break
        thresh = max(0.04, 0.6 * thresh)
    if last is None:
        raise RuntimeError("ICP found no correspondences")
    last["T"] = T
    last["n_src"] = int(src0.shape[0])
    last["n_dst"] = int(dst.shape[0])
    return T, last


def nn_rmse(source, target, max_dist=0.25):
    src = voxel_downsample(finite_xyz(source), 0.03)
    dst = voxel_downsample(finite_xyz(target), 0.03)
    if src.shape[0] < 10 or dst.shape[0] < 10:
        return float("nan"), 0
    dist, _idx = cKDTree(dst).query(src, k=1)
    keep = dist < float(max_dist)
    if int(keep.sum()) < 10:
        return float(np.median(dist)), int(keep.sum())
    return float(np.sqrt(np.mean(dist[keep] ** 2))), int(keep.sum())


def T_handbook_optical():
    return T_xyz_rpy(HANDBOOK_OPTICAL_XYZ, HANDBOOK_OPTICAL_RPY)


def T_cad_mount(config_dir=None):
    config_dir = config_dir or _CONFIG
    path = os.path.join(config_dir, "mid360_origin.xacro")
    xyz, rpy = parse_xacro_xyz_rpy(path, "mid360_mount_xyz", "mid360_mount_rpy")
    return T_xyz_rpy(xyz, rpy)


def T_adapter_livox_from_xacro(config_dir=None):
    return T_cad_mount(config_dir).dot(T_handbook_optical())


def wrap_pi(angle):
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def rpy_near_seed(rot, seed):
    """Pick an Euler triple for rot that stays close to seed (gimbal lock)."""
    seed = [float(v) for v in seed]
    primary = list(R_to_rpy(rot))
    alt = [primary[0] + math.pi, math.pi - primary[1], primary[2] + math.pi]
    best = primary
    best_err = None
    for base in (primary, alt):
        for dr in (-2.0 * math.pi, 0.0, 2.0 * math.pi):
            for dy in (-2.0 * math.pi, 0.0, 2.0 * math.pi):
                cand = [base[0] + dr, base[1], base[2] + dy]
                if not np.allclose(rpy_to_R(*cand), rot, atol=1e-6):
                    continue
                err = sum(wrap_pi(cand[i] - seed[i]) ** 2 for i in range(3))
                if best_err is None or err < best_err:
                    best, best_err = cand, err
    return [wrap_pi(v) for v in best]


def mount_from_adapter_livox(T_adapter_livox, rpy_seed=None):
    """Undo handbook optical so URDF still stores mount + 47 mm separately."""
    T_mount = np.asarray(T_adapter_livox, dtype=np.float64).dot(
        invert_T(T_handbook_optical())
    )
    xyz = [float(v) for v in T_mount[:3, 3]]
    seed = rpy_seed if rpy_seed is not None else CAD_MOUNT_RPY
    rpy = rpy_near_seed(T_mount[:3, :3], seed)
    return xyz, rpy


def apply_cloud_correction(T_adapter_livox, T_corr_adapter):
    """T_corr maps current adapter-frame Livox points onto D555 points."""
    return np.asarray(T_corr_adapter, dtype=np.float64).dot(
        np.asarray(T_adapter_livox, dtype=np.float64)
    )


def rotation_deg(T):
    rot = np.asarray(T[:3, :3], dtype=np.float64)
    tr = float(np.clip((np.trace(rot) - 1.0) * 0.5, -1.0, 1.0))
    return math.degrees(math.acos(tr))


def translation_m(T):
    return float(np.linalg.norm(T[:3, 3]))


def conjugate_se3(T_parent_from_a, T_in_a):
    """Express T_in_a (acting on frame-a points) in the parent frame."""
    parent_from_a = np.asarray(T_parent_from_a, dtype=np.float64)
    return parent_from_a.dot(T_in_a).dot(invert_T(parent_from_a))


def height_above_plane(origin, normal, offset):
    """Signed height: normal · origin - offset, with plane normal · x = offset."""
    nrm = np.asarray(normal, dtype=np.float64)
    nrm = nrm / np.linalg.norm(nrm)
    return float(np.dot(origin, nrm) - float(offset))


def set_origin_height(T_world_frame, normal, offset, target_height):
    """Slide the frame origin along the plane normal; keep rotation."""
    out = np.asarray(T_world_frame, dtype=np.float64).copy()
    nrm = np.asarray(normal, dtype=np.float64)
    nrm = nrm / np.linalg.norm(nrm)
    current = height_above_plane(out[:3, 3], nrm, offset)
    out[:3, 3] = out[:3, 3] + nrm * (float(target_height) - current)
    return out, current


def fit_horizontal_plane(
    points,
    rng=None,
    n_iter=140,
    thresh=0.03,
    min_abs_nz=0.85,
    origin=None,
    expected_origin_height=None,
    height_tol=0.12,
):
    """RANSAC a mostly-vertical-normal plane. Returns unit normal and offset.

    If ``expected_origin_height`` is set, keep the plane whose height at
    ``origin`` (default 0) matches the pedestal, not a nearer table top.
    """
    pts = np.asarray(points, dtype=np.float64)
    if pts.shape[0] < 50:
        raise RuntimeError("not enough points to fit a floor plane")
    rng = np.random.default_rng() if rng is None else rng
    origin = np.zeros(3) if origin is None else np.asarray(origin, dtype=np.float64)
    best = None
    for _ in range(int(n_iter)):
        idx = rng.choice(pts.shape[0], 3, replace=False)
        a, b, c = pts[idx]
        nrm = np.cross(b - a, c - a)
        length = np.linalg.norm(nrm)
        if length < 1e-8:
            continue
        nrm = nrm / length
        if abs(nrm[2]) < float(min_abs_nz):
            continue
        if nrm[2] < 0.0:
            nrm = -nrm
        offset = float(a.dot(nrm))
        if expected_origin_height is not None:
            h0 = height_above_plane(origin, nrm, offset)
            if abs(h0 - float(expected_origin_height)) > float(height_tol):
                continue
        inl = np.abs((pts - a).dot(nrm)) < float(thresh)
        score = int(inl.sum())
        if best is None or score > best[0]:
            best = (score, nrm.copy(), pts[inl])
    if best is None:
        raise RuntimeError("no horizontal plane inliers")
    inliers = best[2]
    centroid = inliers.mean(axis=0)
    _u, _s, vh = np.linalg.svd(inliers - centroid, full_matrices=False)
    nrm = vh[-1]
    if nrm[2] < 0.0:
        nrm = -nrm
    if abs(nrm[2]) < float(min_abs_nz):
        nrm = best[1]
    return nrm, float(centroid.dot(nrm))


def replace_mid360_mount(text, xyz, rpy, note):
    out = re.sub(
        r'name="mid360_mount_xyz" value="[^"]+"',
        'name="mid360_mount_xyz" value="%.6f %.6f %.6f"'
        % (xyz[0], xyz[1], xyz[2]),
        text,
        count=1,
    )
    out = re.sub(
        r'name="mid360_mount_rpy" value="[^"]+"',
        'name="mid360_mount_rpy" value="%.8f %.8f %.8f"'
        % (rpy[0], rpy[1], rpy[2]),
        out,
        count=1,
    )
    banner = "<!-- %s -->\n" % note
    if note and banner.strip() not in out:
        out = out.replace('<?xml version="1.0"?>\n', '<?xml version="1.0"?>\n' + banner, 1)
    return out
