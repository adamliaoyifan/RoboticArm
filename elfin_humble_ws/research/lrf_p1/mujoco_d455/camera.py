"""Render D455 RGB-D from MuJoCo and back-project depth to a world XYZ cloud."""

from __future__ import division

import os

import numpy as np

from research.lrf_p1.mujoco_d455 import d455


def _require_egl():
    os.environ.setdefault("MUJOCO_GL", "egl")


def mj_camera_opencv_rotation(xmat):
    """MuJoCo camera axes (X right, Y up, -Z look) -> OpenCV (X right, Y down, +Z look)."""
    r_mj = np.asarray(xmat, dtype=np.float64).reshape(3, 3)
    return r_mj @ np.diag([1.0, -1.0, -1.0])


def render_rgbd(model, data, camera="d455_depth", width=None, height=None):
    import mujoco as mj
    _require_egl()
    width = int(width or d455.DEPTH_WIDTH)
    height = int(height or d455.DEPTH_HEIGHT)
    renderer = mj.Renderer(model, height=height, width=width)
    try:
        renderer.update_scene(data, camera=camera)
        rgb = renderer.render().copy()
        renderer.enable_depth_rendering()
        renderer.update_scene(data, camera=camera)
        depth = renderer.render().copy()
    finally:
        renderer.close()
    return rgb, depth


def apply_d455_noise(depth, rng, sigma=d455.RANGE_NOISE_SIGMA_M, dropout=d455.DROPOUT_RATE):
    z = np.asarray(depth, dtype=np.float64).copy()
    if sigma > 0.0:
        z = z + rng.normal(0.0, float(sigma), size=z.shape)
    if dropout > 0.0:
        drop = rng.random(z.shape) < float(dropout)
        z[drop] = np.nan
    return z


def depth_to_cloud(depth, K, R_cv, t_world, zmin=d455.DEPTH_RANGE_MIN_M, zmax=d455.DEPTH_RANGE_MAX_M):
    """Back-project metric depth (OpenCV) to world XYZ.

    Invalid / out-of-range pixels are dropped. This is a single frame.
    """
    z = np.asarray(depth, dtype=np.float64)
    h, w = z.shape
    fx, fy, cx, cy = K["fx"], K["fy"], K["cx"], K["cy"]
    vs, us = np.indices((h, w))
    valid = np.isfinite(z) & (z >= float(zmin)) & (z <= float(zmax))
    if not np.any(valid):
        return np.zeros((0, 3), dtype=np.float64)
    zv = z[valid]
    x = (us[valid] - cx) * zv / fx
    y = (vs[valid] - cy) * zv / fy
    p_cv = np.stack([x, y, zv], axis=1)
    r = np.asarray(R_cv, dtype=np.float64).reshape(3, 3)
    t = np.asarray(t_world, dtype=np.float64).reshape(3)
    return p_cv.dot(r.T) + t


def camera_state(model, data, camera="d455_depth"):
    import mujoco as mj
    cid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_CAMERA, camera)
    if cid < 0:
        raise ValueError("camera not found: %s" % camera)
    return {
        "R_cv": mj_camera_opencv_rotation(data.cam_xmat[cid]),
        "t": np.asarray(data.cam_xpos[cid], dtype=np.float64).copy(),
        "R_mj": np.asarray(data.cam_xmat[cid], dtype=np.float64).reshape(3, 3),
    }


def project_world(points_world, R_cv, t, K):
    pts = np.asarray(points_world, dtype=np.float64).reshape(-1, 3)
    p_cv = (pts - t.reshape(1, 3)).dot(R_cv)
    z = p_cv[:, 2]
    u = K["fx"] * p_cv[:, 0] / np.maximum(z, 1e-9) + K["cx"]
    v = K["fy"] * p_cv[:, 1] / np.maximum(z, 1e-9) + K["cy"]
    return u, v, z
