"""Render one D455 observation from a MuJoCo suitcase scene."""

from __future__ import division

import os

import numpy as np

from research.lrf_p1.mujoco_d455.camera import (
    apply_d455_noise,
    camera_state,
    depth_to_cloud,
    render_rgbd,
)
from research.lrf_p1.mujoco_d455.d455 import depth_intrinsics
from research.lrf_p1.mujoco_d455.scene import build_xml


def suitcase_gt(visual_id, tier, yaw, models_root):
    from luggage_description.suitcase_visual import (
        mesh_observable_reference,
        sized_stl_path,
    )
    stl = sized_stl_path(visual_id, tier, models_root)
    obs_w, obs_d, lid, full_h = mesh_observable_reference(stl)
    gt_h = float(full_h - lid)
    return {
        "stl": stl,
        "full_h": float(full_h),
        "gt": {
            "width": float(obs_w),
            "depth": float(obs_d),
            "height": gt_h,
            "center_xyz": [0.0, 0.0, gt_h * 0.5],
            "yaw": float(yaw),
        },
    }


def render_observation(spec, models_root, rng, table_z=0.02):
    """Return rgb, depth, world cloud, observation dict, gt, camera state."""
    import mujoco as mj

    info = suitcase_gt(spec["visual_id"], spec["tier"], spec["yaw"], models_root)
    z_body = info["full_h"] * 0.5
    xml, meta = build_xml(
        mesh_path=info["stl"],
        meshdir=os.path.dirname(info["stl"]),
        suitcase_xy=(0.0, 0.0),
        suitcase_yaw=spec["yaw"],
        suitcase_z=z_body,
        occluder=spec.get("occluder"),
        cam_distance=float(spec.get("distance", 1.05)),
        cam_azimuth=float(spec.get("azimuth", -0.5 * np.pi)),
        cam_elevation=float(spec.get("elevation", 0.38)),
    )
    model = mj.MjModel.from_xml_string(xml)
    data = mj.MjData(model)
    mj.mj_forward(model, data)
    rgb, depth = render_rgbd(model, data)
    depth_n = apply_d455_noise(depth, rng)
    cam = camera_state(model, data)
    k = depth_intrinsics()
    cloud = depth_to_cloud(depth_n, k, cam["R_cv"], cam["t"])
    cloud_use = cloud[cloud[:, 2] >= float(table_z)]
    observation = {
        "points": cloud_use,
        "roi_center_xy": (
            float(np.median(cloud_use[:, 0])) if len(cloud_use) else 0.0,
            float(np.median(cloud_use[:, 1])) if len(cloud_use) else 0.0,
        ),
        "frame_id": "world",
        "stamp": 0.0,
    }
    return {
        "rgb": rgb,
        "depth": depth_n,
        "observation": observation,
        "gt": info["gt"],
        "cam": cam,
        "K": k,
        "meta": meta,
        "n_raw": int(len(cloud)),
        "n_points": int(len(cloud_use)),
    }


def enumerate_train_specs():
    """loafbrr only. Vintage meshes stay held out."""
    yaws = (0.0, 0.7, 1.57)
    azimuths = (-0.5 * np.pi, -1.15, -2.0)
    occluders = (
        None,
        {"pos": [-0.18, -0.28, 0.12], "size": [0.16, 0.02, 0.12]},
        {"pos": [0.18, -0.28, 0.12], "size": [0.16, 0.02, 0.12]},
    )
    specs = []
    for tier in ("small", "medium", "large"):
        for yaw in yaws:
            for az in azimuths:
                for occ in occluders:
                    specs.append({
                        "visual_id": "suitcase_loafbrr",
                        "tier": tier,
                        "yaw": yaw,
                        "azimuth": az,
                        "elevation": 0.38,
                        "distance": 1.05,
                        "occluder": occ,
                    })
    return specs
