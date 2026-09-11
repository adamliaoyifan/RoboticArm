"""Build a kinematic MuJoCo pickup scene: platform, one suitcase, D455."""

from __future__ import division

import os

import numpy as np

from research.lrf_p1.mujoco_d455 import d455


def lookat_xyaxes(pos, target, world_up=(0.0, 0.0, 1.0)):
    """MuJoCo camera xyaxes: X=right, Y=up, looks along -Z toward target."""
    pos = np.asarray(pos, dtype=np.float64)
    target = np.asarray(target, dtype=np.float64)
    look = target - pos
    n = np.linalg.norm(look)
    if n < 1e-9:
        raise ValueError("camera pos equals target")
    look = look / n
    up = np.asarray(world_up, dtype=np.float64)
    right = np.cross(look, up)
    rn = np.linalg.norm(right)
    if rn < 1e-9:
        raise ValueError("look parallel to world up")
    right = right / rn
    yup = np.cross(right, look)
    yup = yup / np.linalg.norm(yup)
    return right, yup, look


def camera_pose(target, distance=1.05, azimuth_rad=-0.5 * np.pi, elevation_rad=0.38):
    """Place the D455 relative to the suitcase center."""
    target = np.asarray(target, dtype=np.float64)
    ce, se = math_cos_sin(elevation_rad)
    ca, sa = math_cos_sin(azimuth_rad)
    offset = np.array([
        distance * ce * ca,
        distance * ce * sa,
        distance * se,
    ])
    pos = target + offset
    return pos, target


def math_cos_sin(angle):
    return float(np.cos(angle)), float(np.sin(angle))


def _fmt_vec(vec):
    return " ".join("%.6f" % float(v) for v in vec)


def build_xml(
    mesh_path,
    meshdir,
    suitcase_xy=(0.0, 0.0),
    suitcase_yaw=0.0,
    suitcase_z=0.14,
    occluder=None,
    cam_distance=1.05,
    cam_azimuth=-0.5 * np.pi,
    cam_elevation=0.38,
    width=d455.DEPTH_WIDTH,
    height=d455.DEPTH_HEIGHT,
    vfov_deg=d455.DEPTH_VFOV_DEG,
):
    """Return (xml_string, meta dict). Suitcase mesh origin is AABB center."""
    target = np.array([
        float(suitcase_xy[0]),
        float(suitcase_xy[1]),
        float(suitcase_z),
    ])
    cam_pos, _ = camera_pose(target, cam_distance, cam_azimuth, cam_elevation)
    right, yup, look = lookat_xyaxes(cam_pos, target)
    xyaxes = np.concatenate([right, yup])
    hx, hy, hz = d455.HOUSING_SIZE
    # Housing sits slightly behind the optical origin along +Z_mj = -look.
    housing_pos = cam_pos - 0.15 * look
    occluder_xml = ""
    if occluder:
        occluder_xml = """
    <geom name="occluder" type="box" pos="%s" size="%s" rgba="0.15 0.15 0.18 1"/>
""" % (_fmt_vec(occluder["pos"]), _fmt_vec(occluder["size"]))
    mesh_file = os.path.basename(mesh_path)
    xml = """<mujoco model="lrf_p1_d455_scene">
  <compiler angle="radian" meshdir="%s" autolimits="true"/>
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <visual>
    <global offwidth="%d" offheight="%d"/>
    <map znear="0.05" zfar="20"/>
    <headlight ambient="0.4 0.4 0.4" diffuse="0.4 0.4 0.4"/>
  </visual>
  <asset>
    <mesh name="suitcase" file="%s"/>
    <texture name="grid" type="2d" builtin="checker" width="256" height="256"
             rgb1="0.55 0.55 0.55" rgb2="0.7 0.7 0.7"/>
    <material name="platform" texture="grid" texrepeat="8 8" reflectance="0.05"/>
  </asset>
  <worldbody>
    <light name="key" pos="0.8 -1.0 2.2" dir="-0.2 0.3 -1" diffuse="0.9 0.9 0.9"
           specular="0.2 0.2 0.2"/>
    <light name="fill" pos="-0.8 0.4 1.8" dir="0.3 -0.1 -1" diffuse="0.45 0.45 0.45"/>
    <geom name="platform" type="box" size="0.9 0.9 0.03" pos="0 0 -0.03"
          material="platform" friction="0.8 0.01 0.001"/>
    <body name="suitcase" pos="%s" euler="0 0 %.6f">
      <geom name="suitcase_mesh" type="mesh" mesh="suitcase"
            rgba="0.48 0.36 0.20 1" contype="0" conaffinity="0"/>
    </body>
    %s
    <camera name="d455_depth" pos="%s" xyaxes="%s" fovy="%.6f"/>
    <geom name="d455_housing" type="box" pos="%s" size="%.5f %.5f %.5f"
          rgba="0.12 0.12 0.14 1" contype="0" conaffinity="0"/>
    <site name="d455_imu" pos="%s" size="0.005"/>
  </worldbody>
</mujoco>
""" % (
        meshdir,
        int(width),
        int(height),
        mesh_file,
        _fmt_vec(target),
        float(suitcase_yaw),
        occluder_xml,
        _fmt_vec(cam_pos),
        _fmt_vec(xyaxes),
        float(vfov_deg),
        _fmt_vec(housing_pos),
        hx, hy, hz,
        _fmt_vec(cam_pos + np.array([0.02, 0.0, 0.01])),
    )
    meta = {
        "sensor": d455.NAME,
        "has_imu": d455.HAS_IMU,
        "cam_pos": cam_pos.tolist(),
        "target": target.tolist(),
        "look": look.tolist(),
        "suitcase_xy": [float(suitcase_xy[0]), float(suitcase_xy[1])],
        "suitcase_yaw": float(suitcase_yaw),
        "suitcase_z": float(suitcase_z),
        "occluder": occluder,
        "input": (
            "single-frame D455-like pinhole depth -> XYZ cloud; "
            "not lidar, not a ROS bag, not time-series"
        ),
    }
    return xml, meta
