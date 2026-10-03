#!/usr/bin/env python3
"""Synthesize the deployment-scene SIM pickup-observe pose (ST09-I S5.2).

The mirrored deployment cell (scene_tf_deploy.yaml) keeps the pickup
platform at world (+1, 0) while the stock sim pickup_observe aims the
camera at world (-1, 0) — the empty side. This script derives the
equivalent overhead camera target on the +X side, converts it to a
suction TCP target through the FIXED camera-to-TCP transform read off
the URDF (never solved as a camera pose directly), refines with the
deterministic offline IK, validates limits + FK, and writes
``robot_poses.sim_deploy.yaml``.

SIMULATION ONLY — a clearly-marked synthesized start for the S5 Gazebo
runner. robot_poses.deploy.yaml stays all-valid:false; nothing here is
a measured site pose and nothing here may move the real cell.

Usage (worktree root, ws sourced):
    python3 scripts/st09_i/s5_synthesize_sim_poses.py [--dry-run]

Prints a JSON evidence blob (targets, transforms, seed, solution, FK
errors, limit margins) and writes the YAML unless --dry-run.
"""
from __future__ import division

import argparse
import json
import math
import os
import sys

import numpy as np

REPO = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", ".."))
for entry in ("src/luggage_planning", "src/luggage_description"):
    path = os.path.join(REPO, entry)
    if path not in sys.path:
        sys.path.append(path)

from luggage_planning.offline_ik import load_urdf, refine_to_pose  # noqa: E402

SIM_SCENE = os.path.join(
    REPO, "src", "luggage_description", "config", "scene_tf.yaml.example")
DEPLOY_SCENE = os.path.join(
    REPO, "src", "luggage_description", "config", "scene_tf_deploy.yaml")
OUTPUT = os.path.join(
    REPO, "src", "luggage_description", "config", "robot_poses.sim_deploy.yaml")

#: Stock SIM pickup-observe joints (robot_poses.yaml.example).
SIM_PICKUP_OBSERVE = [1.8806, -1.7736, -1.0491, 4.4439, 1.7721, 1.8473]
CAMERA_LINK = "camera_depth_optical_frame"
TCP_LINK = "suction_contact_frame"
#: Overhead camera target above the DEPLOY pickup platform (+1, 0);
#: same height above the platform top as the sim calibration (1.9 m).
TARGET_XYZ = [1.0, 0.0, 1.9]
JOINTS = ["elfin_joint%d" % index for index in range(1, 7)]


def _wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _yaw_of(matrix):
    return math.atan2(matrix[1, 0], matrix[0, 0])


def _quat_xyzw(matrix):
    from scipy.spatial.transform import Rotation as Rot
    return Rot.from_matrix(matrix).as_quat()


def _wrap_into_limits(urdf, q):
    """Pick the 2*pi-equivalent branch inside the URDF limits per joint.

    Chooses the equivalent branch with the SMALLEST absolute value: the
    multi-turn URDF limits accept any branch, but the principal-range
    form stays inside the real Elf S20 physical limits, which keeps the
    pose clean for controller execution and for the later ServoJ 2*pi
    alignment (real joint limits) pre-certification.
    """
    wrapped = []
    for value, (lower, upper) in zip(q, urdf.joint_limits()):
        candidates = [float(value) + turns * 2.0 * math.pi
                      for turns in (-2, -1, 0, 1, 2)]
        candidates.sort(key=abs)
        best = next((candidate for candidate in candidates
                     if lower - 1e-9 <= candidate <= upper + 1e-9),
                    float(value))
        wrapped.append(best)
    return wrapped


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    sim_urdf = load_urdf(scene_tf_config=SIM_SCENE)
    dep_urdf = load_urdf(scene_tf_config=DEPLOY_SCENE)
    sim_tcp_path = sim_urdf.path_to(TCP_LINK)
    sim_cam_path = sim_urdf.path_to(CAMERA_LINK)
    dep_tcp_path = dep_urdf.path_to(TCP_LINK)
    dep_cam_path = dep_urdf.path_to(CAMERA_LINK)
    base_path = sim_urdf.path_to("elfin_base_link")

    # -- measure the fixed camera-to-TCP transform off the sim pose -----
    tcp_sim = sim_urdf.tcp(SIM_PICKUP_OBSERVE, sim_tcp_path)
    cam_sim = sim_urdf.tcp(SIM_PICKUP_OBSERVE, sim_cam_path)
    t_tcp_camera = np.linalg.inv(tcp_sim) @ cam_sim

    # -- world bearing change of the overhead camera target -------------
    base_sim = sim_urdf.tcp(SIM_PICKUP_OBSERVE, base_path)
    base_dep = dep_urdf.tcp(SIM_PICKUP_OBSERVE, base_path)
    v_sim = cam_sim[:2, 3] - base_sim[:2, 3]
    v_dep = np.asarray(TARGET_XYZ[:2], dtype=float) - base_dep[:2, 3]
    bearing_sim = math.atan2(float(v_sim[1]), float(v_sim[0]))
    bearing_dep = math.atan2(float(v_dep[1]), float(v_dep[0]))
    phi_world = _wrap(bearing_dep - bearing_sim)
    yaw_sim, yaw_dep = _yaw_of(base_sim), _yaw_of(base_dep)
    phi_base = _wrap(phi_world - (yaw_dep - yaw_sim))

    # -- camera target: same overhead attitude, yawed by phi_world -----
    from scipy.spatial.transform import Rotation as Rot
    rot_target = Rot.from_euler("z", phi_world) * Rot.from_matrix(
        cam_sim[:3, :3])
    camera_target = np.eye(4)
    camera_target[:3, :3] = rot_target.as_matrix()
    camera_target[:3, 3] = TARGET_XYZ
    tcp_target = camera_target @ np.linalg.inv(t_tcp_camera)

    # -- seed: the sim solution rotated to the new bearing --------------
    seed = list(SIM_PICKUP_OBSERVE)
    seed[0] = seed[0] + phi_base
    seed = _wrap_into_limits(dep_urdf, seed)

    solution, info = refine_to_pose(
        dep_urdf, dep_tcp_path, tcp_target[:3, 3],
        _quat_xyzw(tcp_target[:3, :3]), seed, pos_tol=1e-3, rot_tol=5e-3)

    # -- validate: limits, FK round-trip on BOTH links ------------------
    limits = dep_urdf.joint_limits()
    margins = []
    if solution is not None:
        for value, (lower, upper) in zip(solution, limits):
            margins.append(round(min(value - lower, upper - value), 4))
        tcp_fk = dep_urdf.tcp(solution, dep_tcp_path)
        cam_fk = dep_urdf.tcp(solution, dep_cam_path)
        cam_pos_err = float(np.linalg.norm(cam_fk[:3, 3] - TARGET_XYZ))
        optical_axis = cam_fk[:3, :3] @ np.array([0.0, 0.0, 1.0])
        downward = float(np.dot(optical_axis, np.array([0.0, 0.0, -1.0])))
        tcp_pos_err = float(np.linalg.norm(
            tcp_fk[:3, 3] - tcp_target[:3, 3]))
    evidence = {
        "sim_camera_position": [round(float(v), 4) for v in cam_sim[:3, 3]],
        "deploy_base_position": [round(float(v), 4) for v in base_dep[:3, 3]],
        "phi_world_rad": round(phi_world, 6),
        "phi_base_rad": round(phi_base, 6),
        "seed": [round(v, 4) for v in seed],
        "solution": None if solution is None else [round(v, 4)
                                                   for v in solution],
        "refine_info": info,
        "limit_margins_rad": None if solution is None else margins,
        "tcp_fk_pos_err_m": None if solution is None else round(tcp_pos_err, 6),
        "camera_fk_pos_err_m": None if solution is None else round(cam_pos_err, 6),
        "optical_axis_down_cos": None if solution is None else round(downward, 6),
        "t_tcp_camera_translation": [round(float(v), 5)
                                     for v in t_tcp_camera[:3, 3]],
    }
    print(json.dumps(evidence, indent=2, sort_keys=True))
    if solution is None:
        print("REFINE FAILED", file=sys.stderr)
        return 1
    if min(margins) < 0.0 or cam_pos_err > 2e-3 or downward < 0.99:
        print("VALIDATION FAILED (limits/FK/attitude)", file=sys.stderr)
        return 2

    if args.dry_run:
        return 0
    values = [round(float(v), 4) for v in solution]

    # -- portal observe pose: the post-release observation camera rides
    #    the sanctioned named-pose goto surface; the target is the old
    #    _exit_to_portal geometry (0.20 m outside the face, 0.35 m above
    #    the opening target), yaw kept in the pickup pose's wrist family.
    from luggage_description.scene_tf_config_utils import (
        container_opening_normal_in_world,
        container_opening_target_point_in_world,
        load_scene_tf_config,
    )
    dep_scene = load_scene_tf_config(DEPLOY_SCENE)
    opening = container_opening_target_point_in_world(dep_scene)
    outward = container_opening_normal_in_world(dep_scene)
    # Wrist-family continuity: the portal yaw keeps the DEPLOY pickup
    # solution's TCP yaw (nearest equivalent branch).
    tcp_dep = dep_urdf.tcp(solution, dep_tcp_path)
    yaw_source = Rot.from_matrix(tcp_dep[:3, :3]).as_euler("xyz")[2]
    portal_position = [
        float(opening[0]) + 0.20 * float(outward[0]),
        float(opening[1]) + 0.20 * float(outward[1]),
        max(float(opening[2]) + 0.35, 0.95),
    ]
    portal_orientation = Rot.from_euler(
        "xyz", [math.pi, 0.0, yaw_source]).as_quat()
    portal_solution, portal_info = refine_to_pose(
        dep_urdf, dep_tcp_path, portal_position, portal_orientation,
        solution, pos_tol=1e-3, rot_tol=5e-3)
    portal_values = None
    if portal_solution is not None:
        portal_values = [round(float(v), 4)
                         for v in _wrap_into_limits(dep_urdf, portal_solution)]
        portal_fk = dep_urdf.tcp(portal_values, dep_tcp_path)
        portal_err = float(np.linalg.norm(portal_fk[:3, 3] - portal_position))
        evidence["portal_solution"] = portal_values
        evidence["portal_refine_info"] = portal_info
        evidence["portal_fk_pos_err_m"] = round(portal_err, 6)
        evidence["portal_position"] = [round(v, 4) for v in portal_position]
        if portal_err > 2e-3:
            print("PORTAL VALIDATION FAILED (FK)", file=sys.stderr)
            return 3
    print(json.dumps({"portal": {
        key: evidence.get(key) for key in (
            "portal_solution", "portal_refine_info", "portal_fk_pos_err_m",
            "portal_position")}}, indent=2, sort_keys=True))

    document = """# Robot named poses for the MIRRORED DEPLOYMENT cell in SIMULATION
# (ST09-I S5). SIMULATION ONLY — synthesized start for the S5 Gazebo
# runner, derived by scripts/st09_i/s5_synthesize_sim_poses.py
# (offline_ik refine of the sim pickup_observe onto the deploy-scene
# overhead camera target at world (+1.0, 0.0, 1.9), converted through
# the fixed camera-to-TCP URDF transform). This is NOT a measured site
# pose: robot_poses.deploy.yaml stays all-valid:false and nothing in
# this file may move the real cell.
#
# portal_observe_sim: the post-release observation camera position (the
# old _exit_to_portal geometry: 0.20 m outside the opening face, 0.35 m
# above the opening target), reached through the sanctioned GoToRobotPose
# surface — the ad-hoc driver-side cartesian escape is deleted
# (SEGMENT_GENERATION_REMOVED). Same synthesized/offline-IK provenance.

poses:
  pickup_observe_sim:
    group: elfin_arm
    joints:
      - elfin_joint1
      - elfin_joint2
      - elfin_joint3
      - elfin_joint4
      - elfin_joint5
      - elfin_joint6
    values: [{values}]
    tolerance: 0.15
  portal_observe_sim:
    group: elfin_arm
    joints:
      - elfin_joint1
      - elfin_joint2
      - elfin_joint3
      - elfin_joint4
      - elfin_joint5
      - elfin_joint6
    values: [{portal_values}]
    tolerance: 0.15

defaults:
  observe_pose: pickup_observe_sim
  planning_time: 30.0
  max_velocity_scaling: 0.3
  max_acceleration_scaling: 0.3

meta:
  name: robot_poses_sim_deploy_v1
  version: 0.2.0
""".format(values=", ".join("%.4f" % v for v in values),
           portal_values=", ".join("%.4f" % v for v in portal_values))
    with open(OUTPUT, "w", encoding="utf-8") as stream:
        stream.write(document)
    print("wrote %s" % OUTPUT)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
