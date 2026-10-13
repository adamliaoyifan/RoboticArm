"""Deterministic offline FK/IK for the S20 arm (no MoveIt, no ROS calls).

Why this exists at runtime: KDLKinematicsPlugin seeds Newton with random
restarts, so /compute_ik is a lottery for large-reconfiguration targets --
the 2026-09-24 smokes measured the same seed/quat/scene flapping between
an instant solution and a full 1 s -31. scipy least_squares from an atlas
seed converges deterministically in milliseconds. The motion seed chain
refines atlas seeds to the exact target here, then submits the joints as
a MoveGroup joint goal (OMPL still plans and validates the path).

URDF expansion shells out to xacro once per process and is cached; the
math is pure numpy/scipy and mirrors scripts/replay_collision_attribution.py
(validated offline against recorded suction poses to 0.4 mm).
"""

from __future__ import division

import math
import os
import threading

import numpy as np

JOINTS = ["elfin_joint%d" % i for i in range(1, 7)]

_LOCK = threading.Lock()
_CACHE = {}


def _origin_of(element):
    import xml.etree.ElementTree as ET
    from scipy.spatial.transform import Rotation as Rot
    origin = element.find("origin") if element is not None else None
    xyz = ([float(v) for v in origin.get("xyz", "0 0 0").split()]
           if origin is not None else [0.0, 0.0, 0.0])
    rpy = ([float(v) for v in origin.get("rpy", "0 0 0").split()]
           if origin is not None else [0.0, 0.0, 0.0])
    return np.array(xyz, dtype=float), Rot.from_euler("xyz", rpy).as_matrix()


class Urdf(object):
    """Minimal URDF: joint origins/axes/limits + link path FK."""

    def __init__(self, xml_text):
        import xml.etree.ElementTree as ET
        self.root = ET.fromstring(xml_text)
        self.joints = {}
        self.children = {}
        for joint in self.root.findall("joint"):
            name = joint.get("name")
            xyz, rot = _origin_of(joint)
            axis_el = joint.find("axis")
            axis = ([float(v) for v in axis_el.get("xyz").split()]
                    if axis_el is not None else [1.0, 0.0, 0.0])
            limit = joint.find("limit")
            self.joints[name] = {
                "type": joint.get("type"),
                "parent": joint.find("parent").get("link"),
                "child": joint.find("child").get("link"),
                "xyz": xyz, "rot": rot,
                "axis": np.array(axis, dtype=float),
                "lower": (float(limit.get("lower", "-1e9"))
                          if limit is not None else -1e9),
                "upper": (float(limit.get("upper", "1e9"))
                          if limit is not None else 1e9),
            }
            self.children.setdefault(
                self.joints[name]["parent"], []).append(name)
        links = {link.get("name") for link in self.root.findall("link")}
        roots = links - {j["child"] for j in self.joints.values()}
        self.root_link = sorted(roots)[0]

    def path_to(self, target_link):
        path = []

        def walk(link):
            if link == target_link:
                return True
            for jname in self.children.get(link, []):
                if walk(self.joints[jname]["child"]):
                    path.append(jname)
                    return True
            return False

        walk(self.root_link)
        return list(reversed(path))

    def joint_limits(self):
        return [(self.joints[n]["lower"], self.joints[n]["upper"])
                for n in JOINTS]

    def tcp(self, q, path):
        """FK along ``path`` (from path_to) for joint values ``q``."""
        from scipy.spatial.transform import Rotation as Rot
        values = dict(zip(JOINTS, [float(v) for v in q]))
        tf = np.eye(4)
        for jname in path:
            joint = self.joints[jname]
            local = np.eye(4)
            local[:3, :3] = joint["rot"]
            local[:3, 3] = joint["xyz"]
            if joint["type"] == "revolute":
                angle = values.get(jname, 0.0)
                rot = Rot.from_rotvec(
                    joint["axis"] / np.linalg.norm(joint["axis"])
                    * angle).as_matrix()
                local[:3, :3] = local[:3, :3] @ rot
            tf = tf @ local
        return tf


def default_urdf_xacro():
    """The with-camera S20 xacro: luggage_description src tree, then share."""
    repo = os.path.normpath(os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", ".."))
    src = os.path.join(
        repo, "src", "luggage_description", "urdf",
        "elfin_s20_with_camera.urdf.xacro")
    if os.path.isfile(src):
        return src
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(
        get_package_share_directory("luggage_description"), "urdf",
        "elfin_s20_with_camera.urdf.xacro")


def load_urdf(xacro_path=None, scene_tf_config=None):
    """Expand and parse the arm URDF (cached per process).

    ``scene_tf_config`` keeps the world_base patch consistent with the
    live scene; None resolves the installed default.
    """
    key = (str(xacro_path or ""), str(scene_tf_config or ""))
    with _LOCK:
        if key in _CACHE:
            return _CACHE[key]
        from luggage_description.xacro_robot_with_scene_base import (
            expand_and_patch)
        xml, _ = expand_and_patch(
            xacro_path or default_urdf_xacro(), scene_tf_config or None,
            ["fixed_world:=true"])
        urdf = Urdf(xml)
        _CACHE[key] = urdf
        return urdf


def quat_multiply(a, b):
    """(x, y, z, w) Hamilton product."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz])


def refine_to_pose(urdf, tcp_path, target_pos, target_quat_xyzw, seed,
                   pos_tol=5e-4, rot_tol=5e-3):
    """Deterministic Newton refine of ``seed`` onto the exact target pose.

    Returns (joints | None, info dict). Respects URDF joint limits.
    """
    from scipy.optimize import least_squares
    from scipy.spatial.transform import Rotation as Rot
    target_pos = np.asarray(target_pos, dtype=float)
    target_rot = Rot.from_quat(np.asarray(target_quat_xyzw, dtype=float))
    lower = np.array([lo for lo, _ in urdf.joint_limits()])
    upper = np.array([hi for _, hi in urdf.joint_limits()])

    def residual(q):
        tf = urdf.tcp(q, tcp_path)
        pos_err = tf[:3, 3] - target_pos
        rot_err = (Rot.from_matrix(tf[:3, :3]).inv() * target_rot).as_rotvec()
        return np.concatenate([pos_err, rot_err])

    sol = least_squares(
        residual, np.clip(np.asarray(seed, dtype=float), lower, upper),
        bounds=(lower, upper), xtol=1e-10, ftol=1e-10, max_nfev=200)
    err = residual(sol.x)
    pos_err = float(np.linalg.norm(err[:3]))
    rot_err = float(np.linalg.norm(err[3:]))
    ok = pos_err <= pos_tol and rot_err <= rot_tol
    return (list(sol.x) if ok else None), {
        "pos_err_m": round(pos_err, 6), "rot_err_rad": round(rot_err, 6),
        "ok": bool(ok)}
