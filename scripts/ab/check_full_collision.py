#!/usr/bin/env python3
"""Full-geometry retreat collision check (user item 1, 2026-10-19).

The 0.73mm TCP-point clearance proves nothing about the arm: this check
FKs EVERY link that carries collision geometry (arm links + tool), maps
each link's collision-mesh point set into the world, and queries the
distance to each arm's ACTUAL container collision asset (A0 physics STL,
A' trimesh, B boxes) — A0 checked independently because its inward
debris means "wider outer boundary" is not a superset guarantee.

Joint samples are OVERSAMPLED with linear interpolation (midpoints to
<= 5 ms command spacing, matching the runner's interpolation), so the
checked path covers what the controller will actually track between
samples. Sampling evidence, not a continuous-geometry proof: the report
states the sampling density explicitly.
"""
import argparse
import json
import math
import os
import sys

import numpy as np
import open3d as o3d

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from design_baseline import load_baseline  # noqa: E402

_ROOT = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", ".."))
_SCENE_TF = os.path.join(_ROOT, "src", "luggage_description", "config",
                        "scene_tf_deploy_update.yaml")

#: Max spacing between checked joint configurations (matches the runner's
#: command interpolation step).
STEP_S = 0.005
#: Points sampled per link collision mesh (sampling evidence density).
POINTS_PER_LINK = 400
#: Arms must clear by this margin (m) to pass; 0 = touching.
PASS_MARGIN_M = 0.0


def load_link_point_sets(urdf_root):
    """{(link_name): (N,3) points in the link frame} for links with
    collision geometry, from the URDF's collision mesh/box/cylinder."""
    import xml.etree.ElementTree as ET
    sets = {}
    install = os.path.join(_ROOT, "install")
    for link in urdf_root.findall("link"):
        name = link.get("name")
        col = link.find("collision")
        if col is None:
            continue
        geometry = col.find("geometry")
        if geometry is None:
            continue
        pts = None
        mesh = geometry.find("mesh")
        box = geometry.find("box")
        cylinder = geometry.find("cylinder")
        origin = col.find("origin")
        xyz = np.array([float(v) for v in (
            origin.get("xyz", "0 0 0").split())]) if origin is not None \
            else np.zeros(3)
        if mesh is not None:
            uri = mesh.get("filename", "")
            path = _resolve_mesh(uri, install)
            if path and os.path.isfile(path):
                m = o3d.io.read_triangle_mesh(path)
                if len(m.vertices):
                    cloud = m.sample_points_uniformly(
                        number_of_points=POINTS_PER_LINK)
                    pts = np.asarray(cloud.points)
        elif box is not None:
            size = [float(v) for v in box.get("size").split()]
            rng = np.random.default_rng(7)
            pts = (rng.random((POINTS_PER_LINK, 3)) - 0.5) * np.array(size)
        elif cylinder is not None:
            radius = float(cylinder.get("radius"))
            length = float(cylinder.get("length"))
            rng = np.random.default_rng(7)
            pts = np.zeros((POINTS_PER_LINK, 3))
            pts[:, 2] = (rng.random(POINTS_PER_LINK) - 0.5) * length
            ang = rng.random(POINTS_PER_LINK) * 2 * math.pi
            rad = np.sqrt(rng.random(POINTS_PER_LINK)) * radius
            pts[:, 0] = np.cos(ang) * rad
            pts[:, 1] = np.sin(ang) * rad
        if pts is not None and len(pts):
            sets[name] = pts + xyz
    return sets


def _resolve_mesh(uri, install):
    uri = uri.replace("package://", "").replace("model://", "")
    parts = uri.split("/")
    if parts and parts[0] in ("elfin_description", "luggage_gazebo",
                              "luggage_description", "elfin_moveit_config"):
        for prefix in (os.path.join(install, parts[0], "share"),):
            path = os.path.join(prefix, *parts[1:])
            if os.path.isfile(path):
                return path
        path = os.path.join(_ROOT, "src", *parts)
        if os.path.isfile(path):
            return path
    return None


def fk_all_links(urdf, q, wanted):
    """World-frame 4x4 transforms for every link in `wanted`."""
    values = dict(zip(
        [j for j in ("elfin_joint1", "elfin_joint2", "elfin_joint3",
                     "elfin_joint4", "elfin_joint5", "elfin_joint6")],
        [float(v) for v in q]))
    from scipy.spatial.transform import Rotation as Rot
    out = {}

    def walk(link, tf):
        out[link] = tf
        for jname in urdf.children.get(link, []):
            joint = urdf.joints[jname]
            local = np.eye(4)
            local[:3, :3] = joint["rot"]
            local[:3, 3] = joint["xyz"]
            if joint["type"] == "revolute" and jname in values:
                axis = joint["axis"] / (np.linalg.norm(joint["axis"])
                                        + 1e-12)
                ang = float(values[jname])
                rot = Rot.from_rotvec(axis * ang).as_matrix()
                step = np.eye(4)
                step[:3, :3] = rot
                local = local.dot(step)
            walk(joint["child"], tf.dot(local))

    walk(urdf.root_link, np.eye(4))
    return out


def container_scenes(baseline, a0_stl, aprime_stl, bprim_manifest):
    scenes = {}
    scenes["A0"] = _mesh_scene(a0_stl)
    scenes["APRIME"] = _mesh_scene(aprime_stl)
    boxes = json.load(open(bprim_manifest))["bprim"]["boxes"]
    scenes["B"] = ("boxes", boxes)
    return scenes


def _mesh_scene(path):
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(
        o3d.io.read_triangle_mesh(path)))
    return scene


def dist_to_scene(entry, pts):
    if isinstance(entry, tuple) and entry[0] == "boxes":
        best = np.full(len(pts), np.inf)
        for bxx in entry[1]:
            c = np.array(bxx["center"])
            s = np.array(bxx["size"]) / 2
            roll = bxx.get("roll") or 0.0
            local = pts - c
            if roll:
                cr, sr = math.cos(roll), math.sin(roll)
                rot = np.array([[1, 0, 0], [0, cr, sr], [0, -sr, cr]])
                local = local.dot(rot.T)
            clamped = np.clip(local, -s, s)
            best = np.minimum(best, np.linalg.norm(local - clamped, axis=1))
        return best
    pts = np.asarray(pts, dtype=np.float32).reshape(-1, 3)
    return entry.compute_distance(o3d.core.Tensor(pts)).numpy()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--tool-stream", required=True)
    parser.add_argument("--release-t-sim", type=float, default=126.889)
    parser.add_argument("--window-s", type=float, default=45.0)
    parser.add_argument("--a0-stl", required=True)
    parser.add_argument("--aprime-stl", required=True)
    parser.add_argument("--bprim-manifest", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    from luggage_planning.offline_ik import load_urdf, JOINTS
    urdf = load_urdf(scene_tf_config=_SCENE_TF)
    link_pts = load_link_point_sets(urdf.root)
    if not link_pts:
        raise SystemExit("no link collision geometry found in URDF")

    # Recorded samples in the retreat window.
    raw = []
    for line in open(args.tool_stream):
        d = json.loads(line)
        if d["kind"] != "joint_states":
            continue
        t = d["t_sim_ns"] / 1e9
        if not (args.release_t_sim - 1.0
                <= t <= args.release_t_sim + args.window_s):
            continue
        names = d.get("name") or []
        if names and set(JOINTS) <= set(names):
            q = [d["position"][names.index(j)] for j in JOINTS]
        else:
            q = [float(v) for v in d["position"][:6]]
        raw.append((t, q))
    raw.sort()
    if len(raw) < 2:
        raise SystemExit("insufficient joint samples in window")

    # Oversample with LINEAR interpolation at STEP_S (the runner's
    # command parameterization): checks what lies between samples.
    dense = []
    for i in range(len(raw) - 1):
        t0, q0 = raw[i]
        t1, q1 = raw[i + 1]
        dense.append((t0, q0))
        span = t1 - t0
        n = max(int(span / STEP_S), 1)
        for k in range(1, n):
            a = k / n
            dense.append((t0 + span * a,
                          [(1 - a) * a0 + a * a1
                           for a0, a1 in zip(q0, q1)]))
    dense.append(raw[-1])

    # Container placement (world <- container_link).
    b = load_baseline(_SCENE_TF)
    t_world = np.array([-1.5, 0.6, 0.0])
    yaw = math.pi
    R_wc = np.array([[math.cos(yaw), -math.sin(yaw), 0.0],
                     [math.sin(yaw), math.cos(yaw), 0.0],
                     [0.0, 0.0, 1.0]])
    scenes = container_scenes(b, args.a0_stl, args.aprime_stl,
                              args.bprim_manifest)

    # Transform link points into CONTAINER frame once per config.
    mins = {arm: float("inf") for arm in scenes}
    worst_at = {arm: None for arm in scenes}
    for t, q in dense:
        transforms = fk_all_links(urdf, q, link_pts)
        pts_c = []
        for link, pts in link_pts.items():
            tf = transforms.get(link)
            if tf is None:
                continue
            world = pts.dot(tf[:3, :3].T) + tf[:3, 3]
            pts_c.append((world - t_world).dot(R_wc))
        pts_c = np.concatenate(pts_c)
        for arm, entry in scenes.items():
            d = float(dist_to_scene(entry, pts_c).min())
            if d < mins[arm]:
                mins[arm] = d
                worst_at[arm] = t
    report = {
        "checked_configs": len(dense),
        "recorded_samples": len(raw),
        "interp_step_s": STEP_S,
        "points_per_link": POINTS_PER_LINK,
        "links_with_geometry": sorted(link_pts),
        "min_distance_m": {k: round(v, 5) for k, v in mins.items()},
        "worst_t_sim": worst_at,
        "pass_margin_m": PASS_MARGIN_M,
        "pass": all(v > PASS_MARGIN_M for v in mins.values()),
        "evidence_class": "sampling (uniform points per link mesh, "
                          "5 ms interpolated configs); not a continuous-"
                          "geometry proof",
    }
    with open(args.out, "w") as handle:
        json.dump(report, handle, indent=2)
    print(json.dumps(report, indent=1))
    raise SystemExit(0 if report["pass"] else 1)


if __name__ == "__main__":
    main()
