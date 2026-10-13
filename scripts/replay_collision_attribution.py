#!/usr/bin/env python3
"""Offline collision attribution for a failed motion segment.

Rebuilds the planning scene from a replay bundle (planning_scene.json +
last_boundary.json), solves IK offline for the recorded target (both
tool-down yaws, several seeds), and reports for every IK branch which
robot part / attached payload comes closest to which named container
face or the pedestal. MoveIt's collision-aware compute_ik only says
NO_IK_SOLUTION and never names the rejecting object; this tool answers
"which face blocked the goal" from the frozen evidence, with no sim.

Usage (repo root, humble sourced for the xacro CLI):
  scripts/replay_collision_attribution.py \
      --replay docs/status/evidence/motion_occ/<run>/dumps/trial_XX_*/replay \
      [--seed-joints 4.2534,-2.593,-0.9438,1.9683,4.7071,1.0786] \
      [--contact-mm 3] [--samples 400] [--figure]
"""

from __future__ import division

import argparse
import json
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ("src", "src/luggage_description", "src/luggage_gazebo"):
    sys.path.insert(0, os.path.join(REPO, _p))

import trimesh  # noqa: E402  (STL loading only; no rtree/fcl needed)
from scipy.optimize import least_squares  # noqa: E402
from scipy.spatial.transform import Rotation as Rot  # noqa: E402
import xml.etree.ElementTree as ET  # noqa: E402

# motion_executor.TOOL_DOWN_QUAT: 180 deg about X, suction normal -Z.
TOOL_DOWN_QUAT = (1.0, 0.0, 0.0, 0.0)  # x y z w
JOINT_ORDER = ["elfin_joint%d" % i for i in range(1, 7)]
COLLISION_LINKS = [
    "elfin_base",
    "elfin_link1", "elfin_link2", "elfin_link3",
    "elfin_link4", "elfin_link5", "elfin_link6",
    "elfin_end_link", "eef_mount_adapter", "suction_panel",
    "camera_link",
]
FACE_NAMES = {
    ("x", -1): "front(opening,robot side)", ("x", +1): "back_wall",
    ("y", -1): "side_wall_-y", ("y", +1): "side_wall_+y",
}


# ----------------------------------------------------------------------
# URDF

def _origin_of(element):
    origin = element.find("origin") if element is not None else None
    xyz = ([float(v) for v in origin.get("xyz", "0 0 0").split()]
           if origin is not None else [0.0, 0.0, 0.0])
    rpy = ([float(v) for v in origin.get("rpy", "0 0 0").split()]
           if origin is not None else [0.0, 0.0, 0.0])
    return np.array(xyz, dtype=float), Rot.from_euler("xyz", rpy).as_matrix()


class Urdf(object):
    def __init__(self, xml_text):
        self.root = ET.fromstring(xml_text)
        self.joints = {}
        self.children = {}
        self.links = set()
        for link in self.root.findall("link"):
            self.links.add(link.get("name"))
            self.link_els = getattr(self, "link_els", {})
            self.link_els[link.get("name")] = link
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
                "lower": float(limit.get("lower", "-1e9")) if limit is not None else -1e9,
                "upper": float(limit.get("upper", "1e9")) if limit is not None else 1e9,
            }
            self.children.setdefault(
                self.joints[name]["parent"], []).append(name)
        roots = self.links - {j["child"] for j in self.joints.values()}
        self.root_link = sorted(roots)[0]

    def path_to(self, target_link):
        """Actuated+fixed joint names from the root to ``target_link``."""
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
                for n in JOINT_ORDER]

    def link_transforms(self, q):
        values = dict(zip(JOINT_ORDER, [float(v) for v in q]))
        out = {}

        def compose(parent_tf, link):
            out[link] = parent_tf
            for jname in self.children.get(link, []):
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
                compose(parent_tf @ local, joint["child"])

        compose(np.eye(4), self.root_link)
        return out

    def tcp(self, q, path):
        values = dict(zip(JOINT_ORDER, [float(v) for v in q]))
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


def _resolve_package_path(uri):
    rest = uri.replace("package://", "")
    name, _, tail = rest.partition("/")
    for base in (os.path.join(REPO, "src", name),
                 os.path.join(REPO, "install", name, "share", name),
                 os.path.join(REPO, "robot_assets", name)):
        candidate = os.path.join(base, tail)
        if os.path.exists(candidate):
            return candidate
    raise FileNotFoundError(uri)


def link_collision_points(urdf, link_name, max_points):
    """Sampled surface points of a link's collision geometry, link frame."""
    link = urdf.link_els.get(link_name)
    if link is None:
        return None
    blocks = []
    for collision in link.findall("collision"):
        xyz, rot = _origin_of(collision)
        geom = collision.find("geometry")
        mesh_el, box_el, cyl_el = (geom.find("mesh"), geom.find("box"),
                                   geom.find("cylinder"))
        if mesh_el is not None:
            m = trimesh.load(_resolve_package_path(mesh_el.get("filename")))
            pts = np.asarray(m.vertices, dtype=float)
            if len(pts) > max_points:
                pts = pts[np.linspace(0, len(pts) - 1,
                                      max_points).astype(int)]
        elif box_el is not None:
            size = np.array([float(v) for v in box_el.get("size").split()])
            pts = _box_surface(size)
        elif cyl_el is not None:
            r = float(cyl_el.get("radius"))
            h = float(cyl_el.get("length"))
            ang = np.linspace(0, 2 * np.pi, 20, endpoint=False)
            zs = np.linspace(-h / 2, h / 2, 5)
            pts = np.array([[r * np.cos(a), r * np.sin(a), z]
                            for z in zs for a in ang])
        else:
            continue
        pts = pts @ rot.T + xyz
        blocks.append(pts)
    if not blocks:
        return None
    return np.vstack(blocks)


def _box_surface(size):
    grid = np.linspace(-0.5, 0.5, 7)
    pts = []
    for axis in range(3):
        for sign in (-0.5, 0.5):
            for a in grid:
                for b in grid:
                    p = np.zeros(3)
                    p[axis] = sign
                    p[(axis + 1) % 3] = a
                    p[(axis + 2) % 3] = b
                    pts.append(p * size)
    return np.unique(np.round(np.array(pts), 6), axis=0)


# ----------------------------------------------------------------------
# Proximity

def points_to_triangles(p, a, b, c):
    """Ericson closest point on every triangle, one query point. (T,) dists."""
    ab, ac = b - a, c - a
    d1 = np.einsum("ij,ij->i", ab, p - a)
    d2 = np.einsum("ij,ij->i", ac, p - a)
    d3 = np.einsum("ij,ij->i", ab, p - b)
    d4 = np.einsum("ij,ij->i", ac, p - b)
    d5 = np.einsum("ij,ij->i", ab, p - c)
    d6 = np.einsum("ij,ij->i", ac, p - c)
    va = d3 * d6 - d5 * d4
    vb = d5 * d2 - d1 * d6
    vc = d1 * d4 - d3 * d2
    denom = np.clip(va + vb + vc, 1e-12, None)
    v, w = vc / denom, vb / denom
    q = a + ab * v[:, None] + ac * w[:, None]
    m1 = (d1 <= 0) & (d2 <= 0)
    q[m1] = a[m1]
    m2 = (d3 >= 0) & (d4 <= d3)
    q[m2] = b[m2]
    m3 = (d6 >= 0) & (d5 <= d6)
    q[m3] = c[m3]
    m4 = (vc <= 0) & (d1 >= 0) & (d3 <= 0)
    if m4.any():
        t = np.zeros(len(d1))
        den = d1 - d3
        t[m4] = np.where(den[m4] != 0, d1[m4] / den[m4], 0.0)
        q[m4] = (a + t[:, None] * ab)[m4]
    m5 = (vb <= 0) & (d2 >= 0) & (d6 <= 0)
    if m5.any():
        w2 = np.zeros(len(d2))
        den = d2 - d6
        w2[m5] = np.where(den[m5] != 0, d2[m5] / den[m5], 0.0)
        q[m5] = (a + w2[:, None] * ac)[m5]
    m6 = (va <= 0) & ((d4 - d3) >= 0) & ((d5 - d6) >= 0)
    if m6.any():
        t2 = np.zeros(len(d1))
        den = (d5 - d6) + (d4 - d3)
        t2[m6] = np.where(den[m6] != 0, (d5 - d6)[m6] / den[m6], 0.0)
        q[m6] = (b + t2[:, None] * (c - b))[m6]
    return np.linalg.norm(q - p, axis=1)


class MeshProximity(object):
    """Point-to-mesh minimum distance + face label (no rtree/fcl)."""

    def __init__(self, mesh, translate=None, z_floor_below=1.0):
        if translate is not None:
            mesh = mesh.copy()
            mesh.apply_translation(translate)
        v = np.asarray(mesh.vertices, dtype=float)
        f = np.asarray(mesh.faces, dtype=int)
        self.mesh = mesh
        self.a, self.b, self.c = v[f[:, 0]], v[f[:, 1]], v[f[:, 2]]
        self.normals = np.asarray(mesh.face_normals, dtype=float)
        centroids = (self.a + self.b + self.c) / 3.0
        labels = []
        for i, n in enumerate(self.normals):
            dom = int(np.abs(n).argmax())
            axis, sign = ("x", "y", "z")[dom], 1 if n[dom] >= 0 else -1
            if axis == "z" and sign > 0:
                name = ("floor_top" if centroids[i, 2] < z_floor_below
                        else "lid/rim_top")
            elif axis == "z":
                name = "underside"
            else:
                name = FACE_NAMES.get((axis, sign), "%s%s" % (axis, sign))
            labels.append(name)
        self.labels = labels

    def closest(self, points):
        """(min_dist, tri_idx, face_label) per query point."""
        dists = np.empty(len(points))
        tris = np.empty(len(points), dtype=int)
        for i, p in enumerate(points):
            d = points_to_triangles(p, self.a, self.b, self.c)
            tris[i] = int(d.argmin())
            dists[i] = d[tris[i]]
        return dists, tris


class OrientedBox(object):
    def __init__(self, center, quat_xyzw, size):
        self.center = np.array(center, dtype=float)
        self.rot = Rot.from_quat(quat_xyzw).as_matrix()
        self.half = np.array(size, dtype=float) / 2.0

    def signed_distance(self, points):
        local = (points - self.center) @ self.rot
        outside = np.maximum(np.abs(local) - self.half, 0.0)
        dist = np.linalg.norm(outside, axis=1)
        inside = (np.abs(local) <= self.half).all(axis=1)
        if inside.any():
            dist[inside] = -np.min(self.half - np.abs(local[inside]), axis=1)
        return dist


# ----------------------------------------------------------------------
# IK

def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz])


def solve_ik(urdf, tcp_path, target_pos, target_quat, seed, limits):
    lower = np.array([lo for lo, _ in limits])
    upper = np.array([hi for _, hi in limits])
    target_rot = Rot.from_quat(target_quat)

    def residual(q):
        tf = urdf.tcp(q, tcp_path)
        pos_err = tf[:3, 3] - target_pos
        rot_err = (Rot.from_matrix(tf[:3, :3]).inv() * target_rot).as_rotvec()
        return np.concatenate([pos_err, rot_err])

    sol = least_squares(residual, np.clip(seed, lower, upper),
                        bounds=(lower, upper), xtol=1e-10, ftol=1e-10,
                        max_nfev=300)
    err = residual(sol.x)
    return sol.x, float(np.linalg.norm(err[:3])), float(np.linalg.norm(err[3:]))


# ----------------------------------------------------------------------

def payload_points(tcp, wdh):
    """Attached box surface points: top center at TCP, +z_tool into box."""
    w, d, h = wdh
    local = _box_surface(np.array([w, d, h]))
    tf = tcp.copy()
    tf[:3, 3] = tcp[:3, 3] + tcp[:3, 2] * (h / 2.0)
    return local @ tf[:3, :3].T + tf[:3, 3]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--replay", required=True)
    ap.add_argument("--seed-joints", default="",
                    help="csv extra IK seed, e.g. the 1838 endpoint")
    ap.add_argument("--contact-mm", type=float, default=3.0)
    ap.add_argument("--samples", type=int, default=400)
    ap.add_argument("--figure", action="store_true",
                    help="render collision_attribution.html/.png")
    ap.add_argument("--figure-state", default=None,
                    help="state label to draw (default: first IK solution)")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    boundary = json.load(open(os.path.join(args.replay, "last_boundary.json")))
    scene = json.load(open(os.path.join(args.replay, "planning_scene.json")))
    target = np.array(boundary["target"], dtype=float)
    payload = boundary.get("payload_wdh") or [0.5, 0.36, 0.25]
    start = np.array(boundary["start_joints"], dtype=float)

    from luggage_description.xacro_robot_with_scene_base import expand_and_patch
    xacro_path = os.path.join(REPO, "src", "luggage_description", "urdf",
                              "elfin_s20_with_camera.urdf.xacro")
    xml_text, _ = expand_and_patch(xacro_path, None, ["fixed_world:=true"])
    urdf = Urdf(xml_text)
    tcp_link = "suction_contact_frame"
    tcp_path = urdf.path_to(tcp_link)
    limits = urdf.joint_limits()

    tf0 = urdf.tcp(start, tcp_path)
    print("FK(recorded start) suction_contact = %s"
          % np.round(tf0[:3, 3], 4).tolist())
    print("tool z (suction normal)             = %s"
          % np.round(tf0[:3, 2], 3).tolist())

    container = pedestal = None
    for obj in scene["world_objects"]:
        if obj["type"] == "mesh":
            path = _resolve_package_path(
                "package://luggage_gazebo/models/%s/meshes/"
                "container_collision.stl" % obj["id"])
            container = MeshProximity(
                trimesh.load(path), translate=(1.5, 0.0, 0.0))
        elif obj["type"] == "box":
            pedestal = OrientedBox(obj.get("xyz_object", obj.get("xyz")),
                                   obj["quat"], obj["size"])

    geometry = []
    for name in COLLISION_LINKS:
        pts = link_collision_points(urdf, name, args.samples)
        if pts is not None:
            geometry.append((name, pts))

    q_down = np.array(TOOL_DOWN_QUAT)
    yaw_pi = quat_multiply(tuple(q_down), (0.0, 0.0, 1.0, 0.0))
    seeds = [start]
    extra = None
    if args.seed_joints:
        extra = np.array([float(v) for v in args.seed_joints.split(",")])
        seeds.append(extra)
    rng = np.random.default_rng(7)
    for _ in range(10):
        lo = np.array([l for l, _ in limits])
        hi = np.array([h for _, h in limits])
        seeds.append(np.clip(start + rng.uniform(-1.2, 1.2, 6), lo, hi))

    solutions = []
    for label, quat in (("yaw0", q_down), ("yaw+pi", yaw_pi)):
        for seed in seeds:
            q, perr, rerr = solve_ik(urdf, tcp_path, target, quat,
                                     seed, limits)
            if perr < 5e-4 and rerr < 5e-3:
                if not any(np.max(np.abs(q - s["q"])) < 0.05
                           for s in solutions):
                    solutions.append({"yaw": label, "q": q,
                                      "pos_err_m": perr,
                                      "rot_err_rad": rerr})
    print("offline IK: %d distinct solution(s)" % len(solutions))
    for s in solutions:
        print("   %-7s q=%s" % (s["yaw"], np.round(s["q"], 3).tolist()))

    states = [{"label": "recorded_start(control)", "q": start}]
    for i, s in enumerate(solutions):
        states.append({"label": "IK_%s #%d" % (s["yaw"], i + 1),
                       "q": s["q"]})
    if extra is not None:
        states.append({"label": "1838_endpoint(control)", "q": extra})

    threshold = args.contact_mm / 1000.0
    results = []
    for state in states:
        tfs = urdf.link_transforms(state["q"])
        tcp = urdf.tcp(state["q"], tcp_path)
        parts = [("pickup_box(attached)", payload_points(tcp, payload))]
        parts += [(name, pts @ tfs[name][:3, :3].T + tfs[name][:3, 3])
                  for name, pts in geometry]
        rows = []
        for name, pts in parts:
            row = {"part": name}
            if container is not None:
                dists, tris = container.closest(pts)
                k = int(dists.argmin())
                row["container_mm"] = round(float(dists[k]) * 1000, 1)
                row["container_face"] = (container.labels[tris[k]]
                                         if dists[k] < 0.03 else "")
                row["contact"] = bool(dists[k] < threshold)
            if pedestal is not None:
                pd = pedestal.signed_distance(pts)
                row["pedestal_mm"] = round(float(pd.min()) * 1000, 1)
                row["pedestal_contact"] = bool(pd.min() < threshold)
                row.setdefault("contact", False)
                row["contact"] = row["contact"] or row["pedestal_contact"]
            rows.append(row)
        results.append({"label": state["label"], "q": state["q"].tolist(),
                        "rows": rows})

        clean = [r for r in rows if not r["contact"]]
        blocked = [r for r in rows if r["contact"]]
        worst = min((r["container_mm"] for r in rows
                     if "container_mm" in r), default=None)
        print("\n== %s ==  container worst approach %s mm, %d part(s) in contact"
              % (state["label"],
                 "-" if worst is None else round(worst, 1), len(blocked)))
        for r in sorted(blocked, key=lambda r: min(
                r.get("container_mm", 9e9), r.get("pedestal_mm", 9e9))):
            print("   CONTACT %-24s container %8s mm %-28s pedestal %8s mm"
                  % (r["part"], r.get("container_mm", "-"),
                     r.get("container_face", ""), r.get("pedestal_mm", "-")))
        for r in sorted(clean, key=lambda r: r.get("container_mm", 9e9))[:3]:
            print("   nearest %-23s container %8s mm %-28s pedestal %8s mm"
                  % (r["part"], r.get("container_mm", "-"),
                     r.get("container_face", ""), r.get("pedestal_mm", "-")))

    out_dir = args.out or args.replay
    with open(os.path.join(out_dir, "collision_attribution.json"), "w") as f:
        json.dump({"target": target.tolist(), "payload_wdh": payload,
                   "contact_threshold_mm": args.contact_mm,
                   "states": results}, f, indent=1)
    print("\nwrote %s" % os.path.join(out_dir, "collision_attribution.json"))

    if args.figure:
        fig_state = next((s for s in states
                          if s["label"] == args.figure_state), None)
        if fig_state is None:
            fig_state = next((s for s in states if s["label"].startswith("IK_")),
                             states[-1])
        render_figure(urdf, tcp_path, geometry, container, pedestal,
                      fig_state, payload, target, out_dir)


# ----------------------------------------------------------------------
# Figure

FACE_COLORS = {
    "floor_top": "#8a5a44", "lid/rim_top": "#c2a25a",
    "underside": "#555555", "front(opening,robot side)": "#3d6fb4",
    "back_wall": "#b4653d", "side_wall_-y": "#7a52a8", "side_wall_+y": "#7a52a8",
}
LINK_COLORS = ["#d62728", "#ff7f0e", "#2ca02c", "#1f77b4", "#9467bd",
               "#8c564b", "#e377c2", "#17becf", "#bcbd22", "#7f7f7f"]


def _tri_indices(mesh):
    f = np.asarray(mesh.faces, dtype=int)
    return f[:, 0], f[:, 1], f[:, 2]


def render_figure(urdf, tcp_path, geometry, container, pedestal,
                  state, payload, target, out_dir):
    """Interactive HTML + static PNG of one state against the scene."""
    import plotly.graph_objects as go
    q = np.array(state["q"], dtype=float)
    tfs = urdf.link_transforms(q)
    tcp = urdf.tcp(q, tcp_path)
    label = state["label"]

    traces = []
    if container is not None:
        m = container.mesh
        v = np.asarray(m.vertices, dtype=float)
        i, j, k = _tri_indices(m)
        colors = [FACE_COLORS.get(lab, "#999999") for lab in container.labels]
        traces.append(go.Mesh3d(
            x=v[:, 0], y=v[:, 1], z=v[:, 2], i=i, j=j, k=k,
            facecolor=colors, opacity=0.25, flatshading=True,
            name="container (faces colored)", showscale=False))
    if pedestal is not None:
        corners = _box_corners(pedestal)
        traces.append(go.Mesh3d(
            x=corners[:, 0], y=corners[:, 1], z=corners[:, 2],
            i=_QUAD_I, j=_QUAD_J, k=_QUAD_K, color="#666666",
            opacity=0.4, name="robot_pedestal"))
    for idx, name in enumerate(COLLISION_LINKS):
        link = urdf.link_els.get(name)
        if link is None:
            continue
        for collision in link.findall("collision"):
            geom = collision.find("geometry")
            mesh_el = geom.find("mesh")
            if mesh_el is None:
                continue
            m = trimesh.load(_resolve_package_path(mesh_el.get("filename")))
            v = np.asarray(m.vertices, dtype=float)
            v = v @ tfs[name][:3, :3].T + tfs[name][:3, 3]
            i, j, k = _tri_indices(m)
            traces.append(go.Mesh3d(
                x=v[:, 0], y=v[:, 1], z=v[:, 2], i=i, j=j, k=k,
                color=LINK_COLORS[idx % len(LINK_COLORS)], opacity=0.9,
                flatshading=True, name=name, showlegend=False))
    box = _box_corners_from_tf(np.array(payload) / 2.0,
                               tcp, along_z=payload[2] / 2.0)
    traces.append(go.Mesh3d(
        x=box[:, 0], y=box[:, 1], z=box[:, 2],
        i=_QUAD_I, j=_QUAD_J, k=_QUAD_K, color="#00cc96",
        opacity=0.45, name="pickup_box(attached)"))
    traces.append(go.Scatter3d(
        x=[target[0]], y=[target[1]], z=[target[2]], mode="markers+text",
        marker={"size": 6, "color": "black", "symbol": "diamond"},
        text=["target %.3f %.3f %.3f" % tuple(target)],
        textposition="top center", name="IK target"))

    fig = go.Figure(data=traces)
    fig.update_layout(
        title="Collision attribution -- %s (joints %s)"
              % (label, np.round(q, 2).tolist()),
        scene={"aspectmode": "data"})
    html_path = os.path.join(out_dir, "collision_attribution.html")
    # embed plotly.js so the archived evidence renders offline
    fig.write_html(html_path, include_plotlyjs=True)
    print("wrote %s" % html_path)


_QUAD_I = np.array([0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4])
_QUAD_J = np.array([1, 2, 3, 4, 5, 6, 6, 7, 7, 5, 5, 1])
_QUAD_K = np.array([2, 3, 4, 1, 6, 7, 5, 4, 5, 1, 7, 6])


def _box_corners(box):
    """8 corners of an OrientedBox, ordered as two z-stacked rings."""
    signs = np.array([[sx, sy, sz] for sz in (-1, 1) for sy in (1, -1)
                      for sx in (1, -1)], dtype=float)
    local = signs * box.half
    return local @ box.rot.T + box.center


def _box_corners_from_tf(half, tcp, along_z):
    signs = np.array([[sx, sy, sz] for sz in (-1, 1) for sy in (1, -1)
                      for sx in (1, -1)], dtype=float)
    local = signs * half
    tf = tcp.copy()
    tf[:3, 3] = tcp[:3, 3] + tcp[:3, 2] * along_z
    return local @ tf[:3, :3].T + tf[:3, 3]


if __name__ == "__main__":
    main()
