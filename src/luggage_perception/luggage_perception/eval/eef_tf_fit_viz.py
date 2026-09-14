"""Eval-only TF + mesh + cloud seating check. Offline, no ROS graph.

Places arm / suction panel / mounter / camera / Livox visuals and TF axes
in ``elfin_base_link`` using the locked EEF tree, and documents how D555
depth and Mid-360 points are projected into that same frame.
"""
from __future__ import division

import json
import os
import struct

import numpy as np

from luggage_description.handeye_layer3 import T_xyz_rpy
from luggage_perception.eval.bag_tf import apply_matrix
from luggage_perception.eval.lidar_camera_calib_export import (
    BASE_FRAME,
    CAMERA_LINK,
    EOF_FRAME,
    LIVOX_FRAME,
    MID360_MOUNT_FRAME,
    MOUNTER_FRAME,
    PANEL_FRAME,
    format_tf_tree_text,
)
from luggage_perception.eval.table_patch_icp import read_ply_points, write_ply_xyzrgb


SP_VISUAL_XYZ = (-0.0909, -0.28583, -0.0909)
# D555 Datasheet v1.1 envelope in camera_link: X=depth 48 mm, Y=length 167 mm,
# Z=height 42 mm. Front face at +X=0; Y offset is half of the 95 mm baseline.
CAM_BOX_SIZE = (0.048, 0.167, 0.042)
CAM_BOX_ORIGIN = (-0.024, -0.0475, 0.0)
CAM_LENS_SIZE = (0.001, 0.167, 0.038)
CAM_LENS_ORIGIN = (0.0005, -0.0475, 0.0)
LIVOX_BOX_SIZE = (0.065, 0.065, 0.060)
LIVOX_BOX_ORIGIN = (0.0, 0.0, 0.030)

ARM_LINKS = (
    ("elfin_base", "elfin_base.STL", (0.55, 0.55, 0.58, 0.45)),
    ("elfin_link1", "elfin_link1.STL", (0.75, 0.78, 0.82, 0.35)),
    ("elfin_link2", "elfin_link2.STL", (0.75, 0.78, 0.82, 0.35)),
    ("elfin_link3", "elfin_link3.STL", (0.75, 0.78, 0.82, 0.35)),
    ("elfin_link4", "elfin_link4.STL", (0.75, 0.78, 0.82, 0.35)),
    ("elfin_link5", "elfin_link5.STL", (0.75, 0.78, 0.82, 0.35)),
    ("elfin_link6", "elfin_link6.STL", (0.75, 0.78, 0.82, 0.35)),
    ("elfin_end_link", "elfin_end_link.STL", (0.85, 0.55, 0.25, 0.45)),
)

EEF_FOCUS = (
    EOF_FRAME, PANEL_FRAME, MOUNTER_FRAME, MID360_MOUNT_FRAME, LIVOX_FRAME,
    CAMERA_LINK, "d555_link", "d555_color_frame", "d555_color_optical_frame",
)


def _ws_root():
    return os.path.abspath(os.path.join(
        os.path.dirname(__file__), "..", "..", "..", ".."))


def _pkg_share(package):
    src = os.path.join(_ws_root(), "src", package)
    if os.path.isdir(src):
        return src
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory(package)
    except Exception:
        return src


def _first_existing(paths):
    for path in paths:
        if path and os.path.isfile(path):
            return path
    return None


def read_binary_stl(path, max_faces=0):
    """Return (vertices, faces) from a binary STL. Faces are (N, 3) indices."""
    with open(path, "rb") as handle:
        _header = handle.read(80)
        raw_n = handle.read(4)
        if len(raw_n) != 4:
            raise ValueError("truncated STL: %s" % path)
        n_tri = struct.unpack("<I", raw_n)[0]
        blob = handle.read()
    rec = 50
    if len(blob) < n_tri * rec:
        n_tri = len(blob) // rec
    stride = 1
    if max_faces and n_tri > int(max_faces):
        stride = int(np.ceil(float(n_tri) / float(max_faces)))
    verts = []
    faces = []
    for i in range(0, n_tri, stride):
        off = i * rec
        tri = blob[off:off + rec]
        if len(tri) < 48:
            break
        pts = np.frombuffer(tri[12:48], dtype="<f4").reshape(3, 3)
        base = len(verts)
        verts.extend(pts)
        faces.append((base, base + 1, base + 2))
    if not verts:
        return np.zeros((0, 3), dtype=np.float64), np.zeros((0, 3), dtype=np.int32)
    return (np.asarray(verts, dtype=np.float64),
            np.asarray(faces, dtype=np.int32))


def box_mesh(size, origin=(0.0, 0.0, 0.0)):
    """Axis-aligned box centred at *origin* with full *size* (sx, sy, sz)."""
    hx, hy, hz = 0.5 * np.asarray(size, dtype=np.float64)
    ox, oy, oz = origin
    corners = np.array([
        [-hx, -hy, -hz], [hx, -hy, -hz], [hx, hy, -hz], [-hx, hy, -hz],
        [-hx, -hy, hz], [hx, -hy, hz], [hx, hy, hz], [-hx, hy, hz],
    ], dtype=np.float64) + np.array([ox, oy, oz], dtype=np.float64)
    faces = np.array([
        [0, 1, 2], [0, 2, 3], [4, 6, 5], [4, 7, 6],
        [0, 4, 5], [0, 5, 1], [1, 5, 6], [1, 6, 2],
        [2, 6, 7], [2, 7, 3], [3, 7, 4], [3, 4, 0],
    ], dtype=np.int32)
    return corners, faces


def transform_mesh(T, vertices, faces, visual_T=None, scale=1.0):
    pts = np.asarray(vertices, dtype=np.float64) * float(scale)
    if visual_T is not None:
        pts = apply_matrix(visual_T, pts)
    return apply_matrix(T, pts), np.asarray(faces, dtype=np.int32)


def hops_to_T(hops):
    """Product of parent<-child hop matrices (points in last -> first)."""
    composed = np.eye(4, dtype=np.float64)
    for hop in hops or []:
        if hop.get("missing"):
            return None
        composed = composed.dot(np.asarray(hop["matrix"], dtype=np.float64))
    return composed


def frame_poses_from_tree(tree):
    """``T_elfin_base_link_from_frame`` for every hop in the dump tree."""
    poses = {BASE_FRAME: np.eye(4, dtype=np.float64)}
    chains = [
        tree.get("arm_base_to_eof") or {},
        tree.get("base_to_livox") or {},
        tree.get("base_to_optical") or {},
        tree.get("eof_to_livox") or {},
        tree.get("eof_to_optical") or {},
    ]
    progressed = True
    while progressed:
        progressed = False
        for blob in chains:
            for hop in blob.get("hops") or []:
                if hop.get("missing"):
                    continue
                parent = str(hop["parent"])
                child = str(hop["child"])
                if parent in poses and child not in poses:
                    poses[child] = poses[parent].dot(
                        np.asarray(hop["matrix"], dtype=np.float64))
                    progressed = True
    return poses


def projection_recipe(tree):
    """Documented camera-depth and Livox projection into elfin_base_link."""
    optical = str(tree.get("optical_frame") or "d555_color_optical_frame")
    cam_hops = (tree.get("base_to_optical") or {}).get("hops") or []
    liv_hops = (tree.get("base_to_livox") or {}).get("hops") or []
    return {
        "common_frame": BASE_FRAME,
        "lookup": (
            "BagTfBuffer.lookup_via_eof_mounter(target, source): "
            "T_target_from_source = T_target_from_eof "
            "@ (eof -> suction_panel -> eef_mount_adapter -> source)"
        ),
        "camera_depth": {
            "sensor_frame": optical,
            "deproject": (
                "aligned depth millimetres, colour K: "
                "z = depth_mm * 0.001; "
                "x = (u - cx) * z / fx; "
                "y = (v - cy) * z / fy; "
                "drop z==0. Points are in %s." % optical
            ),
            "hops": [
                "%s -> %s" % (h["parent"], h["child"])
                for h in cam_hops if not h.get("missing")
            ],
            "formula": "p_base = T_%s_from_%s @ p_optical" % (
                BASE_FRAME, optical),
            "T_elfin_base_link_from_optical":
                tree.get("T_elfin_base_link_from_optical"),
        },
        "livox": {
            "sensor_frame": LIVOX_FRAME,
            "source": (
                "driver PointCloud2 / CustomMsg in livox_frame "
                "(optical centre, handbook +X heading)"
            ),
            "hops": [
                "%s -> %s" % (h["parent"], h["child"])
                for h in liv_hops if not h.get("missing")
            ],
            "formula": "p_base = T_%s_from_%s @ p_livox" % (
                BASE_FRAME, LIVOX_FRAME),
            "T_elfin_base_link_from_livox_frame":
                tree.get("T_elfin_base_link_from_livox_frame"),
        },
        "fixed_eef_edges": [
            "%s -> %s (flange CAD)" % (EOF_FRAME, PANEL_FRAME),
            "%s -> %s (solved adapter, locked T_icp)" % (
                PANEL_FRAME, MOUNTER_FRAME),
            "%s -> %s (CAD square pocket)" % (
                MOUNTER_FRAME, MID360_MOUNT_FRAME),
            "%s -> %s (handbook 47 mm optical)" % (
                MID360_MOUNT_FRAME, LIVOX_FRAME),
            "%s -> %s (Layer 3 re-expressed)" % (MOUNTER_FRAME, CAMERA_LINK),
            "%s -> d555_link -> d555_color_frame -> %s (driver)" % (
                CAMERA_LINK, optical),
        ],
    }


def _axis_traces(origin, rot, name, length=0.05):
    traces = []
    colors = ("#ff5555", "#44dd66", "#5599ff")
    labels = ("X", "Y", "Z")
    o = np.asarray(origin, dtype=np.float64)
    for i in range(3):
        tip = o + np.asarray(rot[:, i], dtype=np.float64) * float(length)
        traces.append({
            "type": "scatter3d", "mode": "lines",
            "name": "%s %s" % (name, labels[i]),
            "x": [float(o[0]), float(tip[0])],
            "y": [float(o[1]), float(tip[1])],
            "z": [float(o[2]), float(tip[2])],
            "line": {"color": colors[i], "width": 6},
            "hoverinfo": "name",
            "showlegend": False,
            "legendgroup": "tf_axes",
        })
    traces.append({
        "type": "scatter3d", "mode": "text",
        "name": name,
        "x": [float(o[0])], "y": [float(o[1])], "z": [float(o[2])],
        "text": [name],
        "textfont": {"size": 11, "color": "#ffe08a"},
        "showlegend": False,
        "legendgroup": "tf_labels",
    })
    return traces


def _edge_trace(p0, p1, name):
    a = np.asarray(p0, dtype=np.float64)
    b = np.asarray(p1, dtype=np.float64)
    return {
        "type": "scatter3d", "mode": "lines",
        "name": name,
        "x": [float(a[0]), float(b[0])],
        "y": [float(a[1]), float(b[1])],
        "z": [float(a[2]), float(b[2])],
        "line": {"color": "#f6d36e", "width": 3, "dash": "dot"},
        "hoverinfo": "name",
        "showlegend": False,
        "legendgroup": "tf_edges",
    }


def _mesh_trace(vertices, faces, name, color, opacity=0.55):
    v = np.asarray(vertices, dtype=np.float64)
    f = np.asarray(faces, dtype=np.int32)
    if len(v) == 0 or len(f) == 0:
        return None
    rgba = "rgba(%d,%d,%d,%.2f)" % (
        int(color[0] * 255), int(color[1] * 255), int(color[2] * 255),
        float(opacity if len(color) < 4 else color[3]))
    return {
        "type": "mesh3d",
        "name": name,
        "x": v[:, 0].tolist(),
        "y": v[:, 1].tolist(),
        "z": v[:, 2].tolist(),
        "i": f[:, 0].tolist(),
        "j": f[:, 1].tolist(),
        "k": f[:, 2].tolist(),
        "color": rgba,
        "opacity": float(opacity if len(color) < 4 else color[3]),
        "flatshading": True,
        "hoverinfo": "name",
        "legendgroup": name,
    }


def _cloud_trace(points, name, color, size=2, rgb=None, cap=5000):
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    if pts.shape[0] == 0:
        return None
    cols = None if rgb is None else np.asarray(rgb).reshape(-1, 3)
    if pts.shape[0] > int(cap):
        rng = np.random.RandomState(0)
        idx = np.sort(rng.choice(pts.shape[0], size=int(cap), replace=False))
        pts = pts[idx]
        if cols is not None:
            cols = cols[idx]
    marker = {"size": int(size)}
    if cols is not None and len(cols) == len(pts):
        marker["color"] = [
            "rgb(%d,%d,%d)" % (int(r), int(g), int(b)) for r, g, b in cols]
    else:
        marker["color"] = color
    return {
        "type": "scatter3d", "mode": "markers", "name": name,
        "x": pts[:, 0].tolist(),
        "y": pts[:, 1].tolist(),
        "z": pts[:, 2].tolist(),
        "marker": marker,
        "legendgroup": name,
    }


def load_visuals(poses, arm_max_faces=1400):
    """Load URDF visuals, transformed into elfin_base_link."""
    items = []
    elfin = os.path.join(_pkg_share("elfin_description"), "meshes", "S20")
    for link, fname, color in ARM_LINKS:
        T = poses.get(link)
        path = os.path.join(elfin, fname)
        if T is None or not os.path.isfile(path):
            continue
        verts, faces = read_binary_stl(path, max_faces=arm_max_faces)
        pts, faces = transform_mesh(T, verts, faces)
        items.append({
            "name": "arm/%s" % link, "group": "arm", "frame": link,
            "vertices": pts, "faces": faces, "color": color,
        })

    desc_share = os.path.join(
        _ws_root(), "install", "luggage_description", "share",
        "luggage_description", "meshes", "suction_panel")
    try:
        from ament_index_python.packages import get_package_share_directory
        desc_share = os.path.join(
            get_package_share_directory("luggage_description"),
            "meshes", "suction_panel")
    except Exception:
        pass
    T_panel = poses.get(PANEL_FRAME)
    vis = T_xyz_rpy(SP_VISUAL_XYZ, (0.0, 0.0, 0.0))
    if T_panel is not None:
        for part in ("p01", "p02", "p03", "p04", "p05"):
            path = os.path.join(desc_share, "%s.stl" % part)
            if not os.path.isfile(path):
                continue
            verts, faces = read_binary_stl(path)
            pts, faces = transform_mesh(T_panel, verts, faces, visual_T=vis)
            items.append({
                "name": "suction_panel/%s" % part, "group": "suction_panel",
                "frame": PANEL_FRAME,
                "vertices": pts, "faces": faces,
                "color": (0.72, 0.73, 0.76, 0.55),
            })

    T_adp = poses.get(MOUNTER_FRAME)
    adp_path = _first_existing([
        os.path.join(_pkg_share("luggage_gazebo"), "models", "arm_realsense",
                     "arm_realsense_v1.3.stl"),
        os.path.join(_ws_root(), "src", "luggage_gazebo", "models",
                     "arm_realsense", "arm_realsense_v1.3.stl"),
        os.path.join(_ws_root(), "install", "luggage_description", "share",
                     "luggage_description", "meshes", "arm_realsense",
                     "arm_realsense_v1.3.stl"),
    ])
    if T_adp is not None and adp_path:
        verts, faces = read_binary_stl(adp_path)
        pts, faces = transform_mesh(T_adp, verts, faces, scale=0.001)
        items.append({
            "name": "eef_mount_adapter", "group": "adapter",
            "frame": MOUNTER_FRAME,
            "vertices": pts, "faces": faces,
            "color": (0.15, 0.72, 0.68, 0.55),
        })

    T_cam = poses.get(CAMERA_LINK)
    if T_cam is not None:
        verts, faces = box_mesh(CAM_BOX_SIZE, CAM_BOX_ORIGIN)
        pts, faces = transform_mesh(T_cam, verts, faces)
        items.append({
            "name": "camera_link", "group": "camera", "frame": CAMERA_LINK,
            "vertices": pts, "faces": faces,
            "color": (0.55, 0.56, 0.62, 0.78),
        })
        lens, lf = box_mesh(CAM_LENS_SIZE, CAM_LENS_ORIGIN)
        lp, lf = transform_mesh(T_cam, lens, lf)
        items.append({
            "name": "camera_lens", "group": "camera", "frame": CAMERA_LINK,
            "vertices": lp, "faces": lf,
            "color": (0.1, 0.25, 0.7, 0.85),
        })

    T_mid = poses.get(MID360_MOUNT_FRAME)
    if T_mid is not None:
        verts, faces = box_mesh(LIVOX_BOX_SIZE, LIVOX_BOX_ORIGIN)
        pts, faces = transform_mesh(T_mid, verts, faces)
        items.append({
            "name": "livox_body", "group": "livox",
            "frame": MID360_MOUNT_FRAME,
            "vertices": pts, "faces": faces,
            "color": (0.15, 0.45, 0.78, 0.45),
        })
    return items


def _subsample_pts(points, cap):
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    if pts.shape[0] <= cap:
        return pts
    rng = np.random.RandomState(0)
    idx = np.sort(rng.choice(pts.shape[0], size=int(cap), replace=False))
    return pts[idx]


def _poly_xy(vertices, faces, max_faces=4000):
    v = np.asarray(vertices, dtype=np.float64)
    f = np.asarray(faces, dtype=np.int32)
    if len(v) == 0 or len(f) == 0:
        return []
    if len(f) > int(max_faces):
        stride = int(np.ceil(float(len(f)) / float(max_faces)))
        f = f[::stride]
    return [v[tri][:, :2] for tri in f]


def write_png_views(out_dir, items, poses, cam_pts, lid_pts):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection

    T_adp = poses.get(MOUNTER_FRAME)
    if T_adp is None:
        T_adp = np.eye(4, dtype=np.float64)
    T_adp_inv = np.linalg.inv(T_adp)

    def _in_adapter(vertices):
        return apply_matrix(T_adp_inv, vertices)

    def _draw_groups(ax, groups, to_xy, hide_arm=True, labels=None):
        for item in items:
            if hide_arm and item["group"] == "arm":
                continue
            if groups and item["group"] not in groups:
                continue
            xy3 = to_xy(item["vertices"])
            polys = _poly_xy(xy3, item["faces"])
            if not polys:
                continue
            col = item["color"]
            coll = PolyCollection(
                polys, facecolors=(col[0], col[1], col[2],
                                   float(col[3] if len(col) > 3 else 0.45)),
                edgecolors=(0.1, 0.1, 0.1, 0.15), linewidths=0.15)
            ax.add_collection(coll)
        for name in (labels if labels is not None else EEF_FOCUS):
            T = poses.get(name)
            if T is None:
                continue
            o = to_xy(T[:3, 3].reshape(1, 3))[0]
            ax.scatter([o[0]], [o[1]], s=18, c="#ffe08a", zorder=5)
            ax.annotate(name, (o[0], o[1]), color="#ffe08a", fontsize=7,
                        textcoords="offset points", xytext=(4, 4))
            R = T[:3, :3]
            o3 = T[:3, 3]
            for i, c in enumerate(("#f55", "#4d6", "#59f")):
                tip = to_xy((o3 + R[:, i] * 0.04).reshape(1, 3))[0]
                ax.plot([o[0], tip[0]], [o[1], tip[1]], color=c, lw=1.6, zorder=6)

    def iso_xy(pts):
        p = np.asarray(pts, dtype=np.float64).reshape(-1, 3)
        return np.column_stack((p[:, 0] + 0.5 * p[:, 2], p[:, 1] + 0.35 * p[:, 2]))

    def adapter_xz(pts):
        p = _in_adapter(np.asarray(pts, dtype=np.float64).reshape(-1, 3))
        return p[:, [0, 2]]

    def adapter_xy(pts):
        p = _in_adapter(np.asarray(pts, dtype=np.float64).reshape(-1, 3))
        return p[:, [0, 1]]

    def _save(path, groups, title, to_xy, xlabel, ylabel, lim=0.16, origin_xy=(0.0, 0.0),
              labels=None):
        fig = plt.figure(figsize=(8.5, 8), facecolor="#111")
        ax = fig.add_subplot(111)
        ax.set_facecolor("#111")
        _draw_groups(ax, groups, to_xy, labels=labels)
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlim(origin_xy[0] - lim, origin_xy[0] + lim)
        ax.set_ylim(origin_xy[1] - lim, origin_xy[1] + lim)
        ax.set_xlabel(xlabel, color="#ccc")
        ax.set_ylabel(ylabel, color="#ccc")
        ax.set_title(title, color="#eee")
        ax.tick_params(colors="#aaa")
        ax.grid(True, color="#333", lw=0.4)
        fig.savefig(path, dpi=130, bbox_inches="tight", facecolor="#111")
        plt.close(fig)

    c_iso = iso_xy(T_adp[:3, 3].reshape(1, 3))[0]
    _save(os.path.join(out_dir, "eef_iso.png"),
          {"suction_panel", "adapter", "camera", "livox"},
          "EEF isometric (elfin_base_link X+0.5Z, Y+0.35Z)",
          iso_xy, "X + 0.5 Z (m)", "Y + 0.35 Z (m)",
          lim=0.20, origin_xy=c_iso,
          labels=(PANEL_FRAME, MOUNTER_FRAME, MID360_MOUNT_FRAME, LIVOX_FRAME,
                  CAMERA_LINK))
    _save(os.path.join(out_dir, "eef_pocket.png"),
          {"adapter", "livox", "camera"},
          "Mounter pocket: look along adapter +Y (adapter X vs Z)",
          adapter_xz, "adapter X (m)", "adapter Z (m)",
          lim=0.12, origin_xy=(0.0, 0.0),
          labels=(MOUNTER_FRAME, MID360_MOUNT_FRAME, LIVOX_FRAME, CAMERA_LINK,
                  "d555_color_optical_frame"))
    _save(os.path.join(out_dir, "eef_adapter_xy.png"),
          {"suction_panel", "adapter", "camera", "livox"},
          "Look along adapter +Z (adapter X vs Y) — Livox +Z is adapter +Y",
          adapter_xy, "adapter X (m)", "adapter Y (m)",
          lim=0.18, origin_xy=(0.0, 0.08),
          labels=(PANEL_FRAME, MOUNTER_FRAME, MID360_MOUNT_FRAME, LIVOX_FRAME,
                  CAMERA_LINK))

    def _scatter_clouds(path, title, xlim=None, ylim=None, lid_cap=40000):
        fig = plt.figure(figsize=(8, 8), facecolor="#111")
        ax = fig.add_subplot(111)
        ax.set_facecolor("#111")
        cam = np.asarray(cam_pts, dtype=np.float64).reshape(-1, 3)
        lid = np.asarray(lid_pts, dtype=np.float64).reshape(-1, 3)
        if xlim is not None:
            if len(lid):
                lid = lid[(lid[:, 0] >= xlim[0]) & (lid[:, 0] <= xlim[1])
                          & (lid[:, 1] >= ylim[0]) & (lid[:, 1] <= ylim[1])]
            if len(cam):
                cam = cam[(cam[:, 0] >= xlim[0]) & (cam[:, 0] <= xlim[1])
                          & (cam[:, 1] >= ylim[0]) & (cam[:, 1] <= ylim[1])]
        cam = _subsample_pts(cam, 12000)
        lid = _subsample_pts(lid, lid_cap)
        if len(lid):
            ax.scatter(lid[:, 0], lid[:, 1], s=2, c="#f6c", alpha=0.35, label="Mid-360")
        if len(cam):
            ax.scatter(cam[:, 0], cam[:, 1], s=2, c="#6cf", alpha=0.45, label="D555 depth")
        ax.set_aspect("equal", adjustable="box")
        if xlim is not None:
            ax.set_xlim(*xlim)
            ax.set_ylim(*ylim)
        ax.set_xlabel("X elfin_base_link (m)")
        ax.set_ylabel("Y elfin_base_link (m)")
        ax.set_title(title)
        ax.legend(facecolor="#222", edgecolor="#444", labelcolor="#eee")
        ax.tick_params(colors="#aaa")
        fig.savefig(path, dpi=120, bbox_inches="tight", facecolor="#111")
        plt.close(fig)

    _scatter_clouds(os.path.join(out_dir, "clouds_xy_full.png"),
                    "Projected clouds XY (full Livox FOV)")
    if len(cam_pts):
        cam_all = np.asarray(cam_pts, dtype=np.float64).reshape(-1, 3)
        pad = 0.08
        xlim = (float(np.percentile(cam_all[:, 0], 1) - pad),
                float(np.percentile(cam_all[:, 0], 99) + pad))
        ylim = (float(np.percentile(cam_all[:, 1], 1) - pad),
                float(np.percentile(cam_all[:, 1], 99) + pad))
        _scatter_clouds(os.path.join(out_dir, "clouds_xy.png"),
                        "Projected D555 depth vs Mid-360 (camera FOV XY)",
                        xlim=xlim, ylim=ylim)


_HTML = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8"/>
<title>锁定 TF 树 / 外观贴合 / 点云投影</title>
<script src="https://cdn.plot.ly/plotly-2.27.0.min.js"></script>
<style>
 body { font-family: sans-serif; margin: 12px; background: #111; color: #eee; }
 h1 { font-size: 20px; }
 h2 { font-size: 16px; margin-top: 18px; }
 .meta, pre { color: #ccc; font-size: 13px; white-space: pre-wrap; }
 .row { display: flex; gap: 12px; flex-wrap: wrap; }
 .card { background: #1c1c1c; padding: 10px; border-radius: 8px; }
 table { border-collapse: collapse; font-size: 13px; }
 td, th { border: 1px solid #444; padding: 4px 8px; }
 th { color: #9cf; }
 .tog { margin-right: 10px; }
 a { color: #8ab4f8; }
</style>
</head>
<body>
<h1>锁定 TF 树、外观贴合、D555 depth / Mid-360 同坐标系投影</h1>
<p class="meta">相机灰盒是 D555 Datasheet 167×42×48 mm（不是 D435 90×25×25）。ChArUco Layer 3 光学外参未改。</p>
<p class="meta" id="meta"></p>
<div class="card">
  <label class="tog"><input type="checkbox" data-g="arm" checked/>机械臂</label>
  <label class="tog"><input type="checkbox" data-g="suction_panel" checked/>吸盘面板</label>
  <label class="tog"><input type="checkbox" data-g="adapter" checked/>mounter</label>
  <label class="tog"><input type="checkbox" data-g="camera" checked/>相机</label>
  <label class="tog"><input type="checkbox" data-g="livox" checked/>Livox 壳体</label>
  <label class="tog"><input type="checkbox" data-g="tf_axes" checked/>TF 轴</label>
  <label class="tog"><input type="checkbox" data-g="tf_edges" checked/>TF 连线</label>
  <label class="tog"><input type="checkbox" data-g="tf_labels" checked/>TF 名</label>
  <label class="tog"><input type="checkbox" data-g="D555 depth" checked/>D555 depth</label>
  <label class="tog"><input type="checkbox" data-g="Mid-360" checked/>Mid-360</label>
</div>
<div class="row">
  <div class="card" style="flex:1;min-width:640px">
    <h2>全景（elfin_base_link）</h2>
    <div id="full" style="height:640px"></div>
  </div>
  <div class="card" style="flex:1;min-width:520px">
    <h2>EEF 贴合特写</h2>
    <div id="eef" style="height:640px"></div>
  </div>
</div>
<div class="row">
  <div class="card" style="flex:1;min-width:420px">
    <h2>TF 树（parent → child）</h2>
    <pre id="tree"></pre>
  </div>
  <div class="card" style="flex:1;min-width:420px">
    <h2>投影怎么算</h2>
    <pre id="proj"></pre>
  </div>
</div>
<div class="card">
  <h2>关节表</h2>
  <div id="hops"></div>
  <p>截图：<a href="eef_iso.png">eef_iso.png</a>
     <a href="eef_pocket.png">eef_pocket.png</a>
     <a href="eef_adapter_xy.png">eef_adapter_xy.png</a>
     <a href="clouds_xy.png">clouds_xy.png</a>
     · 说明：<a href="projection.md">projection.md</a></p>
</div>
<script>
const DATA = __DATA__;
document.getElementById("meta").textContent = DATA.meta || "";
document.getElementById("tree").textContent = DATA.tree_text || "";
document.getElementById("proj").textContent = DATA.projection_text || "";
function hopTable(hops) {
  if (!hops || !hops.length) return "";
  let html = "<table><tr><th>parent</th><th>child</th><th>xyz (m)</th><th>rpy</th></tr>";
  hops.forEach(h => {
    if (h.missing) {
      html += "<tr><td>"+h.parent+"</td><td>"+h.child+"</td><td colspan=2>MISSING</td></tr>";
      return;
    }
    const xyz = (h.xyz||[]).map(v => Number(v).toFixed(6)).join(" ");
    const rpy = (h.rpy||[]).map(v => Number(v).toFixed(8)).join(" ");
    html += "<tr><td>"+h.parent+"</td><td>"+h.child+"</td><td>"+xyz+"</td><td>"+rpy+"</td></tr>";
  });
  return html + "</table>";
}
document.getElementById("hops").innerHTML =
  "<p>arm FK</p>" + hopTable(DATA.arm_hops) +
  "<p>base → Livox</p>" + hopTable(DATA.livox_hops) +
  "<p>base → camera optical</p>" + hopTable(DATA.cam_hops);
const layout = (title, xrange, yrange, zrange) => ({
  paper_bgcolor: "#111", plot_bgcolor: "#111",
  font: {color: "#eee"},
  scene: {
    xaxis: {title: "X "+DATA.frame, range: xrange},
    yaxis: {title: "Y", range: yrange},
    zaxis: {title: "Z", range: zrange},
    aspectmode: xrange ? "cube" : "data",
    camera: {eye: {x: 1.4, y: 1.1, z: 0.9}}
  },
  margin: {t: 28, l: 0, r: 0, b: 0},
  legend: {font: {color: "#eee"}, itemsizing: "constant"},
  title
});
Plotly.newPlot("full", DATA.full_traces, layout("full"), {responsive: true});
Plotly.newPlot("eef", DATA.eef_traces, layout("eef", DATA.eef_xlim, DATA.eef_ylim, DATA.eef_zlim), {responsive: true});
function applyToggles() {
  const vis = {};
  document.querySelectorAll("input[data-g]").forEach(el => { vis[el.dataset.g] = el.checked; });
  function mask(traces) {
    return traces.map(t => {
      const g = t.legendgroup || t.name || "";
      const key = Object.keys(vis).find(k => g === k || g.indexOf(k) === 0);
      return (key == null) ? true : vis[key];
    });
  }
  Plotly.restyle("full", {visible: mask(DATA.full_traces)});
  Plotly.restyle("eef", {visible: mask(DATA.eef_traces)});
}
document.querySelectorAll("input[data-g]").forEach(el => el.addEventListener("change", applyToggles));
</script>
</body>
</html>
"""


def _recipe_text(recipe):
    cam = recipe["camera_depth"]
    liv = recipe["livox"]
    lines = [
        "共同坐标系: %s" % recipe["common_frame"],
        "查找函数: %s" % recipe["lookup"],
        "",
        "【D555 depth】",
        "1. " + cam["deproject"],
        "2. 用下面 hop 连乘得到 T_base_from_optical（点在 optical → base）:",
    ]
    for hop in cam["hops"]:
        lines.append("     " + hop)
    lines.append("3. %s" % cam["formula"])
    lines.extend([
        "",
        "【Mid-360】",
        "1. " + liv["source"],
        "2. hop 连乘得到 T_base_from_livox_frame:",
    ])
    for hop in liv["hops"]:
        lines.append("     " + hop)
    lines.append("3. %s" % liv["formula"])
    lines.extend(["", "【EEF 固定边】"])
    lines.extend("- " + edge for edge in recipe["fixed_eef_edges"])
    return "\n".join(lines) + "\n"


def dump_eef_tf_fit(dump_dir, out_dir=None, cloud_cap=6000):
    """Build seating/TF/cloud HTML from an existing calib dump directory."""
    dump_dir = os.path.abspath(dump_dir)
    out_dir = os.path.abspath(out_dir or os.path.join(dump_dir, "tf_fit"))
    os.makedirs(out_dir, exist_ok=True)
    tree_path = os.path.join(dump_dir, "tf_tree.json")
    with open(tree_path, encoding="utf-8") as handle:
        tree = json.load(handle)
    poses = frame_poses_from_tree(tree)
    recipe = projection_recipe(tree)
    items = load_visuals(poses)

    cam_ply = os.path.join(dump_dir, "fused", "camera_depth_in_elfin_base_link.ply")
    lid_ply = os.path.join(dump_dir, "fused", "livox_in_elfin_base_link.ply")
    cam_pts, cam_rgb = (np.zeros((0, 3)), None)
    lid_pts = np.zeros((0, 3))
    if os.path.isfile(cam_ply):
        cam_pts, cam_rgb = read_ply_points(cam_ply)
    if os.path.isfile(lid_ply):
        lid_pts, _ = read_ply_points(lid_ply)

    full_traces = []
    eef_traces = []
    eef_groups = {"suction_panel", "adapter", "camera", "livox"}
    for item in items:
        tr = _mesh_trace(
            item["vertices"], item["faces"], item["name"], item["color"],
            opacity=item["color"][3] if len(item["color"]) > 3 else 0.5)
        if tr is None:
            continue
        tr["legendgroup"] = item["group"]
        full_traces.append(tr)
        if item["group"] in eef_groups:
            eef_traces.append(tr)

    hops_for_edges = []
    for key in ("arm_base_to_eof", "eof_to_livox", "eof_to_optical"):
        hops_for_edges.extend((tree.get(key) or {}).get("hops") or [])
    seen = set()
    for hop in hops_for_edges:
        if hop.get("missing"):
            continue
        parent, child = hop["parent"], hop["child"]
        if parent not in poses or child not in poses:
            continue
        edge_id = (parent, child)
        if edge_id in seen:
            continue
        seen.add(edge_id)
        tr = _edge_trace(poses[parent][:3, 3], poses[child][:3, 3],
                         "%s→%s" % (parent, child))
        full_traces.append(tr)
        if parent in EEF_FOCUS or child in EEF_FOCUS:
            eef_traces.append(tr)

    for name, T in poses.items():
        axes = _axis_traces(T[:3, 3], T[:3, :3], name,
                            length=0.08 if name in EEF_FOCUS else 0.05)
        full_traces.extend(axes)
        if name in EEF_FOCUS:
            eef_traces.extend(_axis_traces(T[:3, 3], T[:3, :3], name, 0.04))

    cam_tr = _cloud_trace(cam_pts, "D555 depth", "#66ccff", size=2,
                          rgb=cam_rgb, cap=cloud_cap)
    lid_tr = _cloud_trace(lid_pts, "Mid-360", "#ff66cc", size=2, cap=cloud_cap)
    if cam_tr:
        full_traces.append(cam_tr)
    if lid_tr:
        full_traces.append(lid_tr)

    T_adp = poses.get(MOUNTER_FRAME, np.eye(4))
    c = T_adp[:3, 3]
    pad = 0.16
    eef_xlim = [float(c[0] - pad), float(c[0] + pad)]
    eef_ylim = [float(c[1] - pad), float(c[1] + pad)]
    eef_zlim = [float(c[2] - pad), float(c[2] + pad)]

    try:
        write_png_views(out_dir, items, poses, cam_pts, lid_pts)
        png_ok = True
    except Exception as exc:
        png_ok = False
        with open(os.path.join(out_dir, "png_error.txt"), "w",
                  encoding="utf-8") as handle:
            handle.write("%s: %s\n" % (type(exc).__name__, exc))

    mesh_pts = []
    mesh_rgb = []
    colors = {
        "arm": (180, 180, 190),
        "suction_panel": (200, 200, 210),
        "adapter": (40, 190, 175),
        "camera": (150, 155, 170),
        "livox": (40, 120, 210),
    }
    for item in items:
        v = _subsample_pts(item["vertices"], 4000)
        rgb = np.tile(np.array([colors.get(item["group"], (200, 200, 200))],
                               dtype=np.uint8), (len(v), 1))
        if len(v):
            mesh_pts.append(v)
            mesh_rgb.append(rgb)
    if mesh_pts:
        write_ply_xyzrgb(
            os.path.join(out_dir, "meshes_in_elfin_base_link.ply"),
            np.concatenate(mesh_pts, axis=0),
            np.concatenate(mesh_rgb, axis=0))

    recipe_text = _recipe_text(recipe)
    with open(os.path.join(out_dir, "projection.json"), "w",
              encoding="utf-8") as handle:
        json.dump(recipe, handle, indent=2)
        handle.write("\n")
    with open(os.path.join(out_dir, "projection.md"), "w",
              encoding="utf-8") as handle:
        handle.write("# D555 depth / Mid-360 投影到 elfin_base_link\n\n")
        handle.write(
            "Camera visual is D555 Datasheet v1.1 envelope "
            "167 x 42 x 48 mm in `camera_link` "
            "(front face at +X=0, Y offset 47.5 mm). "
            "ChArUco Layer 3 / `cam_mount_*` is unchanged.\n\n")
        handle.write("```\n%s```\n" % recipe_text)
    tree_text = format_tf_tree_text(tree)
    with open(os.path.join(out_dir, "tf_tree.txt"), "w", encoding="utf-8") as handle:
        handle.write(tree_text)

    meta = [
        "dump: %s" % dump_dir,
        "common frame: %s" % BASE_FRAME,
        "optical: %s   livox: %s" % (
            tree.get("optical_frame"), LIVOX_FRAME),
        "meshes: %d   camera pts: %d   livox pts: %d" % (
            len(items), int(len(cam_pts)), int(len(lid_pts))),
        "png_ok: %s" % png_ok,
    ]
    html = _HTML.replace("__DATA__", json.dumps({
        "meta": "\n".join(meta),
        "frame": BASE_FRAME,
        "tree_text": tree_text,
        "projection_text": recipe_text,
        "arm_hops": (tree.get("arm_base_to_eof") or {}).get("hops") or [],
        "livox_hops": (tree.get("base_to_livox") or {}).get("hops") or [],
        "cam_hops": (tree.get("base_to_optical") or {}).get("hops") or [],
        "full_traces": full_traces,
        "eef_traces": eef_traces,
        "eef_xlim": eef_xlim,
        "eef_ylim": eef_ylim,
        "eef_zlim": eef_zlim,
    }))
    html_path = os.path.join(out_dir, "tf_fit.html")
    with open(html_path, "w", encoding="utf-8") as handle:
        handle.write(html)
    index = [
        "# TF fit visualization",
        "",
        "- HTML: `tf_fit.html`",
        "- Projection: `projection.md`",
        "- TF tree: `tf_tree.txt`",
        "- EEF iso: `eef_iso.png`",
        "- Pocket: `eef_pocket.png`",
        "- Adapter XY: `eef_adapter_xy.png`",
        "- Clouds XY: `clouds_xy.png`",
        "- Clouds XY full FOV: `clouds_xy_full.png`",
        "- Meshes PLY: `meshes_in_elfin_base_link.ply`",
        "",
    ]
    with open(os.path.join(out_dir, "INDEX.md"), "w", encoding="utf-8") as handle:
        handle.write("\n".join(index))
    return {
        "out_dir": out_dir,
        "html": html_path,
        "n_meshes": len(items),
        "n_camera": int(len(cam_pts)),
        "n_livox": int(len(lid_pts)),
        "frames": sorted(poses.keys()),
        "png_ok": png_ok,
        "recipe": recipe,
    }
