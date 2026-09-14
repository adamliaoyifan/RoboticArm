"""Eval-only Mid-360 vs D555 dump for mount-TF ICP.

Offline mcap: no ROS graph. Camera TF and mounter TF are recorded as
fixed; the adjustable edge is ``eef_mount_adapter -> mid360_mount_frame``.
Does not run ICP and does not change online nodes or URDF.
"""
from __future__ import division

import json
import os

import numpy as np

from luggage_perception.depth_deprojection import deproject_selected
from luggage_perception.eval.bag_frame_join import (
    dedupe_stamped_entries,
    nearest_stamp,
    plan_frame_join,
)
from luggage_perception.eval.bag_mcap_source import (
    COLOR_INFO_TOPIC,
    DEPTH_INFO_TOPIC,
    JOINT_TOPIC,
    LIDAR_TOPIC,
    TCP_TOPIC,
    TF_STATIC_TOPIC,
    TF_TOPIC,
    decode_color_message,
    decode_depth_message,
    decode_lidar_scan,
    find_mcap_file,
    iter_bag_messages,
    scan_bag,
    select_image_topics,
)
from luggage_perception.eval.bag_tf import (
    BagTfBuffer,
    apply_matrix,
    transform_to_matrix,
)
from luggage_perception.eval.gate4_dump import write_ply_xyz
from luggage_perception.ros_message_adapters import camera_info_frame_from_msg

NS_PER_MS = 1_000_000
MOUNTER_FRAME = "eef_mount_adapter"
LIVOX_FRAME = "livox_frame"
MID360_MOUNT_FRAME = "mid360_mount_frame"
CAMERA_LINK = "camera_link"

_ADJUST_EDGE = (MOUNTER_FRAME, MID360_MOUNT_FRAME)
_FIXED_EDGES = (
    (MOUNTER_FRAME, CAMERA_LINK),
    ("suction_panel", MOUNTER_FRAME),
    (MID360_MOUNT_FRAME, LIVOX_FRAME),
    (CAMERA_LINK, "d555_link"),
    ("d555_link", "d555_color_frame"),
    ("d555_color_frame", "d555_color_optical_frame"),
)


def _stamp_ns(stamped):
    stamp = stamped.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _tf_dict(stamped):
    t = stamped.transform.translation
    r = stamped.transform.rotation
    mat = transform_to_matrix(stamped.transform)
    return {
        "parent": str(stamped.header.frame_id),
        "child": str(stamped.child_frame_id),
        "xyz": [float(t.x), float(t.y), float(t.z)],
        "xyzw": [float(r.x), float(r.y), float(r.z), float(r.w)],
        "matrix": mat.tolist(),
        "stamp_ns": _stamp_ns(stamped),
    }


def _write_json(path, payload):
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2)
        handle.write("\n")


def write_ply_xyzrgb(path, points, colors):
    """Binary XYZRGB PLY (CloudCompare point-cloud friendly)."""
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    rgb = np.asarray(colors).reshape(-1, 3)
    n = min(len(pts), len(rgb))
    pts = pts[:n]
    rgb = np.clip(rgb[:n], 0, 255).astype(np.uint8)
    header = (
        "ply\n"
        "format binary_little_endian 1.0\n"
        "comment lidar-camera calib dump\n"
        "element vertex %d\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "property uchar red\n"
        "property uchar green\n"
        "property uchar blue\n"
        "element face 0\n"
        "property list uchar int vertex_indices\n"
        "end_header\n"
    ) % n
    rec = np.empty(n, dtype=[
        ("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
        ("r", "u1"), ("g", "u1"), ("b", "u1"),
    ])
    rec["x"] = pts[:, 0]
    rec["y"] = pts[:, 1]
    rec["z"] = pts[:, 2]
    rec["r"] = rgb[:, 0]
    rec["g"] = rgb[:, 1]
    rec["b"] = rgb[:, 2]
    with open(path, "wb") as handle:
        handle.write(header.encode("ascii"))
        handle.write(rec.tobytes())
    return int(n)


def _desc_config(name):
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(
            get_package_share_directory("luggage_description"), "config", name)
        if os.path.isfile(path):
            return path
    except Exception:
        pass
    return os.path.abspath(os.path.join(
        os.path.dirname(__file__), "..", "..", "..", "luggage_description",
        "config", name))


def apply_current_xacro_mounts(tf_buffer):
    """Overwrite bag static sensor-mount edges with current URDF xacro."""
    from luggage_description.handeye_layer3 import T_xyz_rpy, parse_xacro_xyz_rpy
    cam_xyz, cam_rpy = parse_xacro_xyz_rpy(
        _desc_config("camera_mount_origin.xacro"),
        "cam_mount_xyz", "cam_mount_rpy")
    mid_xyz, mid_rpy = parse_xacro_xyz_rpy(
        _desc_config("mid360_origin.xacro"),
        "mid360_mount_xyz", "mid360_mount_rpy")
    opt_xyz, opt_rpy = parse_xacro_xyz_rpy(
        _desc_config("mid360_origin.xacro"),
        "livox_optical_xyz", "livox_optical_rpy")
    adp_xyz, adp_rpy = parse_xacro_xyz_rpy(
        _desc_config("eef_mount_adapter_origin.xacro"),
        "adapter_mount_xyz", "adapter_mount_rpy")
    used = {
        "camera_mount_xyz": [float(v) for v in cam_xyz],
        "camera_mount_rpy": [float(v) for v in cam_rpy],
        "mid360_mount_xyz": [float(v) for v in mid_xyz],
        "mid360_mount_rpy": [float(v) for v in mid_rpy],
        "livox_optical_xyz": [float(v) for v in opt_xyz],
        "adapter_mount_xyz": [float(v) for v in adp_xyz],
    }
    tf_buffer.set_static(
        "eef_mount_adapter", "camera_link", T_xyz_rpy(cam_xyz, cam_rpy))
    tf_buffer.set_static(
        "eef_mount_adapter", "mid360_mount_frame", T_xyz_rpy(mid_xyz, mid_rpy))
    tf_buffer.set_static(
        "mid360_mount_frame", "livox_frame", T_xyz_rpy(opt_xyz, opt_rpy))
    tf_buffer.set_static(
        "suction_panel", "eef_mount_adapter", T_xyz_rpy(adp_xyz, adp_rpy))
    return used


def ransac_plane(points, n_iter=80, thresh=0.012, rng_seed=0):
    """Return (normal, offset, inlier_mask) for n·x = offset, or None."""
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    n = int(pts.shape[0])
    if n < 30:
        return None
    rng = np.random.RandomState(rng_seed)
    best = None
    for _ in range(int(n_iter)):
        idx = rng.choice(n, 3, replace=False)
        p0, p1, p2 = pts[idx]
        normal = np.cross(p1 - p0, p2 - p0)
        ln = np.linalg.norm(normal)
        if ln < 1e-9:
            continue
        normal = normal / ln
        offset = float(normal.dot(p0))
        dist = np.abs(pts.dot(normal) - offset)
        mask = dist < float(thresh)
        count = int(mask.sum())
        if best is None or count > best[0]:
            best = (count, normal, offset, mask)
    if best is None or best[0] < 30:
        return None
    inl = pts[best[3]]
    mean = inl.mean(axis=0)
    _, _, vh = np.linalg.svd(inl - mean, full_matrices=False)
    normal = vh[-1]
    if normal[2] < 0:
        normal = -normal
    offset = float(normal.dot(mean))
    mask = np.abs(pts.dot(normal) - offset) < float(thresh)
    return normal, offset, mask


def _plane_axes(normal):
    nrm = np.asarray(normal, dtype=np.float64)
    nrm = nrm / max(np.linalg.norm(nrm), 1e-12)
    helper = np.array([1.0, 0.0, 0.0]) if abs(nrm[0]) < 0.9 else np.array(
        [0.0, 1.0, 0.0])
    axis_x = np.cross(helper, nrm)
    axis_x = axis_x / max(np.linalg.norm(axis_x), 1e-12)
    axis_y = np.cross(nrm, axis_x)
    return axis_x, axis_y


def check_table_corner(cam_pts, cam_rgb, lid_pts, white_thresh=170,
                       plane_thresh=0.012):
    """Compare white-table hull corners of camera vs lidar in one frame."""
    cam_pts = np.asarray(cam_pts, dtype=np.float64).reshape(-1, 3)
    lid_pts = np.asarray(lid_pts, dtype=np.float64).reshape(-1, 3)
    rgb = np.asarray(cam_rgb).reshape(-1, 3)
    empty = {
        "ok": False,
        "reason": "empty",
        "corner_residual_m": None,
        "plane_gap_median_m": None,
        "plane_angle_deg": None,
        "nn_rmse_m": None,
    }
    if cam_pts.shape[0] == 0 or rgb.shape[0] != cam_pts.shape[0]:
        return empty
    white = ((rgb[:, 0] >= white_thresh) & (rgb[:, 1] >= white_thresh)
             & (rgb[:, 2] >= white_thresh))
    table_cam = cam_pts[white]
    if table_cam.shape[0] < 80:
        empty["reason"] = "not_enough_white_camera_points"
        empty["n_white"] = int(table_cam.shape[0])
        return empty
    fitted = ransac_plane(table_cam, thresh=plane_thresh)
    if fitted is None:
        empty["reason"] = "camera_plane_failed"
        return empty
    normal, offset, cam_inl = fitted
    cam_inliers = table_cam[cam_inl]
    lid_dist = np.abs(lid_pts.dot(normal) - offset) if len(lid_pts) else np.array([])
    lo, hi = cam_inliers.min(axis=0) - 0.08, cam_inliers.max(axis=0) + 0.08
    in_box = np.ones(len(lid_pts), dtype=bool)
    if len(lid_pts):
        in_box = np.all((lid_pts >= lo) & (lid_pts <= hi), axis=1)
    lid_near = lid_pts[in_box & (lid_dist < 0.04)] if len(lid_pts) else lid_pts
    axis_x, axis_y = _plane_axes(normal)
    origin = cam_inliers.mean(axis=0)
    cam_2d = np.column_stack([
        (cam_inliers - origin).dot(axis_x),
        (cam_inliers - origin).dot(axis_y),
    ])
    from scipy.spatial import ConvexHull, cKDTree
    if cam_2d.shape[0] < 8:
        empty["reason"] = "camera_inliers_too_few"
        return empty
    cam_hull = ConvexHull(cam_2d)
    cam_corners = cam_inliers[cam_hull.vertices]
    lid_corners = np.zeros((0, 3))
    if lid_near.shape[0] >= 8:
        lid_2d = np.column_stack([
            (lid_near - origin).dot(axis_x),
            (lid_near - origin).dot(axis_y),
        ])
        lid_hull = ConvexHull(lid_2d)
        lid_corners = lid_near[lid_hull.vertices]
    residual = None
    paired = []
    if len(lid_corners):
        tree = cKDTree(lid_corners)
        dist, idx = tree.query(cam_corners, k=1)
        residual = float(np.min(dist))
        for i, d in enumerate(dist):
            paired.append({
                "camera_xyz": [float(v) for v in cam_corners[i]],
                "lidar_xyz": [float(v) for v in lid_corners[int(idx[i])]],
                "residual_m": float(d),
            })
        paired.sort(key=lambda row: row["residual_m"])
    nn_rmse = None
    if lid_near.shape[0] >= 20:
        tree = cKDTree(lid_near)
        nn, _ = tree.query(cam_inliers, k=1)
        nn_rmse = float(np.sqrt(np.mean(nn * nn)))
    plane_gap = None
    if len(lid_near):
        plane_gap = float(np.median(lid_near.dot(normal) - offset))
    z_axis = np.array([0.0, 0.0, 1.0])
    angle = float(np.degrees(np.arccos(np.clip(abs(normal.dot(z_axis)), 0, 1))))
    return {
        "ok": residual is not None,
        "reason": "ok" if residual is not None else "no_lidar_on_table",
        "n_white": int(white.sum()),
        "n_camera_inliers": int(cam_inliers.shape[0]),
        "n_lidar_near": int(lid_near.shape[0]),
        "plane_normal": [float(v) for v in normal],
        "plane_offset": float(offset),
        "plane_tilt_from_z_deg": angle,
        "plane_gap_median_m": plane_gap,
        "nn_rmse_m": nn_rmse,
        "corner_residual_m": residual,
        "corner_pairs": paired[:6],
        "coincide_table_corner": (
            residual is not None and residual <= 0.03
            and (nn_rmse is None or nn_rmse <= 0.04)
            and (plane_gap is None or abs(plane_gap) <= 0.02)
        ),
    }


def _write_frame_clouds(fused_dir, frame_name, cam_pts, cam_rgb, lid_pts):
    cam_pts = np.asarray(cam_pts)
    lid_pts = np.asarray(lid_pts)
    write_ply_xyzrgb(
        os.path.join(fused_dir, "camera_depth_in_%s.ply" % frame_name),
        cam_pts, cam_rgb)
    write_ply_xyz(
        os.path.join(fused_dir, "livox_in_%s.ply" % frame_name), lid_pts)
    if len(cam_pts) and len(lid_pts):
        lid_c = np.tile(np.array([[255, 80, 200]], dtype=np.uint8),
                        (len(lid_pts), 1))
        write_ply_xyzrgb(
            os.path.join(fused_dir, "overlay_in_%s.ply" % frame_name),
            np.concatenate([cam_pts, lid_pts], axis=0),
            np.concatenate([np.asarray(cam_rgb, dtype=np.uint8), lid_c],
                           axis=0))
    elif len(cam_pts):
        write_ply_xyzrgb(
            os.path.join(fused_dir, "overlay_in_%s.ply" % frame_name),
            cam_pts, cam_rgb)


def _deproject_rgb(depth_mm, rgb, frame, stride):
    depth = np.asarray(depth_mm)
    color = np.asarray(rgb)
    h, w = depth.shape[:2]
    stride = max(1, int(stride))
    uu, vu = np.meshgrid(np.arange(0, w, stride), np.arange(0, h, stride))
    uu = uu.ravel()
    vu = vu.ravel()
    pts, n = deproject_selected(depth, uu, vu, frame)
    if n <= 0:
        return np.zeros((0, 3), dtype=np.float64), np.zeros((0, 3), dtype=np.uint8)
    # deproject_selected drops z==0; rebuild the kept pixel list the same way
    z_mm = depth[vu, uu].astype(np.float32)
    keep = np.isfinite(z_mm) & (z_mm > 0.0)
    uu_k = uu[keep][:n]
    vu_k = vu[keep][:n]
    cols = color[vu_k, uu_k]
    return np.asarray(pts[:n], dtype=np.float64), np.asarray(cols, dtype=np.uint8)


def _subsample(points, colors, cap):
    pts = np.asarray(points)
    if pts.shape[0] <= cap:
        return pts, None if colors is None else np.asarray(colors)
    rng = np.random.RandomState(0)
    idx = np.sort(rng.choice(pts.shape[0], size=int(cap), replace=False))
    if colors is None:
        return pts[idx], None
    return pts[idx], np.asarray(colors)[idx]


_HTML = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8"/>
<title>Mid-360 vs D555 calib dump</title>
<script src="https://cdn.plot.ly/plotly-2.27.0.min.js"></script>
<style>
 body { font-family: sans-serif; margin: 12px; background: #111; color: #eee; }
 h1 { font-size: 18px; }
 .meta { color: #bbb; font-size: 13px; white-space: pre-wrap; }
 .row { display: flex; gap: 12px; flex-wrap: wrap; }
 .card { background: #1c1c1c; padding: 8px; border-radius: 6px; }
 img { max-width: 640px; background: #000; }
</style>
</head>
<body>
<h1>Mid-360 / D555 点云（当前 bag TF，mounter 系）</h1>
<div class="meta" id="meta"></div>
<div class="row">
  <div class="card"><img id="preview" alt="color"/></div>
  <div class="card" style="flex:1;min-width:480px">
    <div id="xyz" style="height:520px"></div>
  </div>
</div>
<script>
const DATA = __DATA__;
document.getElementById("meta").textContent = DATA.meta || "";
if (DATA.preview_jpeg_b64) {
  document.getElementById("preview").src =
    "data:image/jpeg;base64," + DATA.preview_jpeg_b64;
}
const traces = [];
if (DATA.camera_xyz && DATA.camera_xyz.length) {
  traces.push({
    type: "scatter3d", mode: "markers", name: "D555",
    x: DATA.camera_xyz.map(p => p[0]),
    y: DATA.camera_xyz.map(p => p[1]),
    z: DATA.camera_xyz.map(p => p[2]),
    marker: {size: 2, color: DATA.camera_rgb || "#6cf"}
  });
}
if (DATA.lidar_xyz && DATA.lidar_xyz.length) {
  traces.push({
    type: "scatter3d", mode: "markers", name: "Mid-360",
    x: DATA.lidar_xyz.map(p => p[0]),
    y: DATA.lidar_xyz.map(p => p[1]),
    z: DATA.lidar_xyz.map(p => p[2]),
    marker: {size: 2, color: "#f6c"}
  });
}
Plotly.newPlot("xyz", traces, {
  paper_bgcolor: "#111", plot_bgcolor: "#111",
  scene: {
    xaxis: {title: "x (mounter)"},
    yaxis: {title: "y"},
    zaxis: {title: "z"},
    aspectmode: "data"
  },
  margin: {t: 10, l: 0, r: 0, b: 0},
  legend: {font: {color: "#eee"}}
}, {responsive: true});
</script>
</body>
</html>
"""


def export_lidar_camera_calib(bag_path, out_dir, mounter_frame=MOUNTER_FRAME,
                              join_tolerance_ms=30.0, lidar_slop_ms=80.0,
                              camera_stride=2, html_cap=8000,
                              source_iter=None, apply_xacro=True,
                              extra_frames=("elfin_base_link", "world")):
    """Dump RGB-D, CustomMsg lidar, TF, and mounter-frame overlay clouds."""
    scan = scan_bag(bag_path)
    color_topic, depth_topic = select_image_topics(scan)
    mcap_path = find_mcap_file(bag_path)
    stream = source_iter or (lambda topics: iter_bag_messages(
        mcap_path, topics=topics))
    os.makedirs(out_dir, exist_ok=True)
    color_dir = os.path.join(out_dir, "color")
    depth_dir = os.path.join(out_dir, "depth")
    lidar_dir = os.path.join(out_dir, "lidar")
    fused_dir = os.path.join(out_dir, "fused")
    for path in (color_dir, depth_dir, lidar_dir, fused_dir):
        os.makedirs(path, exist_ok=True)

    topics = [color_topic, depth_topic, COLOR_INFO_TOPIC, DEPTH_INFO_TOPIC,
              LIDAR_TOPIC, TF_TOPIC, TF_STATIC_TOPIC, JOINT_TOPIC, TCP_TOPIC]
    color_entries, depth_entries = [], []
    info_first = {}
    tf_buffer = BagTfBuffer(max_dt_ns=200_000_000)
    tf_static_rows = []
    joint0 = None
    tcp0 = None
    lidar_rows = []
    for rec in stream(topics):
        if rec.topic == color_topic:
            color_entries.append((rec.header_stamp_ns, rec.log_time_ns, None))
        elif rec.topic == depth_topic:
            depth_entries.append((rec.header_stamp_ns, rec.log_time_ns, None))
        elif rec.topic in (COLOR_INFO_TOPIC, DEPTH_INFO_TOPIC):
            if rec.topic not in info_first:
                info_first[rec.topic] = rec.message
        elif rec.topic == TF_STATIC_TOPIC:
            tf_buffer.add_tf_message(rec.message, static=True)
            for stamped in rec.message.transforms:
                tf_static_rows.append(_tf_dict(stamped))
        elif rec.topic == TF_TOPIC:
            tf_buffer.add_tf_message(rec.message, static=False)
        elif rec.topic == JOINT_TOPIC and joint0 is None:
            joint0 = {
                "stamp_ns": rec.header_stamp_ns,
                "name": list(rec.message.name),
                "position": [float(v) for v in rec.message.position],
            }
        elif rec.topic == TCP_TOPIC and tcp0 is None:
            p = rec.message.pose.position
            tcp0 = {
                "stamp_ns": rec.header_stamp_ns,
                "frame_id": rec.message.header.frame_id,
                "xyz": [float(p.x), float(p.y), float(p.z)],
            }
        elif rec.topic == LIDAR_TOPIC:
            pts = decode_lidar_scan(rec.message)
            frame_id = str(getattr(rec.message, "frame_id", "")
                           or getattr(getattr(rec.message, "header", None),
                                      "frame_id", "") or LIVOX_FRAME)
            lidar_rows.append({
                "stamp_ns": rec.header_stamp_ns,
                "frame_id": frame_id,
                "points": pts,
                "n": 0 if pts is None else int(len(pts)),
            })

    xacro_used = None
    if apply_xacro:
        xacro_used = apply_current_xacro_mounts(tf_buffer)

    color_sorted, _ = dedupe_stamped_entries(color_entries)
    depth_sorted, _ = dedupe_stamped_entries(depth_entries)
    plan = plan_frame_join(
        [s for s, _l, _p in color_sorted],
        [s for s, _l, _p in depth_sorted],
        tolerance_ns=int(join_tolerance_ms * NS_PER_MS))
    wanted_color = {p.stamp_ns: p for p in plan.pairs}
    wanted_depth = {p.depth_stamp_ns for p in plan.pairs}

    info_msg = info_first.get(COLOR_INFO_TOPIC) or info_first.get(
        DEPTH_INFO_TOPIC)
    if info_msg is None:
        raise ValueError("bag has no camera_info")
    cam = camera_info_frame_from_msg(info_msg)
    optical_frame = str(info_msg.header.frame_id or "")

    pending_color, pending_depth = {}, {}
    camera_clouds = []

    def _flush():
        for stamp, pair in list(wanted_color.items()):
            if pair.stamp_ns not in pending_color:
                continue
            if pair.depth_stamp_ns not in pending_depth:
                continue
            rgb = pending_color.pop(pair.stamp_ns)
            depth = pending_depth.pop(pair.depth_stamp_ns)
            wanted_color.pop(stamp, None)
            if rgb is None or depth is None:
                continue
            idx = len(camera_clouds)
            import cv2
            cv2.imwrite(os.path.join(color_dir, "%06d.jpg" % idx),
                        np.ascontiguousarray(rgb[:, :, ::-1]))
            d16 = np.asarray(depth)
            if d16.dtype != np.uint16:
                d16 = np.clip(d16, 0, 65535).astype(np.uint16)
            cv2.imwrite(os.path.join(depth_dir, "%06d.png" % idx), d16)
            pts_opt, cols = _deproject_rgb(depth, rgb, cam, camera_stride)
            mat = tf_buffer.lookup_matrix(
                mounter_frame, optical_frame, pair.stamp_ns)
            pts_m = None if mat is None else apply_matrix(mat, pts_opt)
            camera_clouds.append({
                "index": idx,
                "stamp_ns": int(pair.stamp_ns),
                "n_optical": int(len(pts_opt)),
                "tf_ok": mat is not None,
                "rgb": rgb,
                "points_optical": pts_opt,
                "colors": cols,
                "points_mounter": pts_m,
            })

    for rec in stream([color_topic, depth_topic]):
        if rec.topic == color_topic and rec.header_stamp_ns in wanted_color:
            pending_color[rec.header_stamp_ns] = decode_color_message(
                rec.message)
        elif rec.topic == depth_topic and rec.header_stamp_ns in wanted_depth:
            pending_depth[rec.header_stamp_ns] = decode_depth_message(
                rec.message)
        _flush()

    lidar_stamps = [row["stamp_ns"] for row in lidar_rows]
    lidar_mounter_chunks = []
    lidar_index = []
    for i, row in enumerate(lidar_rows):
        pts = row["points"]
        xyz = np.zeros((0, 3), dtype=np.float64)
        if pts is not None:
            xyz = np.column_stack([pts["x"], pts["y"], pts["z"]]).astype(
                np.float64)
            write_ply_xyz(os.path.join(lidar_dir, "%06d.ply" % i), xyz)
            np.save(os.path.join(lidar_dir, "%06d.npy" % i), pts)
        src = row["frame_id"] or LIVOX_FRAME
        mat = tf_buffer.lookup_matrix(mounter_frame, src, row["stamp_ns"])
        xyz_m = None if mat is None else apply_matrix(mat, xyz)
        if xyz_m is not None and len(xyz_m):
            lidar_mounter_chunks.append(xyz_m)
        lidar_index.append({
            "index": i,
            "stamp_ns": int(row["stamp_ns"]),
            "frame_id": src,
            "n": int(len(xyz)),
            "tf_ok": mat is not None,
        })

    lidar_mounter = (np.concatenate(lidar_mounter_chunks, axis=0)
                     if lidar_mounter_chunks else
                     np.zeros((0, 3), dtype=np.float64))
    write_ply_xyz(os.path.join(fused_dir, "lidar_in_mounter.ply"),
                  lidar_mounter)

    # Representative camera cloud: middle joined frame with a TF.
    cam_ok = [c for c in camera_clouds if c["tf_ok"] and c["points_mounter"] is not None]
    rep = cam_ok[len(cam_ok) // 2] if cam_ok else None
    if rep is not None:
        write_ply_xyzrgb(
            os.path.join(fused_dir, "camera_in_mounter.ply"),
            rep["points_mounter"], rep["colors"])
        cam_m = np.asarray(rep["points_mounter"])
        cam_c = np.asarray(rep["colors"])
        lid_c = np.tile(np.array([[255, 80, 200]], dtype=np.uint8),
                        (len(lidar_mounter), 1))
        overlay_pts = np.concatenate([cam_m, lidar_mounter], axis=0) \
            if len(lidar_mounter) else cam_m
        overlay_rgb = np.concatenate([cam_c, lid_c], axis=0) \
            if len(lidar_mounter) else cam_c
        write_ply_xyzrgb(
            os.path.join(fused_dir, "overlay_in_mounter.ply"),
            overlay_pts, overlay_rgb)
        import cv2
        ok, buf = cv2.imencode(".jpg", np.ascontiguousarray(
            rep["rgb"][:, :, ::-1]), [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        preview_b64 = __import__("base64").b64encode(buf).decode("ascii") if ok else ""
    else:
        cam_m = np.zeros((0, 3))
        cam_c = np.zeros((0, 3), dtype=np.uint8)
        preview_b64 = ""

    extra_dumps = {}
    corner_by_frame = {}

    def _lidar_in_frame(frame):
        chunks = []
        for row in lidar_rows:
            pts = row["points"]
            if pts is None:
                continue
            xyz = np.column_stack(
                [pts["x"], pts["y"], pts["z"]]).astype(np.float64)
            src = row["frame_id"] or LIVOX_FRAME
            mat = tf_buffer.lookup_matrix(frame, src, row["stamp_ns"])
            if mat is None:
                continue
            chunks.append(apply_matrix(mat, xyz))
        if not chunks:
            return np.zeros((0, 3), dtype=np.float64)
        return np.concatenate(chunks, axis=0)

    for frame_name in extra_frames:
        lid_f = _lidar_in_frame(frame_name)
        cam_f = np.zeros((0, 3), dtype=np.float64)
        cols_f = np.zeros((0, 3), dtype=np.uint8)
        tf_ok = False
        if rep is not None:
            mat = tf_buffer.lookup_matrix(
                frame_name, optical_frame, rep["stamp_ns"])
            tf_ok = mat is not None
            if mat is not None:
                cam_f = apply_matrix(mat, rep["points_optical"])
                cols_f = np.asarray(rep["colors"])
        _write_frame_clouds(fused_dir, frame_name, cam_f, cols_f, lid_f)
        check = check_table_corner(cam_f, cols_f, lid_f) if tf_ok else {
            "ok": False, "reason": "missing_tf",
            "corner_residual_m": None,
        }
        extra_dumps[frame_name] = {
            "n_camera": int(len(cam_f)),
            "n_lidar": int(len(lid_f)),
            "tf_ok": tf_ok,
            "camera_ply": "fused/camera_depth_in_%s.ply" % frame_name,
            "livox_ply": "fused/livox_in_%s.ply" % frame_name,
            "overlay_ply": "fused/overlay_in_%s.ply" % frame_name,
        }
        corner_by_frame[frame_name] = check

    _write_json(os.path.join(out_dir, "xacro_used.json"), xacro_used or {})
    _write_json(os.path.join(out_dir, "corner_check.json"), corner_by_frame)

    html_frame = "elfin_base_link" if "elfin_base_link" in extra_dumps else (
        extra_frames[0] if extra_frames else mounter_frame)
    html_cam, html_lid = cam_m, lidar_mounter
    html_cols = cam_c
    if html_frame in extra_dumps and extra_dumps[html_frame]["tf_ok"] and rep is not None:
        mat = tf_buffer.lookup_matrix(
            html_frame, optical_frame, rep["stamp_ns"])
        if mat is not None:
            html_cam = apply_matrix(mat, rep["points_optical"])
            html_cols = np.asarray(rep["colors"])
            html_lid = _lidar_in_frame(html_frame)

    pairs = []
    slop = int(lidar_slop_ms * NS_PER_MS)
    for crow in camera_clouds:
        hit = nearest_stamp(lidar_stamps, crow["stamp_ns"], slop)
        item = {
            "camera_index": crow["index"],
            "camera_stamp_ns": crow["stamp_ns"],
            "camera_n": crow["n_optical"],
            "camera_tf_ok": crow["tf_ok"],
            "lidar_index": None,
            "lidar_stamp_ns": None,
            "dt_ms": None,
        }
        if hit is not None:
            idx, dt = hit
            item["lidar_index"] = int(idx)
            item["lidar_stamp_ns"] = int(lidar_stamps[idx])
            item["dt_ms"] = float(dt) / 1e6
        pairs.append(item)

    def _edge(parent, child):
        for row in tf_static_rows:
            if row["parent"] == parent and row["child"] == child:
                return row
        return None

    tf_focus = {
        "adjust": _edge(*_ADJUST_EDGE),
        "fixed": [{"edge": "%s -> %s" % (a, b), "tf": _edge(a, b)}
                  for a, b in _FIXED_EDGES],
        "optical_to_mounter": None,
        "livox_to_mounter": None,
    }
    stamp_rep = rep["stamp_ns"] if rep is not None else (
        camera_clouds[0]["stamp_ns"] if camera_clouds else 0)
    mat_opt = tf_buffer.lookup_matrix(
        mounter_frame, optical_frame, stamp_rep)
    mat_livox = tf_buffer.lookup_matrix(
        mounter_frame, LIVOX_FRAME, stamp_rep)
    if mat_opt is not None:
        tf_focus["optical_to_mounter"] = mat_opt.tolist()
    if mat_livox is not None:
        tf_focus["livox_to_mounter"] = mat_livox.tolist()

    _write_json(os.path.join(out_dir, "tf_static.json"), tf_static_rows)
    _write_json(os.path.join(out_dir, "tf_focus.json"), tf_focus)
    _write_json(os.path.join(out_dir, "camera_info.json"), {
        "frame_id": optical_frame,
        "width": cam.width, "height": cam.height,
        "fx": cam.fx, "fy": cam.fy, "cx": cam.cx, "cy": cam.cy,
        "distortion_model": cam.distortion_model,
        "distortion_coeffs": list(cam.distortion_coeffs),
    })
    _write_json(os.path.join(out_dir, "joints.json"), joint0 or {})
    _write_json(os.path.join(out_dir, "tcp.json"), tcp0 or {})
    _write_json(os.path.join(out_dir, "lidar_index.json"), lidar_index)
    _write_json(os.path.join(out_dir, "pairs.json"), pairs)

    cam_html, cam_html_rgb = _subsample(
        html_cam, html_cols, html_cap) if len(html_cam) else (np.zeros((0, 3)), None)
    lid_html, _ = _subsample(html_lid, None, html_cap)
    html_rgb = None
    if cam_html_rgb is not None and len(cam_html_rgb):
        html_rgb = ["rgb(%d,%d,%d)" % (int(r), int(g), int(b))
                    for r, g, b in cam_html_rgb]
    adjust = tf_focus["adjust"] or {}
    base_check = corner_by_frame.get("elfin_base_link") or {}
    world_check = corner_by_frame.get("world") or {}
    meta_lines = [
        "bag: %s" % os.path.abspath(scan.bag_path),
        "overlay frame: %s   xacro mounts: %s" % (
            html_frame, "yes" if xacro_used else "bag TF"),
        "optical: %s   livox: %s" % (optical_frame, LIVOX_FRAME),
        "camera frames: %d   lidar scans: %d" % (
            len(camera_clouds), len(lidar_rows)),
        "mid360_mount xyz: %s" % ((xacro_used or {}).get("mid360_mount_xyz")),
        "elfin_base_link corner residual: %s m  coincide: %s" % (
            base_check.get("corner_residual_m"),
            base_check.get("coincide_table_corner")),
        "world corner residual: %s m  coincide: %s" % (
            world_check.get("corner_residual_m"),
            world_check.get("coincide_table_corner")),
        "PLY: fused/camera_depth_in_elfin_base_link.ply + livox_in_elfin_base_link.ply",
    ]
    html = _HTML.replace("__DATA__", json.dumps({
        "meta": "\n".join(meta_lines),
        "preview_jpeg_b64": preview_b64,
        "camera_xyz": cam_html.tolist() if len(cam_html) else [],
        "camera_rgb": html_rgb,
        "lidar_xyz": lid_html.tolist() if len(lid_html) else [],
    }))
    with open(os.path.join(out_dir, "overlay.html"), "w",
              encoding="utf-8") as handle:
        handle.write(html)

    summary = {
        "bag": os.path.abspath(scan.bag_path),
        "mcap": os.path.abspath(scan.mcap_path),
        "out_dir": os.path.abspath(out_dir),
        "mounter_frame": mounter_frame,
        "optical_frame": optical_frame,
        "n_camera": len(camera_clouds),
        "n_lidar": len(lidar_rows),
        "n_lidar_mounter": int(len(lidar_mounter)),
        "n_paired": sum(1 for p in pairs if p["lidar_index"] is not None),
        "adjust_edge": "%s -> %s" % _ADJUST_EDGE,
        "adjust_xyz": adjust.get("xyz"),
        "adjust_xyzw": adjust.get("xyzw"),
        "preview": "overlay.html",
        "overlay_ply": "fused/overlay_in_%s.ply" % html_frame,
        "apply_xacro": bool(xacro_used),
        "xacro": xacro_used,
        "extra_frames": extra_dumps,
        "corner_check": corner_by_frame,
    }
    _write_json(os.path.join(out_dir, "summary.json"), summary)

    md = [
        "# %s — Mid-360 / D555 with current xacro TF" % os.path.basename(
            os.path.normpath(scan.bag_path)),
        "",
        "- Camera frames: `%d`  Lidar scans: `%d`" % (
            len(camera_clouds), len(lidar_rows)),
        "- Optical frame: `%s`" % optical_frame,
        "- Xacro mounts applied: `%s`" % bool(xacro_used),
        "- `elfin_base_link` corner residual: `%s` m coincide `%s`" % (
            base_check.get("corner_residual_m"),
            base_check.get("coincide_table_corner")),
        "- `world` corner residual: `%s` m coincide `%s`" % (
            world_check.get("corner_residual_m"),
            world_check.get("coincide_table_corner")),
        "- PLY: `fused/camera_depth_in_elfin_base_link.ply`, `fused/livox_in_elfin_base_link.ply`",
        "- Overlay: `overlay.html`",
        "",
    ]
    with open(os.path.join(out_dir, "INDEX.md"), "w", encoding="utf-8") as handle:
        handle.write("\n".join(md) + "\n")
    return summary
