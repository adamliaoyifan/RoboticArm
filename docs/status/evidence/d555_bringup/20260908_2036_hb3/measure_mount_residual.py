#!/usr/bin/env python3
"""HB-3: capture D555 + Mid-360 at one static pose and fit floor planes in elfin_base.

Does not command the arm. Joints come from CPS ReadActACS, not /joint_states
(the live executor still publishes zeros). EE-to-sensor TF is taken from the
live tree (fixed joints). Base-to-EE is URDF FK at the CPS joint vector.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

JOINT_NAMES = [
    "elfin_joint1", "elfin_joint2", "elfin_joint3",
    "elfin_joint4", "elfin_joint5", "elfin_joint6",
]


def local_tag(elem):
    return elem.tag.split("}")[-1]


def rpy_matrix(rpy):
    # URDF fixed-axis RPY: Rz(yaw) * Ry(pitch) * Rx(roll)
    return Rotation.from_euler("xyz", rpy).as_matrix()


def origin_T(joint_elem):
    xyz = np.zeros(3)
    rpy = np.zeros(3)
    for child in joint_elem:
        if local_tag(child) == "origin":
            if child.get("xyz"):
                xyz = np.array([float(x) for x in child.get("xyz").split()], dtype=np.float64)
            if child.get("rpy"):
                rpy = np.array([float(x) for x in child.get("rpy").split()], dtype=np.float64)
    T = np.eye(4)
    T[:3, :3] = rpy_matrix(rpy)
    T[:3, 3] = xyz
    return T


def motion_T(joint_elem, qmap):
    jtype = joint_elem.get("type", "fixed")
    if jtype in ("fixed", "floating", "planar"):
        return np.eye(4)
    axis = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    for child in joint_elem:
        if local_tag(child) == "axis" and child.get("xyz"):
            axis = np.array([float(x) for x in child.get("xyz").split()], dtype=np.float64)
            n = np.linalg.norm(axis)
            if n > 0:
                axis = axis / n
    q = float(qmap.get(joint_elem.get("name"), 0.0))
    T = np.eye(4)
    if jtype in ("revolute", "continuous"):
        T[:3, :3] = Rotation.from_rotvec(axis * q).as_matrix()
    elif jtype == "prismatic":
        T[:3, 3] = axis * q
    return T


def parse_urdf_chain(xml_text):
    root = ET.fromstring(xml_text)
    joint_by_child = {}
    parent_of = {}
    for elem in root.iter():
        if local_tag(elem) != "joint":
            continue
        parent = child = None
        for sub in elem:
            tag = local_tag(sub)
            if tag == "parent":
                parent = sub.get("link")
            elif tag == "child":
                child = sub.get("link")
        if parent and child:
            joint_by_child[child] = elem
            parent_of[child] = parent
    return parent_of, joint_by_child


def fk_T(parent_of, joint_by_child, base, target, qmap):
    chain = []
    link = target
    seen = set()
    while link != base:
        if link in seen:
            raise RuntimeError("cycle walking %s -> %s at %s" % (target, base, link))
        seen.add(link)
        if link not in joint_by_child:
            raise RuntimeError("no parent joint for link %s (wanted base %s)" % (link, base))
        chain.append(joint_by_child[link])
        link = parent_of[link]
    T = np.eye(4)
    for joint in reversed(chain):
        T = T @ origin_T(joint) @ motion_T(joint, qmap)
    return T


def T_to_xyz_rpy(T):
    xyz = T[:3, 3].tolist()
    rpy = Rotation.from_matrix(T[:3, :3]).as_euler("xyz").tolist()
    return xyz, rpy


def stamp_to_T(st):
    t = st.transform.translation
    q = st.transform.rotation
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    T[:3, 3] = [t.x, t.y, t.z]
    return T


def apply_T(T, pts):
    return pts @ T[:3, :3].T + T[:3, 3]


def xyz_from_cloud(msg):
    fields = {f.name: f for f in msg.fields}
    if not {"x", "y", "z"} <= set(fields):
        raise RuntimeError("cloud missing xyz: %s" % list(fields))
    step = int(msg.point_step)
    n = int(msg.width) * int(msg.height)
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    dtype = np.dtype({
        "names": ["x", "y", "z"],
        "formats": ["<f4", "<f4", "<f4"],
        "offsets": [int(fields["x"].offset), int(fields["y"].offset),
                    int(fields["z"].offset)],
        "itemsize": step,
    })
    arr = np.ndarray(n, dtype=dtype, buffer=raw[: n * step])
    pts = np.stack([arr["x"], arr["y"], arr["z"]], axis=1).astype(np.float64)
    ok = np.isfinite(pts).all(axis=1)
    ok &= np.linalg.norm(pts, axis=1) > 0.05
    return pts[ok], str(msg.header.frame_id), {
        "width": int(msg.width),
        "height": int(msg.height),
        "point_step": step,
        "fields": list(fields),
        "stamp_sec": int(msg.header.stamp.sec),
        "stamp_nsec": int(msg.header.stamp.nanosec),
        "n_valid": int(ok.sum()),
        "n_raw": n,
    }


def ransac_plane(pts, iters=600, thresh=0.02, rng=0):
    rng = np.random.default_rng(rng)
    n = len(pts)
    if n < 50:
        return None
    best_count = 0
    best = None
    for _ in range(iters):
        i = rng.choice(n, 3, replace=False)
        p0, p1, p2 = pts[i]
        nvec = np.cross(p1 - p0, p2 - p0)
        ln = np.linalg.norm(nvec)
        if ln < 1e-9:
            continue
        nvec = nvec / ln
        if nvec[2] < 0:
            nvec = -nvec
        d = -float(nvec.dot(p0))
        inl = np.abs(pts @ nvec + d) < thresh
        c = int(inl.sum())
        if c > best_count:
            best_count = c
            best = (nvec, d, inl)
    if best is None or best_count < 50:
        return None
    nvec, d, inl = best
    pin = pts[inl]
    mean = pin.mean(axis=0)
    _, _, vh = np.linalg.svd(pin - mean, full_matrices=False)
    nvec = vh[-1]
    if nvec[2] < 0:
        nvec = -nvec
    d = -float(nvec.dot(mean))
    inl = np.abs(pts @ nvec + d) < thresh
    return {
        "normal": nvec,
        "d": d,
        "inlier_mask": inl,
        "inliers": int(inl.sum()),
        "centroid": pts[inl].mean(axis=0) if inl.any() else mean,
    }


def fit_floor_in_base(d555_base, livox_base):
    """Fit floor planes on the D555 XY footprint so both sensors share a patch."""
    zs = d555_base[:, 2]
    z_lo = float(np.percentile(zs, 8))
    floor_band = (d555_base[:, 2] >= z_lo - 0.04) & (d555_base[:, 2] <= z_lo + 0.06)
    d_floor_seed = d555_base[floor_band]
    if len(d_floor_seed) < 200:
        d_floor_seed = d555_base
    d_fit = ransac_plane(d_floor_seed, thresh=0.018, rng=1)
    if d_fit is None:
        raise RuntimeError("D555 floor plane fit failed (n=%d)" % len(d_floor_seed))
    d_in = d_floor_seed[d_fit["inlier_mask"]]
    xy_lo = np.percentile(d_in[:, :2], 8, axis=0) - 0.05
    xy_hi = np.percentile(d_in[:, :2], 92, axis=0) + 0.05
    z_c = float(d_fit["centroid"][2])
    liv_mask = (
        (livox_base[:, 0] >= xy_lo[0]) & (livox_base[:, 0] <= xy_hi[0])
        & (livox_base[:, 1] >= xy_lo[1]) & (livox_base[:, 1] <= xy_hi[1])
        & (livox_base[:, 2] >= z_c - 0.12) & (livox_base[:, 2] <= z_c + 0.12)
    )
    liv_patch = livox_base[liv_mask]
    if len(liv_patch) < 80:
        raise RuntimeError("Livox common-patch too small: %d" % len(liv_patch))
    l_fit = ransac_plane(liv_patch, thresh=0.025, rng=2)
    if l_fit is None:
        raise RuntimeError("Livox floor plane fit failed")
    c = d_fit["centroid"]
    n1, d1 = d_fit["normal"], d_fit["d"]
    n2, d2 = l_fit["normal"], l_fit["d"]
    offset_m = abs(float(n2.dot(c) + d2))
    cosang = float(np.clip(n1.dot(n2), -1.0, 1.0))
    angle_deg = math.degrees(math.acos(cosang))
    return {
        "d555_plane": {
            "normal": n1.tolist(),
            "d": float(d1),
            "inliers": int(d_fit["inliers"]),
            "centroid": c.tolist(),
            "seed_points": int(len(d_floor_seed)),
        },
        "livox_plane": {
            "normal": n2.tolist(),
            "d": float(d2),
            "inliers": int(l_fit["inliers"]),
            "centroid": l_fit["centroid"].tolist(),
            "patch_points": int(len(liv_patch)),
        },
        "common_xy_lo": xy_lo.tolist(),
        "common_xy_hi": xy_hi.tolist(),
        "d555_z_percentile8": z_lo,
        "offset_m": offset_m,
        "offset_mm": offset_m * 1000.0,
        "angle_deg": angle_deg,
    }


def livox_smear_stats(a, b, voxel=0.03):
    """Static two-frame occupancy. Large union growth is scan+Decay, not deskew."""
    def voxels(pts):
        q = np.floor(pts / voxel).astype(np.int32)
        return set(map(tuple, q))
    va, vb = voxels(a), voxels(b)
    inter = va & vb
    union = va | vb
    # subsample NN from a to b
    rng = np.random.default_rng(0)
    take = min(2000, len(a), len(b))
    sa = a[rng.choice(len(a), take, replace=False)]
    sb = b[rng.choice(len(b), take, replace=False)]
    # chunked min distance
    dmin = []
    for i in range(0, len(sa), 200):
        chunk = sa[i:i + 200]
        diff = chunk[:, None, :] - sb[None, :, :]
        dmin.append(np.linalg.norm(diff, axis=2).min(axis=1))
    dmin = np.concatenate(dmin)
    return {
        "voxel_m": voxel,
        "voxels_a": len(va),
        "voxels_b": len(vb),
        "voxels_intersection": len(inter),
        "voxels_union": len(union),
        "jaccard": (len(inter) / len(union)) if union else 0.0,
        "nn_median_m": float(np.median(dmin)),
        "nn_p95_m": float(np.percentile(dmin, 95)),
        "n_a": int(len(a)),
        "n_b": int(len(b)),
    }


def read_cps_joints(n_samples=2, pause_s=0.2):
    sys.path.insert(0, "/home/adamliao/work/RoboticArm/third_party/huayan_python_sdk")
    from CPS import CPSClient
    c = CPSClient()
    n = c.HRIF_Connect(0, "192.168.0.10", 10003)
    if n != 0:
        raise RuntimeError("HRIF_Connect failed nRet=%s" % n)
    try:
        samples = []
        for i in range(n_samples):
            result = []
            nret = c.HRIF_ReadActJointPos(0, 0, result)
            if nret != 0 or len(result) < 6:
                raise RuntimeError("ReadActACS nRet=%s result=%s" % (nret, result))
            samples.append([float(x) for x in result[:6]])
            if i + 1 < n_samples:
                time.sleep(pause_s)
        st = []
        c.HRIF_ReadRobotState(0, 0, st)
        fsm = []
        c.HRIF_ReadCurFSM(0, 0, fsm)
        deg = samples[-1]
        deltas = []
        for a, b in zip(samples[0], samples[-1]):
            deltas.append(abs(a - b))
        return {
            "deg": deg,
            "rad": [math.radians(d) for d in deg],
            "samples_deg": samples,
            "max_joint_delta_deg": max(deltas) if deltas else 0.0,
            "robot_state": st,
            "fsm": fsm,
        }
    finally:
        try:
            c.HRIF_DisConnect(0)
        except Exception:
            pass


def _qos_pair():
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    sensor_be = QoSProfile(
        depth=5,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        durability=DurabilityPolicy.VOLATILE,
    )
    reliable = QoSProfile(
        depth=5,
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        durability=DurabilityPolicy.VOLATILE,
    )
    return sensor_be, reliable


def _spin_until(node, predicate, timeout_s):
    import rclpy
    t_end = time.time() + timeout_s
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
    return False


def grab_d555_and_tf(timeout_s=20.0):
    """Domain 7: D555, colour, robot_description, EE-to-sensor TF."""
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image, PointCloud2
    from tf2_ros import Buffer, TransformListener
    from rcl_interfaces.srv import GetParameters

    os.environ["ROS_DOMAIN_ID"] = "7"
    rclpy.init()
    node = Node("hb3_grab_d555")
    buf = Buffer()
    TransformListener(buf, node)
    sensor_be, reliable = _qos_pair()
    box = {"d555": None, "color": None, "robot_description": None}

    def on_d555(msg):
        if box["d555"] is None:
            box["d555"] = msg

    def on_color(msg):
        if box["color"] is None:
            box["color"] = msg

    node.create_subscription(
        PointCloud2, "/camera/d555/depth/color/points", on_d555, reliable)
    node.create_subscription(
        PointCloud2, "/camera/d555/depth/color/points", on_d555, sensor_be)
    node.create_subscription(Image, "/camera/d555/color/image_raw", on_color, reliable)

    client = node.create_client(GetParameters, "/robot_state_publisher/get_parameters")
    fut = None
    if client.wait_for_service(timeout_sec=5.0):
        req = GetParameters.Request()
        req.names = ["robot_description"]
        fut = client.call_async(req)

    tf_end = tf_d555 = tf_livox = None

    def ready():
        nonlocal tf_end, tf_d555, tf_livox
        if fut is not None and fut.done() and box["robot_description"] is None:
            box["robot_description"] = fut.result().values[0].string_value
        try:
            tf_end = buf.lookup_transform(
                "elfin_base", "elfin_end_link", rclpy.time.Time())
            tf_d555 = buf.lookup_transform(
                "elfin_end_link", "d555_depth_optical_frame", rclpy.time.Time())
            tf_livox = buf.lookup_transform(
                "elfin_end_link", "livox_frame", rclpy.time.Time())
        except Exception:
            pass
        return (box["d555"] is not None and box["robot_description"]
                and tf_d555 is not None and tf_livox is not None
                and tf_end is not None)

    _spin_until(node, ready, timeout_s)
    node.destroy_node()
    rclpy.shutdown()
    if box["d555"] is None:
        raise RuntimeError("no D555 PointCloud2 on domain 7")
    if not box["robot_description"]:
        raise RuntimeError("no robot_description on domain 7")
    if tf_d555 is None or tf_livox is None:
        raise RuntimeError("missing EE-to-sensor TF on domain 7")
    return box, tf_end, tf_d555, tf_livox


def grab_livox(timeout_s=12.0):
    """Live Mid-360 is on the default domain (no ROS_DOMAIN_ID), not domain 7."""
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2

    clouds = []

    def on_livox(msg):
        if len(clouds) < 2:
            clouds.append(msg)

    for domain in ("", "7"):
        if domain:
            os.environ["ROS_DOMAIN_ID"] = domain
        else:
            os.environ.pop("ROS_DOMAIN_ID", None)
        rclpy.init()
        node = Node("hb3_grab_livox")
        sensor_be, reliable = _qos_pair()
        node.create_subscription(PointCloud2, "/livox/lidar", on_livox, reliable)
        node.create_subscription(PointCloud2, "/livox/lidar", on_livox, sensor_be)
        _spin_until(node, lambda: len(clouds) >= 2, timeout_s)
        node.destroy_node()
        rclpy.shutdown()
        if len(clouds) >= 2:
            return clouds, domain if domain else "0"
    raise RuntimeError("need 2 Livox clouds, got %d" % len(clouds))


def grab_ros():
    box, tf_end, tf_d555, tf_livox = grab_d555_and_tf()
    livox, livox_domain = grab_livox()
    box["livox"] = livox
    box["livox_domain"] = livox_domain
    return box, tf_end, tf_d555, tf_livox


def save_color_png(path, msg):
    arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
    if msg.encoding == "rgb8":
        bgr = arr[:, :, ::-1]
    elif msg.encoding == "bgr8":
        bgr = arr
    else:
        return
    import cv2
    cv2.imwrite(path, bgr)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--pose-id", required=True)
    args = parser.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    box, tf_end_live, tf_d555, tf_livox = grab_ros()
    joints = read_cps_joints()

    qmap = dict(zip(JOINT_NAMES, joints["rad"]))
    qzero = {n: 0.0 for n in JOINT_NAMES}
    parent_of, joint_by_child = parse_urdf_chain(box["robot_description"])
    T_base_end_fk = fk_T(parent_of, joint_by_child, "elfin_base", "elfin_end_link", qmap)
    T_base_end_fk0 = fk_T(parent_of, joint_by_child, "elfin_base", "elfin_end_link", qzero)
    T_base_end_live = stamp_to_T(tf_end_live)  # untrusted: zero /joint_states
    T_end_d555 = stamp_to_T(tf_d555)
    T_end_livox = stamp_to_T(tf_livox)
    T_base_d555 = T_base_end_fk @ T_end_d555
    T_base_livox = T_base_end_fk @ T_end_livox

    d555_pts, d555_frame, d555_meta = xyz_from_cloud(box["d555"])
    liv_a, liv_frame, liv_meta_a = xyz_from_cloud(box["livox"][0])
    liv_b, _, liv_meta_b = xyz_from_cloud(box["livox"][1])

    d555_base = apply_T(T_base_d555, d555_pts)
    liv_a_base = apply_T(T_base_livox, liv_a)
    liv_b_base = apply_T(T_base_livox, liv_b)

    residual = fit_floor_in_base(d555_base, liv_a_base)
    smear = livox_smear_stats(liv_a, liv_b)

    np.save(os.path.join(args.out_dir, "d555_xyz_optical.npy"), d555_pts)
    np.save(os.path.join(args.out_dir, "d555_xyz_base.npy"), d555_base)
    np.save(os.path.join(args.out_dir, "livox_a_xyz_optical.npy"), liv_a)
    np.save(os.path.join(args.out_dir, "livox_a_xyz_base.npy"), liv_a_base)
    np.save(os.path.join(args.out_dir, "livox_b_xyz_optical.npy"), liv_b)
    if box["color"] is not None:
        save_color_png(os.path.join(args.out_dir, "color.png"), box["color"])

    fk0_xyz, fk0_rpy = T_to_xyz_rpy(T_base_end_fk0)
    live_xyz, live_rpy = T_to_xyz_rpy(T_base_end_live)
    fk_xyz, fk_rpy = T_to_xyz_rpy(T_base_end_fk)
    end_d555_xyz, end_d555_rpy = T_to_xyz_rpy(T_end_d555)
    end_livox_xyz, end_livox_rpy = T_to_xyz_rpy(T_end_livox)

    dq = joints["max_joint_delta_deg"]
    out = {
        "pose_id": args.pose_id,
        "joints_deg": joints["deg"],
        "joints_rad": joints["rad"],
        "joints_samples_deg": joints["samples_deg"],
        "max_joint_delta_deg": dq,
        "cps_fsm": joints["fsm"],
        "cps_robot_state": joints["robot_state"],
        "d555_cloud": {"frame_id": d555_frame, **d555_meta},
        "livox_cloud_a": {"frame_id": liv_frame, **liv_meta_a},
        "livox_cloud_b": liv_meta_b,
        "livox_ros_domain_id": box.get("livox_domain"),
        "fk_check_zero_vs_live_tf": {
            "fk_q0_xyz": fk0_xyz,
            "fk_q0_rpy": fk0_rpy,
            "live_tf_xyz": live_xyz,
            "live_tf_rpy": live_rpy,
            "xyz_err_m": float(np.linalg.norm(np.array(fk0_xyz) - np.array(live_xyz))),
            "note": "live TF uses executor zeros; should match FK(q=0)",
        },
        "T_elfin_base_elfin_end_link_fk": {"xyz": fk_xyz, "rpy": fk_rpy},
        "T_elfin_end_link_d555_depth_optical": {"xyz": end_d555_xyz, "rpy": end_d555_rpy},
        "T_elfin_end_link_livox_frame": {"xyz": end_livox_xyz, "rpy": end_livox_rpy},
        "gui_mount_eef_adapter_to_camera_link": {
            "xyz": [0.013, 0.097, -0.021],
            "rpy": [0.03770, 1.36345, 1.57080],
        },
        "residual": residual,
        "livox_smear_two_static_frames": smear,
        "joint_source": "CPS HRIF_ReadActACS; /joint_states untrusted zeros",
    }
    with open(os.path.join(args.out_dir, "measure.json"), "w") as handle:
        json.dump(out, handle, indent=2)
    print(json.dumps({
        "pose_id": args.pose_id,
        "offset_mm": residual["offset_mm"],
        "angle_deg": residual["angle_deg"],
        "d555_inliers": residual["d555_plane"]["inliers"],
        "livox_inliers": residual["livox_plane"]["inliers"],
        "fk0_vs_live_err_m": out["fk_check_zero_vs_live_tf"]["xyz_err_m"],
        "max_joint_delta_deg": out["max_joint_delta_deg"],
        "smear_jaccard": smear["jaccard"],
        "smear_nn_median_m": smear["nn_median_m"],
    }, indent=2))


if __name__ == "__main__":
    main()
