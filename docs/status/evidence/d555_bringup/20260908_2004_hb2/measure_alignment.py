#!/usr/bin/env python3
"""HB-2: capture one static D555 snapshot and measure colour-vs-depth edge offset.

Does not command the arm. Writes PNG/npy/json under its output directory.
"""
from __future__ import annotations

import argparse
import json
import os
import struct
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2


# HB-1 live values (also overwritten from the snapshot camera_info).
COLOR_K = dict(fx=323.1775207519531, fy=322.8994140625,
               cx=317.7525939941406, cy=178.02940368652344)
DEPTH_K = dict(fx=321.5054931640625, fy=321.5054931640625,
               cx=318.34478759765625, cy=178.16726684570312)
R_D2C = np.array([
    [0.999984085559845, 0.0010870200349017978, -0.005538229830563068],
    [-0.0010853999992832541, 0.9999993443489075, 0.00029490000451914966],
    [0.005538539960980415, -0.00028889000532217324, 0.999984622001648],
], dtype=np.float64)
T_D2C = np.array([-0.05877445265650749, -6.219466013135388e-5,
                  0.0008159596472978592], dtype=np.float64)


def image_to_array(msg: Image):
    if msg.encoding in ("rgb8", "bgr8"):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3)
        if msg.encoding == "bgr8":
            arr = arr[:, :, ::-1].copy()
        else:
            arr = arr.copy()
        return arr
    if msg.encoding == "16UC1":
        depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(
            msg.height, msg.width)
        return depth.astype(np.float32) / 1000.0
    raise RuntimeError("unsupported encoding %s" % msg.encoding)


def k_from_info(msg: CameraInfo):
    k = msg.k
    return dict(fx=float(k[0]), fy=float(k[4]), cx=float(k[2]), cy=float(k[5]))


def sobel_mag(gray):
    gx = np.zeros_like(gray, dtype=np.float32)
    gy = np.zeros_like(gray, dtype=np.float32)
    gx[:, 1:-1] = gray[:, 2:] - gray[:, :-2]
    gy[1:-1, :] = gray[2:, :] - gray[:-2, :]
    return np.hypot(gx, gy)


def phase_shift(a, b):
    """Signed (du, dv) of b relative to a via phase correlation. +u is right."""
    fa = np.fft.fft2(a)
    fb = np.fft.fft2(b)
    r = fa * np.conj(fb)
    n = np.abs(r)
    n[n == 0] = 1.0
    cross = np.fft.ifft2(r / n)
    mag = np.abs(cross)
    peak = np.unravel_index(int(np.argmax(mag)), mag.shape)
    dv = float(peak[0])
    du = float(peak[1])
    h, w = mag.shape
    if dv > h / 2.0:
        dv -= h
    if du > w / 2.0:
        du -= w
    return du, dv, float(mag[peak] / mag.sum())


def unproject_depth(depth, k):
    h, w = depth.shape
    us, vs = np.meshgrid(np.arange(w, dtype=np.float64),
                         np.arange(h, dtype=np.float64))
    z = depth.astype(np.float64)
    x = (us - k["cx"]) / k["fx"] * z
    y = (vs - k["cy"]) / k["fy"] * z
    return np.stack([x, y, z], axis=-1)


def warp_depth_to_color(depth, depth_k, color_k, R, t, out_hw):
    pts = unproject_depth(depth, depth_k)
    valid = np.isfinite(pts[..., 2]) & (pts[..., 2] > 0.2) & (pts[..., 2] < 6.0)
    p = pts[valid]
    pc = p @ R.T + t
    zc = pc[:, 2]
    ok = zc > 0.05
    pc = pc[ok]
    zc = zc[ok]
    u = color_k["fx"] * pc[:, 0] / zc + color_k["cx"]
    v = color_k["fy"] * pc[:, 1] / zc + color_k["cy"]
    h, w = out_hw
    ui = np.rint(u).astype(np.int32)
    vi = np.rint(v).astype(np.int32)
    inb = (ui >= 0) & (ui < w) & (vi >= 0) & (vi < h)
    warped = np.zeros((h, w), dtype=np.float32)
    hit = np.zeros((h, w), dtype=np.uint8)
    warped[vi[inb], ui[inb]] = zc[inb]
    hit[vi[inb], ui[inb]] = 1
    return warped, hit


def save_png_rgb(path, rgb):
    try:
        import cv2
        cv2.imwrite(path, rgb[:, :, ::-1])
        return
    except ImportError:
        pass
    # PPM fallback
    h, w, _ = rgb.shape
    with open(path.replace(".png", ".ppm"), "wb") as handle:
        handle.write(("P6\n%d %d\n255\n" % (w, h)).encode("ascii"))
        handle.write(np.ascontiguousarray(rgb).tobytes())


def colorize_depth(depth, vmax=2.0):
    z = np.clip(depth, 0, vmax) / vmax
    img = np.zeros(depth.shape + (3,), dtype=np.uint8)
    valid = depth > 0.05
    img[..., 0] = np.clip((1.0 - z) * 255, 0, 255).astype(np.uint8)
    img[..., 2] = np.clip(z * 255, 0, 255).astype(np.uint8)
    img[~valid] = 0
    return img


def overlay(color, depth_edge, color_edge):
    out = color.copy()
    out[color_edge > 0] = (0, 255, 0)
    out[depth_edge > 0] = (255, 0, 0)
    both = (color_edge > 0) & (depth_edge > 0)
    out[both] = (255, 255, 0)
    return out


def rgb_stats_from_cloud(msg: PointCloud2):
    names = [f.name for f in msg.fields]
    if "rgb" not in names:
        return {"has_rgb_field": False}
    off = {f.name: f.offset for f in msg.fields}
    step = msg.point_step
    n = msg.width * msg.height
    buf = bytes(msg.data)
    take = min(n, 20000)
    stride = max(1, n // take)
    rgbs = []
    for i in range(0, n, stride):
        base = i * step + off["rgb"]
        if base + 4 > len(buf):
            break
        packed = struct.unpack_from("<f", buf, base)[0]
        bits = struct.unpack("<I", struct.pack("<f", packed))[0]
        r = (bits >> 16) & 255
        g = (bits >> 8) & 255
        b = bits & 255
        rgbs.append((r, g, b))
    arr = np.array(rgbs, dtype=np.float64)
    return {
        "has_rgb_field": True,
        "n_sampled": int(len(arr)),
        "rgb_mean": [float(x) for x in arr.mean(axis=0)],
        "rgb_std": [float(x) for x in arr.std(axis=0)],
        "unique_approx": int(len(np.unique(arr.astype(np.int16), axis=0))),
        "constant": bool(arr.std() < 1.0),
    }


class Grabber(Node):
    def __init__(self):
        super().__init__("hb2_grab")
        sensor = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        reliable = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.color = self.aligned = self.native = self.points = None
        self.color_info = self.depth_info = self.aligned_info = None
        self.create_subscription(
            Image, "/camera/d555/color/image_raw", self._c, reliable)
        self.create_subscription(
            Image, "/camera/d555/aligned_depth_to_color/image_raw",
            self._a, reliable)
        self.create_subscription(
            Image, "/camera/d555/depth/image_rect_raw", self._n, reliable)
        self.create_subscription(
            PointCloud2, "/camera/d555/depth/color/points", self._p, reliable)
        self.create_subscription(
            CameraInfo, "/camera/d555/color/camera_info", self._ci, reliable)
        self.create_subscription(
            CameraInfo, "/camera/d555/depth/camera_info", self._di, reliable)
        self.create_subscription(
            CameraInfo, "/camera/d555/aligned_depth_to_color/camera_info",
            self._ai, reliable)
        # retry BEST_EFFORT in case RELIABLE misses
        self.create_subscription(
            Image, "/camera/d555/color/image_raw", self._c, sensor)
        self.create_subscription(
            Image, "/camera/d555/aligned_depth_to_color/image_raw",
            self._a, sensor)
        self.create_subscription(
            Image, "/camera/d555/depth/image_rect_raw", self._n, sensor)

    def _c(self, msg):
        self.color = msg

    def _a(self, msg):
        self.aligned = msg

    def _n(self, msg):
        self.native = msg

    def _p(self, msg):
        self.points = msg

    def _ci(self, msg):
        self.color_info = msg

    def _di(self, msg):
        self.depth_info = msg

    def _ai(self, msg):
        self.aligned_info = msg

    def ready(self):
        return self.color is not None and self.aligned is not None and (
            self.native is not None)


def edge_mask(mag, q=92.0):
    thr = np.percentile(mag, q)
    return (mag >= max(thr, 8.0)).astype(np.uint8)


def measure(out_dir, labeled_z):
    os.makedirs(out_dir, exist_ok=True)
    rclpy.init()
    node = Grabber()
    t0 = time.time()
    while time.time() - t0 < 8.0 and not node.ready():
        rclpy.spin_once(node, timeout_sec=0.1)
    if not node.ready():
        raise SystemExit("timeout waiting for color+aligned+native depth")
    # wait a bit more for points
    t1 = time.time()
    while time.time() - t1 < 2.0 and node.points is None:
        rclpy.spin_once(node, timeout_sec=0.1)

    color = image_to_array(node.color)
    aligned = image_to_array(node.aligned)
    native = image_to_array(node.native)
    color_k = k_from_info(node.color_info) if node.color_info else COLOR_K
    depth_k = k_from_info(node.depth_info) if node.depth_info else DEPTH_K
    aligned_k = k_from_info(node.aligned_info) if node.aligned_info else color_k

    np.save(os.path.join(out_dir, "color.npy"), color)
    np.save(os.path.join(out_dir, "aligned_depth_m.npy"), aligned)
    np.save(os.path.join(out_dir, "native_depth_m.npy"), native)
    save_png_rgb(os.path.join(out_dir, "color.png"), color)
    save_png_rgb(os.path.join(out_dir, "aligned_depth_colorize.png"),
                 colorize_depth(aligned, vmax=2.0))
    save_png_rgb(os.path.join(out_dir, "native_depth_colorize.png"),
                 colorize_depth(native, vmax=2.0))

    gray = (0.299 * color[..., 0] + 0.587 * color[..., 1]
            + 0.114 * color[..., 2]).astype(np.float32)
    color_mag = sobel_mag(gray)
    aligned_valid = aligned.copy()
    aligned_valid[aligned < 0.05] = np.nan
    depth_mag = sobel_mag(np.nan_to_num(aligned, nan=0.0) * 1000.0)

    warped, hit = warp_depth_to_color(
        native, depth_k, color_k, R_D2C, T_D2C, color.shape[:2])
    identity_warp, _ = warp_depth_to_color(
        native, depth_k, color_k, np.eye(3), np.zeros(3), color.shape[:2])
    warped_mag = sobel_mag(warped * 1000.0)
    ident_mag = sobel_mag(identity_warp * 1000.0)

    # Restrict correlation to pixels with depth so empty borders do not dominate.
    mask = hit > 0
    a = color_mag * mask
    b = warped_mag * mask
    c = ident_mag * mask
    d = depth_mag * (aligned > 0.05)

    du_ext, dv_ext, p_ext = phase_shift(a, b)
    du_id, dv_id, p_id = phase_shift(a, c)
    du_al, dv_al, p_al = phase_shift(a, d)

    z_med = float(np.nanmedian(aligned[aligned > 0.05])) if np.any(
        aligned > 0.05) else float("nan")
    z_p10 = float(np.nanpercentile(aligned[aligned > 0.05], 10)) if np.any(
        aligned > 0.05) else float("nan")
    z_p90 = float(np.nanpercentile(aligned[aligned > 0.05], 90)) if np.any(
        aligned > 0.05) else float("nan")

    def px_to_m(du, dv, z):
        if not np.isfinite(z) or z <= 0:
            return [None, None]
        return [float(du * z / color_k["fx"]), float(dv * z / color_k["fy"])]

    color_grad_p90 = float(np.percentile(color_mag, 90))
    color_grad_p99 = float(np.percentile(color_mag, 99))
    usable_target = bool(color_grad_p99 > 25.0)

    cloud = rgb_stats_from_cloud(node.points) if node.points is not None else {
        "has_rgb_field": False, "reason": "no PointCloud2 in window"
    }

    color_e = edge_mask(color_mag)
    aligned_e = edge_mask(d)
    warped_e = edge_mask(b)
    save_png_rgb(os.path.join(out_dir, "overlay_aligned_on_color.png"),
                 overlay(color, aligned_e, color_e))
    save_png_rgb(os.path.join(out_dir, "overlay_extrinsic_warp_on_color.png"),
                 overlay(color, warped_e, color_e))

    nom_59 = 0.059
    nom_95 = 0.095
    expected_px_59 = nom_59 / z_med * color_k["fx"] if z_med else None
    expected_px_95 = nom_95 / z_med * color_k["fx"] if z_med else None

    report = {
        "operator_labeled_range_m": labeled_z,
        "median_aligned_depth_m": z_med,
        "aligned_depth_p10_p90_m": [z_p10, z_p90],
        "color_frame": node.color.header.frame_id,
        "aligned_frame": node.aligned.header.frame_id,
        "native_frame": node.native.header.frame_id,
        "color_k": color_k,
        "depth_k": depth_k,
        "aligned_k": aligned_k,
        "color_gradient_p90": color_grad_p90,
        "color_gradient_p99": color_grad_p99,
        "usable_high_contrast_target": usable_target,
        "signed_offset_px": {
            "aligned_depth_on_color": {
                "du": du_al, "dv": dv_al, "peak_frac": p_al,
                "metres": px_to_m(du_al, dv_al, z_med),
            },
            "native_warped_with_hb1_extrinsic": {
                "du": du_ext, "dv": dv_ext, "peak_frac": p_ext,
                "metres": px_to_m(du_ext, dv_ext, z_med),
            },
            "native_warped_identity_extrinsic": {
                "du": du_id, "dv": dv_id, "peak_frac": p_id,
                "metres": px_to_m(du_id, dv_id, z_med),
            },
        },
        "expected_u_px_if_unaligned": {
            "from_59mm_color_offset": expected_px_59,
            "from_95mm_baseline": expected_px_95,
        },
        "pointcloud_rgb": cloud,
        "sign_convention": "+du = depth/warped edge is to the right of colour edge",
    }
    with open(os.path.join(out_dir, "measure.json"), "w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2)
    print(json.dumps(report, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", required=True)
    parser.add_argument("--labeled-z", type=float, default=0.6)
    args = parser.parse_args()
    measure(args.out, args.labeled_z)


if __name__ == "__main__":
    main()
