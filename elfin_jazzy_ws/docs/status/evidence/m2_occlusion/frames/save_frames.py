import os
import struct, zlib
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import time

OUT = "/home/adamliao/work/elfin_humble_ws/docs/status/evidence/m2_occlusion/frames"

def write_png(path, rgb_or_gray):
    """Pure-python PNG (cv_bridge broken by numpy 2.x). Accepts HxWx3 uint8 or HxW uint8."""
    arr = np.asarray(rgb_or_gray)
    if arr.ndim == 2:
        arr = arr[:, :, None]
    h, w, c = arr.shape
    color_type = 2 if c == 3 else 0
    raw = b"".join(b"\x00" + arr[y].tobytes() for y in range(h))
    def chunk(tag, data):
        c = tag + data
        return struct.pack(">I", len(data)) + c + struct.pack(">I", zlib.crc32(c) & 0xffffffff)
    png = (b"\x89PNG\r\n\x1a\n"
           + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, color_type, 0, 0, 0))
           + chunk(b"IDAT", zlib.compress(raw, 6))
           + chunk(b"IEND", b""))
    with open(path, "wb") as f:
        f.write(png)

rclpy.init()
node = Node("frame_saver")
qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
buf = Buffer()
TransformListener(buf, node, spin_thread=False)

# wait for TF
_tf = None
_dl = time.time() + 5
while _tf is None and time.time() < _dl:
    rclpy.spin_once(node, timeout_sec=0.1)
    try:
        _tf = buf.lookup_transform("world", "camera_depth_optical_frame", rclpy.time.Time())
    except Exception:
        pass
if _tf is None:
    print("NO TF"); os._exit(1)
tr, rot = _tf.transform.translation, _tf.transform.rotation
qx,qy,qz,qw = rot.x, rot.y, rot.z, rot.w
R = np.array([
    [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
    [2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
    [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)]])
T = np.array([tr.x, tr.y, tr.z])

colors, depths, infos = [], [], []
def on_color(m):
    if len(colors) < 40: colors.append(m)
def on_depth(m):
    if len(depths) < 40: depths.append(m)
def on_info(m):
    if not infos: infos.append(m)
node.create_subscription(Image, "/camera/color/image_raw", on_color, qos)
node.create_subscription(Image, "/camera/depth/image_raw", on_depth, qos)
node.create_subscription(CameraInfo, "/camera/depth/camera_info", on_info, qos)

end = time.time() + 25
while time.time() < end and len(colors) < 24 and len(depths) < 24:
    rclpy.spin_once(node, timeout_sec=0.1)
print("collected: %d color, %d depth" % (len(colors), len(depths)))

# pair color+depth by nearest stamp
pairs = []
used = set()
for d in depths:
    ds = d.header.stamp.sec + d.header.stamp.nanosec*1e-9
    best, bd = None, 1e9
    for i, c in enumerate(colors):
        if i in used: continue
        cs = c.header.stamp.sec + c.header.stamp.nanosec*1e-9
        if abs(cs-ds) < bd:
            bd, best = abs(cs-ds), i
    if best is not None and bd < 0.2:
        pairs.append((colors[best], d))
        used.add(best)
    if len(pairs) >= 12:
        break

summary = []
summary.append("frame pairing: color+depth from same rgbd render, matched by header stamp")
summary.append("classification counts use TF world<-camera_depth_optical_frame at frame stamp")
summary.append("B=box-top(1.115±0.02) P=platform(0.86±0.02) M=mystery(1.13-1.45) n=panel(1.45-1.8) g=ground(<0.5)")
summary.append("")
for k, (c, d) in enumerate(pairs):
    # depth -> float meters
    if d.encoding == "32FC1":
        dep = np.frombuffer(bytes(d.data), dtype=np.float32).reshape(d.height, d.width)
    elif d.encoding == "16UC1":
        dep = np.frombuffer(bytes(d.data), dtype=np.uint16).reshape(d.height, d.width).astype(np.float32) * 0.001
    else:
        summary.append("frame %02d: unsupported depth encoding %s" % (k, d.encoding)); continue
    # color (encoding strings are lowercase in ROS: rgb8/bgr8/rgba8/mono8)
    enc = c.encoding.lower()
    if enc in ("rgb8", "r8g8b8"):
        rgb = np.frombuffer(bytes(c.data), dtype=np.uint8).reshape(c.height, c.width, 3)
    elif enc in ("bgr8", "b8g8r8"):
        rgb = np.frombuffer(bytes(c.data), dtype=np.uint8).reshape(c.height, c.width, 3)[:, :, ::-1]
    elif enc == "rgba8":
        rgb = np.frombuffer(bytes(c.data), dtype=np.uint8).reshape(c.height, c.width, 4)[:, :, :3]
    elif enc == "mono8":
        mono = np.frombuffer(bytes(c.data), dtype=np.uint8).reshape(c.height, c.width)
        rgb = np.stack([mono]*3, axis=-1)
    else:
        rgb = None
    # classification via back-projection with K (optical convention)
    K = infos[0].k if infos else None
    fx, cx, fy, cy = K[0], K[2], K[4], K[5]
    H, W = dep.shape
    vv, uu = np.mgrid[0:H, 0:W]
    fin = np.isfinite(dep)
    z = dep
    x = (uu - cx) / fx * z
    y = (vv - cy) / fy * z
    pts = np.stack([x, y, z], axis=-1).astype(np.float64)
    sel = pts[fin]
    w = sel.dot(R.T) + T
    zw = w[:, 2]
    counts = {
        'B': int((np.abs(zw-1.115)<0.02).sum()),
        'P': int((np.abs(zw-0.860)<0.02).sum()),
        'M': int(((zw>=1.13)&(zw<1.45)).sum()),
        'n': int(((zw>=1.45)&(zw<1.8)).sum()),
        'g': int((zw<0.5).sum()),
    }
    ds = d.header.stamp.sec + d.header.stamp.nanosec*1e-9
    base = "frame_%02d" % k
    write_png(os.path.join(OUT, base + "_color.png"), rgb if rgb is not None else np.zeros((H,W), np.uint8))
    # depth visualization: 0-2.5m -> 0-255 (grayscale), inf/nan -> 0
    vis = np.clip(dep, 0, 2.5) / 2.5 * 255
    vis = np.nan_to_num(vis, nan=0.0, posinf=0.0, neginf=0.0).astype(np.uint8)
    write_png(os.path.join(OUT, base + "_depth.png"), vis)
    np.save(os.path.join(OUT, base + "_depth.npy"), dep.astype(np.float32))
    line = ("%s stamp=%.3f  B=%d P=%d M=%d n=%d g=%d  finite=%d/%d" %
            (base, ds, counts['B'], counts['P'], counts['M'], counts['n'], counts['g'], fin.sum(), H*W))
    summary.append(line)
    print(line)

with open(os.path.join(OUT, "summary.txt"), "w") as f:
    f.write("\n".join(summary) + "\n")
if infos:
    with open(os.path.join(OUT, "camera_info.txt"), "w") as f:
        f.write("K=[%s]\nframe_id=%s\n" % (", ".join(str(v) for v in infos[0].k), infos[0].header.frame_id))
os._exit(0)
