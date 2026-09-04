import os
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from sensor_msgs_py import point_cloud2 as pc2
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import time

rclpy.init()
node = Node("pixel_audit")
qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
got = {}
def cb_pts(m): got['pts'] = m
def cb_depth(m): got['depth'] = m
def cb_info(m): got['info'] = m
node.create_subscription(PointCloud2, "/camera/depth/points", cb_pts, qos)
node.create_subscription(Image, "/camera/depth/image_raw", cb_depth, qos)
node.create_subscription(CameraInfo, "/camera/depth/camera_info", cb_info, qos)
buf = Buffer()
TransformListener(buf, node, spin_thread=False)

end = time.time() + 10
need = {'pts', 'depth', 'info'}
while time.time() < end and not need <= set(got):
    rclpy.spin_once(node, timeout_sec=0.1)

pts_msg, depth_msg, info_msg = got.get('pts'), got.get('depth'), got.get('info')
print("organized cloud: %dx%d frame=%s | depth image: %dx%d enc=%s frame=%s | K fx=%.2f cx=%.1f cy=%.1f frame=%s" % (
    pts_msg.width, pts_msg.height, pts_msg.header.frame_id,
    depth_msg.width, depth_msg.height, depth_msg.encoding, depth_msg.header.frame_id,
    info_msg.k[0], info_msg.k[2], info_msg.k[5], info_msg.header.frame_id))

# structured read preserving pixel order
arr = np.array(list(pc2.read_points(pts_msg, field_names=("x","y","z"), skip_nans=False)))
pts = np.stack([arr['x'],arr['y'],arr['z']],axis=1).astype(np.float64)
W, H = pts_msg.width, pts_msg.height
print("cloud pixel count:", pts.shape[0], "expected:", W*H)

finite = np.isfinite(pts).all(axis=1)
print("finite pixels: %d / %d" % (finite.sum(), pts.shape[0]))

# world transform via camera_link (empirically validated frame)
t = buf.lookup_transform("world", "camera_link", rclpy.time.Time(),
                         timeout=rclpy.duration.Duration(seconds=1.0))
tr, rot = t.transform.translation, t.transform.rotation
qx,qy,qz,qw = rot.x, rot.y, rot.z, rot.w
R = np.array([
    [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
    [2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
    [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)]])
world = pts.dot(R.T) + np.array([tr.x, tr.y, tr.z])

# ground-truth targets in world
targets = {
  "box_top_center (-1.0, 0, 1.12)": np.array([-1.0, 0.0, 1.12]),
  "platform_center (-1.0, 0, 0.86)": np.array([-1.0, 0.0, 0.861]),
}
fx, cx, cy = info_msg.k[0], info_msg.k[2], info_msg.k[5]
for name, tgt in targets.items():
    d = np.linalg.norm(world[finite] - tgt, axis=1)
    idx = np.where(finite)[0][np.argmin(d)]
    u, v = idx % W, idx // W
    print("%s -> nearest cloud pixel (u=%d, v=%d), world err %.3f m, cloud_cam=(%.2f,%.2f,%.2f)" % (
        name, u, v, d.min(), *pts[idx]))

# predictions for box center rel-camera (cam_link coords computed: fwd=0.71, y=+0.20, z=0)
fwd, y_cl, z_cl = 0.71, 0.20, 0.0
print("prediction if image = optical w.r.t. labeled optical frame (x_opt=-y_cl): u=%.0f" % (cx + fx*(-y_cl)/fwd))
print("prediction if image u along +y_cl:                                  u=%.0f" % (cx + fx*(+y_cl)/fwd))

# depth image cross-check at the found box pixel
darr = np.frombuffer(bytes(depth_msg.data), dtype=np.float32).reshape(H, W) \
    if depth_msg.encoding in ("32FC1", "32F") else None
if darr is not None:
    # find box pixel again
    d = np.linalg.norm(world[finite] - targets["box_top_center (-1.0, 0, 1.12)"], axis=1)
    idx = np.where(finite)[0][np.argmin(d)]
    u, v = idx % W, idx // W
    print("depth image at (u=%d,v=%d): %.3f m | cloud forward dist x_cl at same pixel: %.3f m" % (
        u, v, darr[v, u], pts[idx][0]))
    print("depth image center (320,240): %.3f | corner (0,0): %.3f | corner (639,479): %.3f" % (
        darr[240,320], darr[0,0], darr[479,639]))
os._exit(0)
