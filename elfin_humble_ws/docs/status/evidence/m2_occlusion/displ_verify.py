import os
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from sensor_msgs_py import point_cloud2 as pc2
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import json

rclpy.init()
node = Node("displ_verify")
qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
got = {}
node.create_subscription(PointCloud2, "/luggage/preprocessed/camera/depth/points",
                         lambda m: got.__setitem__('cloud', m), qos)
gt = {}
tqos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                  durability=DurabilityPolicy.TRANSIENT_LOCAL)
node.create_subscription(String, "/luggage/perception/size_eval/spawned",
                         lambda m: gt.__setitem__('gt', m.data), tqos)
buf = Buffer()
TransformListener(buf, node, spin_thread=False)

end = time.time() + 20
while time.time() < end and ('cloud' not in got or 'gt' not in got):
    rclpy.spin_once(node, timeout_sec=0.1)
if 'cloud' not in got:
    print("NO CLOUD"); os._exit(1)
gt_data = json.loads(gt['gt']) if 'gt' in got else {}
print("GT: %s size %.3f x %.3f x %.3f, yaw %.3f, model %s" % (
    "", gt_data.get('width',0), gt_data.get('depth',0), gt_data.get('height',0),
    gt_data.get('yaw',0), gt_data.get('model_name','?')))
# box center world: pickup source (-1, 0), z = 0.86 + h/2
h = gt_data.get('height', 0.255)
top_z = 0.86 + h
center_gt = np.array([-1.0, 0.0, 0.86 + h/2])
print("GT 箱顶 z = %.3f, 箱中心 = (-1.000, 0.000, %.3f)" % (top_z, 0.86+h/2))

m = got['cloud']
stamp = rclpy.time.Time.from_msg(m.header.stamp)
# 等 TF 到位
_t = None
_dl = time.time() + 3
while _t is None and time.time() < _dl:
    rclpy.spin_once(node, timeout_sec=0.05)
    try:
        _t = buf.lookup_transform("world", "camera_depth_optical_frame", stamp)
    except Exception:
        pass
if _t is None:
    # fallback: latest
    try:
        _t = buf.lookup_transform("world", "camera_depth_optical_frame", rclpy.time.Time())
    except Exception as e:
        print("TF FAIL", e); os._exit(1)
tr, rot = _t.transform.translation, _t.transform.rotation
qx,qy,qz,qw = rot.x, rot.y, rot.z, rot.w
R = np.array([
    [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
    [2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
    [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)]])
print("相机 TF 位置: (%.3f, %.3f, %.3f)" % (tr.x, tr.y, tr.z))

arr = np.array(list(pc2.read_points(m, field_names=("x","y","z"), skip_nans=True)))
pts = np.stack([arr['x'],arr['y'],arr['z']],axis=1).astype(np.float64)
pts = pts[np.isfinite(pts).all(axis=1)]
w = pts.dot(R.T) + np.array([tr.x, tr.y, tr.z])

# 箱顶带
band = np.abs(w[:,2] - top_z) < 0.02
sel = w[band]
print("\n预处理点云世界系: 总点 %d, 箱顶带(|z-%.3f|<0.02): %d 点" % (len(w), top_z, band.sum()))
if band.sum() > 100:
    cx, cy = np.median(sel[:,0]), np.median(sel[:,1])
    dxy = np.sqrt((cx+1.0)**2 + (cy-0.0)**2)
    print("箱顶重建中心: (%.3f, %.3f)  vs GT (-1.000, 0.000)" % (cx, cy))
    print("横向误差: %.3f m  (3cm 门槛%s)" % (dxy, " 过" if dxy<0.03 else " 不过"))
    print("箱顶 xy 范围: x %.2f..%.2f, y %.2f..%.2f" % (
        sel[:,0].min(), sel[:,0].max(), sel[:,1].min(), sel[:,1].max()))
# 平台带对照
plat = np.abs(w[:,2] - 0.86) < 0.015
if plat.sum() > 100:
    ps = w[plat]
    print("平台带(z=0.86)中心: (%.3f, %.3f)  (取货平台真值中心 -1.000, 0.000)" % (
        np.median(ps[:,0]), np.median(ps[:,1])))
os._exit(0)
