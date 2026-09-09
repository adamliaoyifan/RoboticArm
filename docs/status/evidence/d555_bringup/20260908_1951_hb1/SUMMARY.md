# HB-1 D555 device fact dump — 2026-09-08 19:51 +08

Read-only. Arm stationary. No URDF/yaml edits. Raw command output lives in
this directory (`topic_info/`, `msgs/`, `hz/`, `tf/`, `versions/`).

Tested commit: `40ab61ca0f6d5237f26091d946c05227bb6fc468` (workspace
`ros2_humble` snapshot). Higher-resolution `896x504@30` was **not**
re-attempted; it previously dropped the DDS device.

## Item 1 — topics, types, QoS

Publisher is `/camera/d555` (`realsense2_camera` 4.58.1). No
`/camera/d555/imu` topic (device IMU is BMI088; gyro/accel were not enabled
in this launch). `ros2 topic info -v` reports History depth as UNKNOWN.

| Topic | Type | Reliability | Durability |
|---|---|---|---|
| `/camera/d555/color/image_raw` | `sensor_msgs/Image` | RELIABLE | TRANSIENT_LOCAL |
| `/camera/d555/color/camera_info` | `sensor_msgs/CameraInfo` | RELIABLE | VOLATILE |
| `/camera/d555/depth/image_rect_raw` | `sensor_msgs/Image` | RELIABLE | TRANSIENT_LOCAL |
| `/camera/d555/depth/camera_info` | `sensor_msgs/CameraInfo` | RELIABLE | VOLATILE |
| `/camera/d555/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | RELIABLE | TRANSIENT_LOCAL |
| `/camera/d555/aligned_depth_to_color/camera_info` | `sensor_msgs/CameraInfo` | RELIABLE | VOLATILE |
| `/camera/d555/depth/color/points` | `sensor_msgs/PointCloud2` | RELIABLE | VOLATILE |
| `/camera/d555/extrinsics/depth_to_color` | `realsense2_camera_msgs/Extrinsics` | RELIABLE | (see dump) |

Also live, not in the nine-item table: `color1` / `aligned_depth_to_color1`
(`enable_color1: true`). IMU: none under `/camera/d555`.

## Item 2 — driver frame tree

`tf2_tools view_frames` → `tf/frames.pdf` and `tf/frames.gv`.

Driver (D400 wrapper convention, body origin = depth / left IR):

```
d555_link
  → d555_depth_frame → d555_depth_optical_frame   (identity xyz, rpy −π/2 0 −π/2)
  → d555_color_frame → d555_color_optical_frame    (xyz ≈ 0, −0.059, 0 m in d555_link)
  → d555_color1_frame / aligned_depth_to_color1_frame
```

URDF still publishes the D435-named chain in parallel:

```
eef_mount_adapter → camera_link → camera_depth_frame / camera_color_frame
camera_link → d555_link   (identity)
```

`tf2_echo camera_link → d555_link`: translation 0, identity rotation.
Body origin of `d555_link` coincides with the depth/left-IR convention, not
the colour imager. Colour is offset by about −59 mm on ROS body Y.

## Item 3 — `/extrinsics/depth_to_color`

From `msgs/depth_to_color.yaml` (rotation nearly I):

```
translation: [-0.05877445, -6.219e-05, 0.00081596]   # metres
```

| Axis | Live | D450 nominal cited in plan | Delta |
|---|---|---|---|
| Message X (optical right) | −58.77 mm | baseline ~59 mm on that axis | 0.23 mm |
| TF `d555_link` → colour optical Y | −59 mm | “about −59 mm on Y” | ~0 mm |

The live stereo baseline matches the D450/D555 ~59 mm colour offset. The
message stores it on translation[0]; the ROS body TF stores it on −Y. Same
physical lever, two conventions.

## Item 4 — live `camera_info` at 640×360 (not sim `fx=337.222`)

Colour (`d555_color_optical_frame`, `plumb_bob`):

- size 640×360
- K: fx=323.1775, fy=322.8994, cx=317.7526, cy=178.0294
- D: [−0.054931, 0.061236, 0.000130, −0.001346, −0.020642]
- R = I; P matches K with Tx=Ty=0

Depth (`d555_depth_optical_frame`, `plumb_bob`):

- size 640×360
- K: fx=fy=321.5055, cx=318.3448, cy=178.1673
- D all zero
- R = I; P matches K

Aligned-depth `camera_info` is a copy of **colour** K/D in
`d555_color_optical_frame`.

Derived pinhole FOV (not datasheet): colour ≈ 89.4° × 58.4°; depth ≈ 89.7° ×
58.5°. Datasheet RGB 90×65 / depth 87×58. Hardware K is **not** a rescale of
sim 640×480 `fx=fy=337.222`.

## Item 5 — `align_depth`

SafeDDS / `rs_launch.py` path: **present and ON**.

```
ros2 param get /camera/d555 align_depth.enable  →  Boolean value is: True
```

Launch used `align_depth.enable:=true`. Aligned topics
`/camera/d555/aligned_depth_to_color/{image_raw,camera_info}` are publishing.

## Item 6 — frame versus K (explicit verdict)

| Product | `frame_id` | K used | Verdict |
|---|---|---|---|
| colour image | `d555_color_optical_frame` | colour K | **consistent** |
| native depth `image_rect_raw` | `d555_depth_optical_frame` | depth K | **consistent** (depth-native) |
| aligned depth `aligned_depth_to_color` | `d555_color_optical_frame` | colour K | **consistent** (colour-aligned) |
| `/depth/color/points` | `d555_depth_optical_frame` | xyz unprojected in depth; `rgb` field present | **consistent as depth-native geometry with colour texture** |

Both a colour-aligned depth image and a depth-native coloured cloud exist at
once. Treating `/camera/d555/depth/color/points` as if it lived in colour
optical with colour K would be the 6–9 cm class of error. The aligned image
is the colour-frame product.

## Item 7 — rates (≥30 s), 640×360@15 ceiling

Last window over ~32 s (`hz/*.txt`):

| Stream | Last window Hz |
|---|---|
| `/camera/d555/color/image_raw` | 14.36 |
| `/camera/d555/depth/image_rect_raw` | 15.28 |
| `/camera/d555/aligned_depth_to_color/image_raw` | 14.40 |
| `/camera/d555/depth/color/points` | 13.92 |

Stable ceiling remains **640×360 at 15 Hz**. Occasional 13.3–15.3 Hz jitter;
no device drop at this profile. Higher `896x504@30` was not repeated.

## Item 8 — versions and ROS distro

| Item | Value |
|---|---|
| Camera | Intel RealSense D555 PoE, SN `419222302385`, product DDS / D500 |
| Firmware | `7.56.37776.6014` |
| Connection | DDS to host `192.168.11.70`, device `192.168.11.55` |
| Official librealsense | `2.58.4-0~realsense.19927` (`LD_LIBRARY_PATH=/lib/x86_64-linux-gnu`) |
| apt `ros-jazzy-librealsense2` | 2.58.1 (present; live path uses 2.58.4) |
| `realsense2_camera` | 4.58.1 (`/opt/ros/jazzy`) |
| OS | Ubuntu 24.04.4 noble |
| ROS distro actually running | **native Jazzy** (`/opt/ros/jazzy`, `ROS_DOMAIN_ID=7`) |

D555 DDS streaming on this host **required Jazzy**. This session did not run
Humble. The Humble source tree is what was pulled; the live graph is Jazzy.

## Item 9 — RViz Color Transformer (operator-given)

2026-09-08 RGBD display was a genuine RGBD render with colour applied, not
FlatColor or Intensity. Recorded as given; not re-investigated.

## Reported, not edited

`realsense_d435.yaml` still has `mount.tune_joints` rpy
`rx=-1.36345, ry=0.03770, rz=1.57080` versus `mount.fixed.rpy`
`0.03770, 1.36345, 1.57080`. Left untouched per plan.

## Commands

Raw: `topics_all.txt`, `topic_info/*.txt`, `msgs/*.yaml`, `hz/*.txt`,
`tf/lookups.txt`, `tf/frames.pdf`, `versions/versions.txt`,
`driver_params.yaml`.
