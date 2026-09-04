# EEF + Livox + D555 snapshot — 2026-09-02 18:35 +08

Working bring-up on native ROS 2 Jazzy (`ROS_DOMAIN_ID=7`). Not a joint
calibration; tape/CAD still pending for Livox yaw and scene TF.

## Frames (parent → child)

| Joint | xyz (m) | rpy (rad) | Note |
|---|---|---|---|
| `elfin_end_link` → `suction_panel` | `0.0006 0 0` | `-1.5645 0 π` | suction_flange_origin.xacro |
| `suction_panel` → `eef_mount_adapter` | `0.0168 -0.0156 0.0702` | `1.57708 -0.00004 -3.13531` | D435 mount GUI |
| `eef_mount_adapter` → `camera_link` | `0.013 0.097 -0.021` | `0.03770 1.36345 1.57080` | D435 mount GUI |
| `camera_link` → `d555_link` | `0 0 0` | `0 0 0` | D555 body; same extrinsics as D435 |
| `eef_mount_adapter` → `mid360_mount_frame` | `0.022 0.103 0.038` | `0 π/2 π/2` | +Y pad of arm_realsense_v1.3 |
| `mid360_mount_frame` → `livox_frame` | `0 0 0.047` | `0 0 0` | handbook optical centre |
| `livox_frame` → `livox_imu_frame` | `0.011 0.02329 -0.04412` | `0 0 0` | handbook IMU lever arm |

D555 intra-camera optical TF (`d555_link` → `d555_depth_optical_frame`,
rpy `-π/2 0 -π/2`) comes from `realsense2_camera`, not this URDF.

## Site network (this cell, 2026-09-02)

- NIC `enp0s31f6`, MTU 9000. Addresses used: `192.168.0.11/24` (arm),
  `192.168.1.5/24` + `192.168.1.50/24` (Livox host), `192.168.11.70/24` (D555 host).
- Arm CPS: `192.168.0.10:10003`.
- Mid-360S: `192.168.1.120` (see `MID360s_config.json` in this folder; host `192.168.1.5`).
- D555 PoE: `192.168.11.55`, SN `419222302385`, FW `7.56.37776.6014`.
  Use official librealsense **2.58.4** (`LD_LIBRARY_PATH=/lib/x86_64-linux-gnu` first).
  Stable stream: `640x360@15` RGB + depth. Default `896x504@30` dropped the DDS device.

`scene_tf.yaml.example` is still simulation geometry. Do not treat it as surveyed.
