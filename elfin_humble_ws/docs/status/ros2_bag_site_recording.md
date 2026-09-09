# 现场 ros2 bag 录制 — 现状与问题

日期：2026-09-08 / 09。主机 ThinkPad T14p Gen 3，ROS 2 Jazzy，`ROS_DOMAIN_ID=7`。
包不进 git（约 10 GB / 两分钟）。代码在 `deployment_ws/src/elfin_trajectory_executor`。

## 要录什么

| 源 | 话题 | 备注 |
|---|---|---|
| CPS | `/joint_states`（rad、rad/s）、`/elfin/tcp_pose`、`/elfin/cps_telemetry` | 示教器遥控时只读，不上电、不使能 |
| Mid-360 | `/livox/lidar`、`/livox/imu` | PointCloud2 + IMU |
| D555 PoE | `/camera/d555/*`（color / depth / aligned depth / 点云 / camera_info / extrinsics） | 可选固件原生 `/realsense/*` |
| 不录（默认） | `/tf`、`/tf_static`、`/robot_description` | `start_scene:=false`，避免零位关节和场景 TF 污染 |

## 代码（已落地，未在「干净只读 CPS」路径上跑通第一包）

| 路径 | 作用 |
|---|---|
| `elfin_trajectory_executor/cps_telemetry_node.py` | `connect(monitor_only=True)`，50 Hz `refresh()` |
| `elfin_trajectory_executor/cps_parse.py` | CPS 字符串列表 → float；速度差分兜底 |
| `elfin_trajectory_executor/huayan_interface.py` | 只读：`HRIF_Connect` + `Connect2Box`；读 `ReadActACS` / `ReadActJointVel` / `ReadActPos`。全量 connect 仍会 Electrify/Enable |
| `launch/cps_telemetry.launch.py` | 只开 CPS 遥测 |
| `launch/d555_rgbd.launch.py` | apt librealsense：节点 `LD_LIBRARY_PATH` 前置 `/lib/x86_64-linux-gnu` |
| `launch/record_site.launch.py` | 组合 CPS + Mid-360 + D555 + `ros2 bag record`；默认 `start_scene:=false` |
| `config/bag_qos_overrides.yaml` | 相机/雷达 Best Effort |
| `luggage_description/launch/mid360.launch.py` | 为 driver 注入 Livox SDK `LD_LIBRARY_PATH` |

CPS 同时只允许 **一个** TCP 客户端。`cps_telemetry` 与 `jazzy_real.launch.py` 互斥。

示教器 jog 时应用只读路径：SDK 查询的是控制器实际 ACS，不抢运动权。同日 HB-3 已用 `HRIF_Connect` + `ReadActACS` 在三个示教器姿态读到非零关节角（`docs/status/evidence/d555_bringup/README.md`）。`ReadActJointVel` 在 jog 中是否非零 **下次验证**；代码在读失败时用角度差分。

## 下次怎么录（验证用）

同一 shell，不要混 Humble overlay 与 Jazzy 的反向 source：

```bash
source /opt/ros/jazzy/setup.bash
source /home/adamliao/work/RoboticArm/deployment_ws/livox_ws/env.sh
source /home/adamliao/work/RoboticArm/elfin_humble_ws/install/setup.bash
source /home/adamliao/work/RoboticArm/deployment_ws/install/setup.bash
export ROS_DOMAIN_ID=7
export PYTHONPATH=/home/adamliao/work/RoboticArm/third_party/huayan_python_sdk:${PYTHONPATH}

# 确认没有旧 executor / 第二份 Livox
pkill -f livox_ros_driver2_node || true
# 不要同时开 jazzy_real.launch.py 或 scene.launch.py

ros2 launch elfin_trajectory_executor record_site.launch.py \
  start_scene:=false use_rviz:=false
```

先确认关节非零再按 bag：

```bash
ros2 topic echo /joint_states --once
```

停录：`pkill -INT -f 'ros2 bag record'`（只停 bag）。网卡：`enp0s31f6` MTU 9000；Livox 主机 `192.168.1.5`；D555 主机 `192.168.11.70`。

边 SDK 运动边录：`start_cps:=false start_executor:=true`（会 Electrify/Enable，**不要**和示教器同时用）。

## 第一包（2026-09-08 21:02–21:04）

路径：`/home/adamliao/robotarm_bags/record_site_20260908_210234`  
时长 107.6 s，9.9 GiB，51221 条，`metadata.yaml` 已封口。传输层丢 1 条。

当时 **没有** 走 `cps_telemetry`。从 9 月 2 日起挂着的旧 `jazzy_real` 占着 CPS；空闲不 `refresh()`，`/joint_states` 100 Hz 但是 **10757 条全 0**。为避免 `disconnect()` 下电，没有杀掉该进程再开遥测。

| 话题 | 条数 | 结论 |
|---|---|---|
| `/livox/lidar` | 1074 | ~10 Hz，约 2e4 点/帧，`livox_frame` |
| `/livox/imu` | 20823 | ~193 Hz |
| `/camera/d555/color/image_raw` | 1535 | 640×360 `rgb8` ~15 Hz |
| `/camera/d555/aligned_depth_to_color/image_raw` | 1532 | 640×360 `16UC1` |
| `/camera/d555/depth/color/points` | 1536 | RGBD 几何可用 |
| `/camera/d555/rgbd` | **无** | 当时驱动 `enable_rgbd:=false` |
| `/camera/d555/aligned_depth_to_color1/image_raw` | 0 | 第二路对齐深度空，可忽略 |
| `/joint_states` | 10757 | **全 0，本包不可用于 FK** |
| `/elfin/*` | 无 | 未开 `cps_telemetry` |

拷到 `192.168.4.93:~/work/` 未完成：`adamliao@192.168.4.93` 公钥被拒。

## 现场还踩过的坑

1. **ROS_DOMAIN_ID**：Livox 曾在 domain 0，CPS/D555 在 7。一包必须同一 domain；录第一包前已把 Mid-360 重拉到 7。
2. **两份 `livox_ros_driver2_node`**：抢 `192.168.1.5` 端口。开录前 `pkill` 再启一份。
3. **`scene.launch.py` 零位关节** 会和真实 `/joint_states` 对打。录传感器包不要开 scene TF。
4. **D555**：Jazzy 的 `realsense2_camera` 要 `LD_LIBRARY_PATH=/lib/x86_64-linux-gnu`；MTU 不足会丢 RGBD。
5. **旧 executor 进程**：改过 `refresh()` 的代码必须 **重启节点** 才生效。第一包用的是 Sep-2 进程。
6. 包体积大约 **100 MB/s**（点云 + RGBD），短录即可。

## 未闭环

| 项 | 状态 |
|---|---|
| 示教器 jog 时 `cps_telemetry` 的 `/joint_states` 非零角 | 方案成立；**下次上机验证** |
| jog 时 `velocity` / `ReadActJointVel` 非零 | API 有；**未在运动中验证** |
| `record_site.launch.py` 从零拉齐 CPS+Livox+D555+bag | 写了；第一包是「已有驱动 + 单独 bag」 |
| D555 `enable_rgbd:=true` 的 `/camera/d555/rgbd` | launch 已开；第一包驱动未开 |
| 新 executor 空闲 `refresh()` | 代码有；需重启后看 |
| scp 到 192.168.4.93 | 等对端 `authorized_keys` |

## 不要做的事

- 示教器遥控时开 `jazzy_real`（Electrify / GrpEnable / 可能 Reset）。
- `cps_telemetry` 和 `jazzy_real` 同时连 `192.168.0.10:10003`。
- `scene.launch.py`（零关节）和真实关节一起录。
- 把 Humble `/opt/ros/humble` 和 Jazzy 叠在同一 shell。
