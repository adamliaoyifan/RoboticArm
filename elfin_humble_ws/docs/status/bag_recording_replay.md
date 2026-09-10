# 两种现场 bag 标准（可回灌改算法）

日期：2026-09-09。主机 Jazzy，`ROS_DOMAIN_ID=7`。包目录 `~/robotarm_bags/`。

两种模式都只录**原始传感器 + 实际关节 + TF**，**不录** `/luggage/*`（preprocessor / detector 输出）。回灌时用当前代码重跑算法，才能对比修改前后。

不要开 `scene.launch.py`（零关节）。两种模式**不能同时**占 CPS（`192.168.0.10:10003` 只允许一个 TCP 客户端）。

Launch：一键脚本 `deployment_ws/scripts/record_site.sh`（内部是 `record_mode:=pendant|real`）。Ctrl+C 停。

```bash
# 示教器，包写到默认 ~/robotarm_bags/record_site_pendant_<时间戳>
/home/adamliao/work/RoboticArm/deployment_ws/scripts/record_site.sh pendant

# 指定目录；目录里再建带名字的包
/home/adamliao/work/RoboticArm/deployment_ws/scripts/record_site.sh pendant \
  -o ~/robotarm_bags/2026-09-09 -n jog_box1

# 真机 FJT（急停旁）
/home/adamliao/work/RoboticArm/deployment_ws/scripts/record_site.sh real \
  -o ~/robotarm_bags/tracking -n fail_blend
```

---

## 公共环境（两种录制、回灌都要）

```bash
source /opt/ros/jazzy/setup.bash
source /home/adamliao/work/RoboticArm/deployment_ws/livox_ws/env.sh
source /home/adamliao/work/RoboticArm/elfin_humble_ws/install/setup.bash
source /home/adamliao/work/RoboticArm/deployment_ws/install/setup.bash
export ROS_DOMAIN_ID=7
export PYTHONPATH=/home/adamliao/work/RoboticArm/third_party/huayan_python_sdk:${PYTHONPATH}
export LD_LIBRARY_PATH=/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH}

# 录制前
sudo ip link set enp0s31f6 mtu 9000
sudo ip addr add 192.168.1.5/24 dev enp0s31f6 2>/dev/null || true
sudo ip addr add 192.168.11.70/24 dev enp0s31f6 2>/dev/null || true
pkill -f livox_ros_driver2_node 2>/dev/null || true
```

不录 D555 点云 / RGBD / infra（体积）。默认 **压缩入库**：彩色 JPEG（`.../image_raw/compressed`）、对齐深度 16-bit PNG（同名 `.../compressed`）。压缩在 `d555_host_stamp` 做；驱动保持 `image_transport/raw`（在驱动上开 compressed 会让本机 `realsense2_camera_node` SIGSEGV）。Mid-360 默认 `xfer_format:=1`（`livox_ros_driver2/CustomMsg`，比 PointCloud2 小）。回灌用 `replay_site.launch.py` 解压/转回 PointCloud2。

这减的是 **本机 ROS DDS + 录包 + CPU**。D555 PoE 上相机仍发原始 RGB8/Depth（librealsense DDS）；Livox UDP 也仍是雷达固有协议。要让大图别再打到机械臂网卡，`record_site.sh` 默认 `ROS_LOCALHOST_ONLY=1`。

---

## 时钟（主机 ROS 系统时间为 master）

现场 **没有** PTP / GPIO 硬件触发。所有可回灌 topic 统一到 **录包主机的 ROS 系统时间**（`use_sim_time=false`）：

| 源 | stamp |
|---|---|
| CPS `/joint_states` `/elfin/*` `/vacuum/io` | 主机 ROS 时间（扣掉 CPS 查询 RTT） |
| RSP `/tf` | 跟随 `/joint_states` |
| Livox `/livox/lidar` `/livox/imu` | 驱动已用主机时钟；lidar stamp 是该圈扫描起点，会比收包早约一圈 |
| D555 彩色 / 对齐深度 / CameraInfo / IMU | `d555_host_stamp`：`enable_sync` 成对后写成 **同一拍** 主机时间。驱动原始 `*_hw` 话题不入库 |

D555 驱动自己的 HARDWARE_CLOCK 会周期性回绕并打 `Hardware clock reset` 日志；入库的压缩图带的是主机 stamp。`/clock_sync/master` 每秒重复声明 master。

这不是多机 PTP 对齐，残余误差是收包抖动 + Livox 扫描时长。回灌仍用 `header.stamp` 对齐关节/相机/雷达。

---

## A. 示教器录制（`record_mode:=pendant`）

**用途：** 示教器 jog，录真实姿态 + 相机/雷达。回灌感知、FK、时间戳、deskew、settle（实际关节轨迹）。没有 FJT，**不能**复盘跟踪误差 / 混合半径。

**会启动：** `cps_telemetry`（只读，不上电）、`scene_hardware`（真 `/joint_states` → TF）、Mid-360、D555、bag。

```bash
/home/adamliao/work/RoboticArm/deployment_ws/scripts/record_site.sh pendant \
  -o ~/robotarm_bags/2026-09-09 -n jog_box1
```

看到 `/joint_states` 非零后再 jog。录 10–60 s 即可。Ctrl+C。

### Topic list（示教器）

| Topic | 类型 | 回灌角色 |
|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | **这一拍** CPS 实际 ACS（rad）+ 速度 + 电流；失败的字段空着，不用上一拍顶上 |
| `/elfin/tcp_pose` | `geometry_msgs/PoseStamped` | 与 ACS 同一包 `ReadActPos` 的实际 TCP |
| `/elfin/cps_telemetry` | `std_msgs/String` JSON | 控制器原值 + `q_cmd_deg` + `sample_ok` + `vel_source` / `qdd_source` |
| `/elfin/cps_rate` | `std_msgs/Float64MultiArray` | `[poll目标Hz, 实际完整快照Hz, 查询耗时ms]` |
| `/elfin/joint_kinematics` | `control_msgs/JointTrajectoryControllerState` | `feedback`=实际，`desired`=CPS 指令角；加速度是差分，不是伺服原值 |
| `/vacuum/io` | `std_msgs/String` JSON | 电箱 DI0/DO0/DO1 这一拍：`suction_ok` / `vacuum_on` / `de_vacuum` |
| `/vacuum/di0` | `std_msgs/Bool` | DI0=1 吸上确认，0 未吸上 |
| `/vacuum/do0` | `std_msgs/Bool` | DO0 抽真空 |
| `/vacuum/do1` | `std_msgs/Bool` | DO1 放开真空 (de_vacuum) |
| `/livox/lidar` | `livox_ros_driver2/CustomMsg` | 雷达（比 PointCloud2 小；回灌转回 PointCloud2） |
| `/livox/imu` | `sensor_msgs/Imu` | Livox IMU |
| `/camera/d555/color/image_raw/compressed` | `sensor_msgs/CompressedImage` | 彩色 JPEG |
| `/camera/d555/color/camera_info` | `sensor_msgs/CameraInfo` | 彩色内参 |
| `/camera/d555/aligned_depth_to_color/image_raw/compressed` | `sensor_msgs/CompressedImage` | 对齐深度 PNG（无损） |
| `/camera/d555/aligned_depth_to_color/camera_info` | `sensor_msgs/CameraInfo` | 深度内参（投影） |
| `/camera/d555/imu` 及 `gyro/` `accel/` | `sensor_msgs/Imu` | D555 BMI088。PoE/DDS 原话题是 `motion/sample`，`d555_host_stamp` 写成主机时间后发到这三条（同一拍） |
| `/clock_sync/master` | `std_msgs/String` JSON | 声明时钟 master=`host_ros_system_time` |
| `/tf` | `tf2_msgs/TFMessage` | 动态 TF（关节 + 相机） |
| `/tf_static` | `tf2_msgs/TFMessage` | 静态 TF / 外参 |

**不录：** `/camera/d555/depth/color/points`、`/camera/d555/rgbd`、infra、`/realsense/*`、`/luggage/*`、`scene.launch` 零关节。

---

## B. 真机自动录制（`record_mode:=real`）

**用途：** `jazzy_real` 执行 `FollowJointTrajectory`。回灌感知（同上）以及 **期望轨迹 vs 实际关节**、混合、延迟、settle 误判。

**会启动：** `trajectory_executor`（上电使能）、`scene_hardware`、Mid-360、D555、bag。规划器在**另一个终端**自己开。

人在急停旁。不要同时开示教器抢臂，不要开 `cps_telemetry`。

```bash
# 终端 1 — 驱动 + 录包
/home/adamliao/work/RoboticArm/deployment_ws/scripts/record_site.sh real \
  -o ~/robotarm_bags/tracking -n fail_blend

# 终端 2 — 同一 ROS_DOMAIN_ID，跑任务（示例）
ros2 run luggage_planning motion_planner_node
```

任务跑完在终端 1 Ctrl+C。

### Topic list（真机 = 示教器全集 + 下面）

| Topic | 类型 | 回灌角色 |
|---|---|---|
| 上表全部 | | 实际运动 + 传感器 |
| `/trajectory_executor/events` | `std_msgs/String` | accepted / executing / succeeded / aborted |
| `/trajectory_executor/status` | `std_msgs/String` | idle / executing |
| `/elfin_arm_controller/follow_joint_trajectory/_action/send_goal` | action | **期望** JointTrajectory |
| `.../_action/feedback` | action | 执行中实际角 |
| `.../_action/status` | action | 目标状态 |
| `.../_action/get_result` | action | 成功/失败 |
| `/motion_planner/plan_motion/_action/*` | action | 有开 planner 才有 |
| `/motion_planner/go_to_robot_pose/_action/*` | action | 有开才有 |

执行器空闲时仍应 `refresh()` 出非零 `/joint_states`。若全 0，包不能用于 FK。

---

## 回灌（两种包同一套感知路径）

**不要**起 Livox / D555 / CPS / `jazzy_real`。**不要** `ros2 bag play` 去驱动真臂。

录包原则：每条关节消息都是控制器那一拍的完整快照（实际角、指令角、速度、电流、TCP），跟实机一致。查询跟不上就少发几帧，**不**用旧值贴新时间戳、**不**插值凑 50 Hz。回灌用 `/joint_states.header.stamp` 对齐相机/雷达；`/elfin/cps_rate` 只是记录当时真实采样率。

```bash
# 与录制相同的 source / ROS_DOMAIN_ID / PYTHONPATH / LD_LIBRARY_PATH
# 回灌不要设 ROS_LOCALHOST_ONLY=1 也可以；单机回灌设了也行。

BAG=~/robotarm_bags/record_site_YYYYMMDD_HHMMSS

# 终端 1：播包（compressed 图在包里；replay_site 仍会解压给旧节点）
ros2 launch elfin_trajectory_executor replay_site.launch.py bag:="$BAG"
```

```bash
# 终端 2：preprocessor 自己解 JPEG/PNG 并反投影，不必 depth_image_proc
# 也不订 /camera/depth/points。
ros2 run luggage_perception sensor_preprocessor_node --ros-args \
  --params-file /home/adamliao/work/RoboticArm/elfin_humble_ws/src/luggage_perception/config/sensor_preprocessor.yaml \
  --params-file /home/adamliao/work/RoboticArm/deployment_ws/src/elfin_trajectory_executor/config/preprocessor_d555_replay.yaml
# 再起 detector / semantic，同样 use_sim_time:=true
```

默认播包自带 `/tf` `/tf_static`。要用**当前 URDF** 重算 TF：播包时 `--remap /tf:=/tf_recorded`，另开

```bash
ros2 launch luggage_description scene_hardware.launch.py use_sim_time:=true use_rviz:=false
```

### 真机包额外：跟踪 / 混合 / 延迟

离线对比 FJT `send_goal` 里的 `trajectory.points`（期望）和同时段 `/joint_states`（实际）；看 `/trajectory_executor/events` 的 `succeeded` 时刻 vs 关节是否已停。`SettleTracker` / `settle_decision` 可对录下的 `/joint_states` 重跑停稳判定。

---

## 不要做的事

- 示教器模式开 `jazzy_real`，或真机模式再开 `cps_telemetry`
- `scene.launch.py` 和真实关节一起录
- 把 `/luggage/preprocessed/*`、`/luggage/perception/*` 录进同一包再 play（改算法会被旧输出盖掉）
- Humble 和 Jazzy overlay 叠在同一 shell
- `bag_full:=true` 除非明确要固件原生 `/realsense/*` 和点云
