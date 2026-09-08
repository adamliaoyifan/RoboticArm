# 如何把腕部 D435/D435i 换成 D455

本页是替换讨论，不是已批准的实现计划。架构文档在采纳前保持 D435。

## 1. 现在接的是什么

| 层 | 现状 |
|---|---|
| 仿真传感器 | 单个 gz `rgbd_camera`，HFOV 87°，640×480，near 0.105 m，挂在 `camera_link` |
| 仿真外参 | color/depth 同一光心；`semantic_point_filter.extrinsics_source: identity` |
| 仿真点云 | 数据在 `camera_link`，header 写 `camera_depth_optical_frame` |
| 真机 launch | `realsense_d435.launch.py` → `rs_launch.py`，`align_depth.enable:=true`，`camera_name:=camera` |
| 机械 | `arm_realsense_v1.3` 口袋按 D435 90 mm；`d555_link` 与 `camera_link` 重合 |
| 2026-09-02 单元 | 口袋里是 **D555 PoE**（SN `419222302385`），不是 D435i |
| 观察位 | `pickup_observe`：光学系在世界 `(-1.0, 0.0, 1.9)`，+Z 朝下 |

下游契约是话题 `/camera/color/image_raw`、`/camera/depth/image_raw`、`/camera/depth/points` 和帧名 `camera_link` / `camera_depth_optical_frame`。换型号应改驱动、URDF 和 yaml，不要改话题名。

`docs/architecture/motion_compensation.md`：位姿来自 FK + 固定 `T_flange_sensor`，不用相机 IMU 做 deskew。D455 的 BMI055 可以记日志，不要接进预处理的运动补偿。

## 2. 三个硬约束

### 2.1 口袋宽度

D455 比 D435 长 **34 mm**。现支架与 Mid-360 共用 `eef_mount_adapter`。不改 STL 会干涉壳体、Livox 或吸盘面板。

> **2026-09-08 更正**：D555 是 **167 × 42 × 48 mm、337 g**，比 D455 的
> 124 mm 还长。但现支架**本来就是按 D555 设计的**（已确认），所以机械安装
> 没有问题，本节把它描述成「D435 90 mm 口袋」是错的，也不能用来反推 D455。
> 真正待办的是外参：`eef_mount_adapter` → `camera_link` 出自
> `eof_mount_stack_tune_gui.py` 的目视调参且按 D435 机体原点，需要按 D555
> 机体原点重新标定。另外 `realsense_d435.yaml` 里 `mount.tune_joints` 的
> rx/ry 与 `mount.fixed.rpy` 的 roll/pitch 互换且符号不一致，用作标定初值
> 前要先确认哪个是有效字段。见 `Q-20260908-2`。

### 2.2 近距盲区

`pickup_observe` 光心 z = 1.9 m，平台约 0.86 m，到台面约 1.04 m。目录大箱高 0.80 m 时，箱顶距相机约 **0.24 m**。D455 Min-Z ≈ 0.52 m，箱顶深度会无效或噪声极大。D435 近距才是这个姿态能估高的原因。

可做的几何补救（要实测，不能只改 yaml）：

- 抬高或后移 `pickup_observe`，使最大箱顶 ≥ 0.6 m。
- 或把 D455 改到更远的安装点（非腕部）。
- 不要只把仿真 `depth_near` 改成 0.4 却保持同一关节角。

### 2.3 真机已经是 D555

单元快照把 D555 当 D435 口袋的实机。若目标是「仿真贴近实机」，应对齐 D555，而不是引进第三种机身。只有明确要 D455 的量程/基线时才走下面的选项 B/C。

> **2026-09-08 决定**：用户已确认对齐目标就是 **D555 PoE**，下面的选项
> B/C（D455）不执行，保留作参考。D555 的实际差异不只是机身尺寸：PoE +
> SafeDDS 取流（无 USB 选项）、实测稳定 640×360@15（896×504@30 会掉 DDS
> 设备）、Min-Z ~26 cm、快照 bring-up 在 Jazzy。这些会影响 PF-R 的速率类
> 验收线和跨机架构，不是换 URDF 能覆盖的。见 `Q-20260908-2`。

## 3. 选项

### 选项 A — 不上 D455（推荐默认）

腕部继续 D435 仿真 + D555 真机。把 D455 参数留在本目录当参考。

适合：当前观察距、现支架、正在进行的感知验收（PF-R*）不能被相机换代打断。

### 选项 B — 仿真名义 D455，仍一个 `rgbd_camera`

改壳体尺寸、HFOV、clip、yaml 基线与 color 偏移，传感器仍是一个针孔 RGBD。

得到：更长的可靠距离、更宽的壳体碰撞。得不到：59 mm RGB 光心、95 mm 立体、真实 align_depth。`extrinsics_source` 在仿真里仍应是 `identity`。点云陷阱不变，预处理器仿真路径不变。

这是「改名和改 FOV」，不是 RealSense D455。只有在新支架和观察距已经满足 2.1–2.2 时才值得做。

### 选项 C — 真机换 D455，仿真分 color/depth 两个传感器（完整替换）

分阶段，不要一次改光心、分辨率和支架。

**阶段 0 — 几何门（先过再写代码）**

1. 新打印件：124 mm 壳体 + 现 Mid-360 垫。
2. 重标 `eef_mount_adapter` → `camera_link`（现 `0.013 0.097 -0.021` 是 D435 口袋）。
3. 重调 `pickup_observe`，使最大目录箱顶工作距 ≥ 0.6 m，大箱仍完整落在 FOV 内（当前注释：更近的 `(-0.8, 0.0, 1.7)` 会切掉 0.80 m 箱）。
4. 记录新工作距、FOV 裁切和与 Livox 的间隙。

**阶段 1 — 描述与碰撞（仿真仍可单传感器）**

- 新宏 `realsense_d455`，数值跟官方 `_d455.urdf.xacro`。
- 保留帧名 `camera_link`、`camera_depth_optical_frame`、`camera_color_optical_frame`。
- `d555_link` 单位阵：要么删掉并改 `test_d555_d435_mount.py`，要么改成文档化的 `d455_link` 别名。
- 光学 TF：真机由 `realsense2_camera` 发；URDF 只在 `use_nominal_extrinsics` 时提供名义值，避免和 driver 双发。这与现 D555 注释一致。
- 仿真暂可保留一个 `rgbd_camera`，clip 改为约 0.40–6.0 m，HFOV 仍按 87°。分辨率先保持 640×480；848×480 或 1280×720 会把点云体积顶回 PF-R9 刚压下去的带宽。

**阶段 2 — 真机驱动**

`realsense_d435.launch.py` 对 D455 几乎能用：同一 `rs_launch.py`，`align_depth.enable:=true`，`pointcloud.enable:=true`，`enable_sync:=true`，`camera_namespace:=""`, `camera_name:=camera`。需要改的是：

- `serial_no` 指向 D455，而不是「找到的第一台 D435」。
- 深度/彩色 profile 显式写出（建议先 `848x480x30` + 对齐后的彩色，或保持 640×480 直到带宽验收）。
- 硬件预处理 overlay：`input_cloud_data_frame` **不要**再写 `camera_link`。对齐点云在 `camera_color_optical_frame`；未对齐点云在 `camera_depth_optical_frame`。与现 launch 的 `align_depth:=true` 一致时，输出帧应是彩色光学系。
- `semantic_segmenter.real.yaml`：`extrinsics_source: config`。yaml 里的 depth_to_color 从 **15 mm +Y** 换成该机标定（名义 **59 mm −Y**，以 `/extrinsics/depth_to_color` 为准）。
- 在线节点继续优先用 `camera_info`，不要把仿真 fx=337.22 拷到真机。

**阶段 3 — 仿真双光心（仅当像素投影误差不可接受）**

两个 gz 相机，color 相对 depth 平移 −59 mm Y，filter 在仿真里也走 `config` 外参。代价：打破「仿真 identity」、双份 `camera_info`、预处理器不能再靠 gz 同源同戳当唯一假设。PF-R9 的 pairing 测量要重做。在阶段 0–2 稳定之前不要做。

**不要做**

- 用 D455 IMU 代替 `/joint_states` FK。
- 把仿真点云 header 陷阱「修」进 `gz_frame_id`（会标错图像）。
- 默默把 640×480 升到 1280×720 而不重跑预处理器吞吐。
- 在未重调观察位时把 `depth_near` 改成 0.4 m。

## 4. 帧与话题在替换后如何接

对齐开启时（现真机 launch 默认）：

```text
RGB 像素 ── camera_color_optical_frame
对齐深度 / RGBD / 着色点云 ── 同一光学系（原点在 RGB）
camera_link ── 左 IR，手眼与 URDF 安装
camera_link → color_optical = 名义 (0, -0.059, 0) 再乘光学旋转
```

YOLO 在 RGB 上跑、点云来自对齐深度时，仿真里的 `extrinsics_source: identity` 对真机是错的。真机必须 `config` 或等价的对齐后「已同光心」约定，且 **点云 `frame_id` 与所用 K 一致**。

未对齐点云仍在深度光学系。禁止未对齐点云配 RGB 内参，除非 filter 显式做 depth_to_color。

## 5. 文件清单（选项 C）

描述：

- `src/luggage_description/urdf/realsense_d435.urdf.xacro` → 新 `realsense_d455.urdf.xacro` 或宏改名
- `src/luggage_description/urdf/elfin_s{20,30}_with_camera.urdf.xacro` 及 `*_tune` 变体
- `src/luggage_description/config/realsense_d435.yaml`
- `src/luggage_description/config/camera_mount_origin.xacro`（阶段 0 重标）
- `src/luggage_description/urdf/eef_sensor_mount.urdf.xacro` 与 `meshes/arm_realsense/`
- `src/luggage_description/test/test_d555_d435_mount.py`
- `src/luggage_description/config/robot_poses.yaml.example`（`pickup_observe`）

感知 / bringup：

- `src/luggage_perception/launch/realsense_d435.launch.py`
- `src/luggage_perception/config/sensor_preprocessor.yaml` 加硬件 overlay
- `src/luggage_perception/config/semantic_segmenter.yaml` 与 `.real.yaml`
- 硬编码 1.5184 / fx=337.22 的测试（仿真 K 变了才改）

架构（仅在型号真正切换后）：

- `docs/architecture/sensor_data_pipeline.md`
- `docs/architecture/motion_compensation.md`

## 6. 验收（若进入实现）

几何：新支架间隙；最大目录箱在 `pickup_observe` 下箱顶距离 ≥ 0.6 m；FOV 不切 0.80 m 箱。

仿真：现感知单测仍过；若改 HFOV/分辨率，K 与 live `camera_info` 一致。

真机：`align_depth` 点云与 RGB 叠图；`frame_id` 与预处理 `input_cloud_data_frame` / `output_cloud_frame` 一致；外参符号为 −Y 量级 59 mm 而不是 +15 mm。

吞吐：预处理 `cloud_ok` 与 PF-R9 带宽约束在新 profile 下重测。不把旧 640×480 的 B3 数字当新分辨率的证明。

评测：Gate 4 / pickup 估高在新观察距下重跑。抬高相机改变每像素米制，旧 top/support 阈值不能直接当回归基线。
