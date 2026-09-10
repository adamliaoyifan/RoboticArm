# D555 官网参数与 TF

Sources:

- Datasheet v1.1: https://realsenseai.com/wp-content/uploads/2025/08/D555-Datasheet-v1.1.pdf
- Product page: https://realsenseai.com/products/d555-poe/
- D450 module (same as D455): https://github.com/realsenseai/realsense-ros/blob/ros2-master/realsense2_description/urdf/_d455.urdf.xacro
- No official D555 URDF: https://github.com/realsenseai/librealsense/issues/14577
- CAD: https://dev.realsenseai.com/docs/cad-files/
- Cell snapshot: `src/luggage_description/config/backups/20260902_183500_eef_livox_d555/NOTES.md`

## 1. 这是什么相机

D555 不是 D455 换个名字，也不是 D435i。

| 层 | D555 |
|---|---|
| 光学模组 | RealSense **D450**（与 D455 同一模组：长基线全局快门 + IMU） |
| 处理器 | **Vision SoC V5** Board 1（D455 是 D4） |
| 外壳 | IP65，167 × 42 × 48 mm，337 g |
| 取流 | **PoE RJ45 + SafeDDS**（ISO 26262）。USB-C 只用于供电、产线调试、硬件同步 |
| 料号 | IVS110DSD555 / VID:PID `8086:0x0B56` |

产品页 FAQ：「D555 is comprised of the same D450 Optical module used in D455 camera」。所以 **机内左右 IR / RGB 的名义几何跟 D455 走**；**壳体、安装孔、重量、接口、分辨率表跟 D455 不同**。

官方 `realsense2_description` **没有** `_d555.urdf.xacro`。仓库里最接近的壳体参考是未发布的 `_d585.urdf.xacro`（182 mm 宽），RealSense 只建议把宽度改成 0.167 m 试看，不能当标定。

## 2. 规格（Datasheet v1.1 优先）

| 项 | D555 | 本仓库 D435 仿真 |
|---|---|---|
| 深度 FOV | **87° × 58° ± 3°**（HD 16:9） | HFOV 1.5184 rad = 87°，画面 4:3 |
| RGB FOV | **90° × 65° ± 3°**（与深度 **不** 匹配） | 与深度共用一个针孔，K 相同 |
| 立体基线 | **95 mm** | yaml `0.05` m |
| depth→color 名义 | D450/D455 URDF **y = −0.059 m**（RGB 在左右 IR 之间） | y = **+0.015 m** |
| 深度分辨率 | 1280×720 / **896×504** / **640×360** / 448×252 | **640×480** |
| 深度帧率 | 720p：5/15/30；更低档最高 60。产品页写最高 90，以表为准 | 30 |
| RGB | 无畸变 YUY2 同档；标定 IR 1280×800@15 | 640×480 RGB8 |
| Min-Z | **26 cm（VGA）**；满分辨率 **~52 cm** | clip near **0.105 m** |
| 理想距离 | **0.6–6 m** | yaml max_reliable 3.0 m |
| 精度 | 立体，随距离下降（脚注 3） | 仿真噪声 σ = 4 mm |
| IMU | D450 模组内置（与 D455 同类） | 未建模 |
| 功率 | 规格书 typical **3.5 W**；产品页 **5.5 W** | — |
| 接口 | RJ45 PoE IEEE 802.3at；jumbo **9000**；Cat6 | gz topic |
| 工作温度 | 壳体 −20–50 °C | — |
| 本单元 | `192.168.11.55`，FW **7.56.37776.6014**，稳定 **640×360@15** | — |

分辨率是 **16:9**。不要把仿真 fx=337.22（640×480、4:3、87°）缩放到 640×360 当真机 K。cy 会错。

本单元：896×504@30 **会掉 DDS 设备**。更高分辨率不能当默认。

## 3. 坐标系

与 D400 ROS wrapper 相同。站在相机后面朝前看：

| 系 | 轴 |
|---|---|
| `*_link`（ROS 机体，`camera_link` / `d555_link`） | +X 前，+Y 左，+Z 上 |
| 光学系（REP-103） | +X 右，+Y 下，+Z 前 |

**深度、左 IR（infra1）、`camera_link` 三者重合。** 这是 wrapper 对 D450 模组的约定，不因 PoE 壳体改变。

光学关节：平移为零，`rpy = -π/2, 0, -π/2`。

```text
p_link = ( z_opt,  -x_opt,  -y_opt )
p_opt  = ( -y_link, -z_link,  x_link )
```

## 4. TF 树（名义；真机以 driver 为准）

没有官方 D555 URDF。机内光学名义值用 D450/D455 宏；壳体用 CAD / 实测，不要用 D435 的 90 mm 盒或 D455 的 15.8 mm 安装孔偏移。

```text
eef_mount_adapter
  └─ camera_link                 当前 URDF：目视调参，按 D435 原点
       └─ d555_link              当前：单位阵（定义，不是标定）
            ├─ depth_frame                    identity（左 IR）
            │    └─ depth_optical_frame       rpy(-π/2, 0, -π/2)
            ├─ color_frame                    名义 y ≈ -0.059 m
            │    └─ color_optical_frame
            ├─ infra2_frame                   名义 y = -0.095 m
            └─ accel/gyro                     D450 IMU 杠杆臂；D555 壳体未公布
```

D455 官方名义（D450 模组，仿真 `use_nominal_extrinsics`）：

| 子坐标系 | 相对 `camera_link` (m) |
|---|---|
| depth / infra1 | `(0, 0, 0)` |
| color | `(0, -0.059, 0)` |
| infra2 | `(0, -0.095, 0)` |
| IMU（D455 壳体） | `(-0.01602, -0.03022, +0.00740)` — **D555 IP65 壳体不要用这个** |

真机 `/extrinsics/depth_to_color` 覆盖名义值。HB-1 必须记录实测旋转和平移。

已知问题：[realsense-ros#3435](https://github.com/realsenseai/realsense-ros/issues/3435) 报告 D555 driver 把 `camera_link → camera_infra2_frame` 的 Y **符号发反**，彩色框跑到立体对外侧。HB-1 的 TF dump 要明确是否复现。复现则名义 −59 mm 不能当投影外参。

`camera_name:=d555` 时帧名是 `d555_link`、`d555_depth_optical_frame`、`d555_color_optical_frame`。本仓库 RViz 订的是 `/camera/d555/depth/color/points`。URDF 的 `camera_link` 必须用固定 TF 接到 driver 的 `d555_link`，且只能发一次。

## 5. 深度点云、RGBD、`camera_link`

| 数据 | 预期 `frame_id`（driver 惯例） | 原点 |
|---|---|---|
| 未对齐深度 | `d555_depth_optical_frame` | = 左 IR = `d555_link` / `camera_link` |
| RGB | `d555_color_optical_frame` | 相对 link 约 −59 mm Y |
| `depth/color/points`（本仓库 RViz 话题） | 名称表示 **对齐到彩色的着色点云** | 应在 **彩色光学系** |
| RGBD / `align_depth` | 彩色光学系 | RGB 光心 |
| IMU | `d555_*_imu_optical_frame` | 壳体杠杆臂，未公布 |

D555 的 RGB FOV（90×65）比深度（87×58）**更宽**。对齐后深度裁到 RGB 时，深度边缘会多一圈；反过来会裁掉 RGB 边缘。这与 D435（RGB 更窄）相反。

判定对齐不能靠「和 Livox 共面」。HB-2 用色边界对深度跳变的有符号像素偏差。RViz 里 D555 显示的 Color Transformer 必须是 RGB8；Intensity / FlatColor 不能当对齐证据。仓库 `view_arm_livox.rviz` 里 D555 已是 RGB8。

## 6. 仿真陷阱（D435 gz，不是 D555）

Fortress `rgbd_camera` 仍挂在 `camera_link`：

- 图像 / `camera_info`：光学系，标签对。
- `/camera/depth/points`：坐标在 `camera_link`，header 写光学系。

预处理器用 `input_cloud_data_frame: camera_link`。D555 driver 的点云已在光学系；硬件 overlay 不得沿用。真机话题还可能是 `/camera/d555/...`，要一层 namespace，不能只改参数。

## 7. 本仓库错误假设（对照）

| 假设 | 实际 |
|---|---|
| D555 进 D435 90 mm 口袋 | 支架按 D555 设计；壳体 167 mm |
| `camera_link` → `d555_link` 单位阵是测出来的 | URDF 定义；安装平移是 D435 目视调参 |
| 仿真 640×480@30 可当硬件 | 硬件 640×360@15，16:9 |
| USB `realsense2_camera` 即插即用 | PoE DDS，MTU 9000；快照在 Jazzy |
| 抄 D455 安装孔 / IMU 偏移 | 壳体不同；无官方 D555 URDF |
