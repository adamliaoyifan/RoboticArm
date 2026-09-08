# D455 官网参数与 TF

Sources:

- Product page: https://www.intelrealsense.com/depth-camera-d455/
- Product brief PDF (1worldsync D455 sheet)
- Official ROS URDF: https://github.com/realsenseai/realsense-ros/blob/ros2-master/realsense2_description/urdf/_d455.urdf.xacro
- ROS wrapper TF notes: https://github.com/realsenseai/realsense-ros (ros2-master README)

## 1. 命名

没有独立 SKU「D455i」。D455 出厂带 Bosch BMI055 IMU。D455f 是带 IR 滤光片的变体，不是 IMU 变体。

本仓库仿真宏是 `realsense_d435`。URDF 里另有 `d555_link`，相对 `camera_link` 为单位阵，表示 D555 坐在同一打印口袋。

## 2. 规格对照

| 项 | D435 / D435i（本仓库仿真按 D435） | D455（官网 / 简报） |
|---|---|---|
| 模组 | D430 + D4 | D450 + D4 |
| 深度 FOV | 87° ± 3° × 58° ± 3°（仿真 HFOV 1.5184 rad = 87°） | 官网 87° × 58°；简报 86° ± 3° × 57° ± 3° |
| RGB FOV | 约 69°；仿真用同一个 rgbd 传感器，K 等于深度 K | 官网 90° × 65°；简报写成与深度匹配的 86° ± 3° × 57° ± 3° |
| 立体基线 | 50 mm（本仓库 yaml `baseline: 0.05`） | **95 mm** |
| depth→color 名义平移 | URDF +15 mm `camera_link` +Y | URDF **−59 mm** `camera_link` +Y |
| 深度分辨率 | 最高 1280×720；仿真 640×480 | 最高 1280×720；驱动默认常为 848×480 |
| RGB 分辨率 | 最高 1920×1080；仿真 640×480 | 最高 1280×800 |
| 快门 | 深度全局；D435 RGB 卷帘，D435i 带 IMU | 深度与 RGB 均为全局快门 |
| 推荐距离 | 0.1–3 m 量级；仿真 near 0.105 m、far 3.0 m | **0.6–6 m**；满分辨率 Min-Z ≈ 52 cm；4 m 处 < 2% |
| IMU | 仅 D435i | 标配 BMI055 |
| 外形 | 90 × 25 × 25 mm | **124 × 26 × 29 mm** |
| 接口 | USB-C 3.1 Gen 1 | USB-C 3.1 Gen 1 |

营销页与 PDF 简报的 RGB FOV 不一致。工程以出厂标定和 live `camera_info` / `extrinsics/depth_to_color` 为准。URDF 数值只是名义值。

## 3. 坐标系约定

站在相机后面朝前看（ROS wrapper 的约定）：

| 系 | 轴 |
|---|---|
| `camera_link`（ROS 机体） | +X 前（出镜头），+Y 左，+Z 上 |
| 光学系（REP-103） | +X 右，+Y 下，+Z 前 |

**原点：深度、左 IR（infra1）、`camera_link` 三者重合。** 这是 wrapper 写死的，D435/D435i/D455 相同。

每个传感器还有一对「ROS 机体系」和「光学系」。光学关节平移为零、姿态固定：

```text
xyz = 0 0 0
rpy = -π/2  0  -π/2
```

同一原点时：

```text
p_link = ( z_opt,  -x_opt,  -y_opt )
p_opt  = ( -y_link, -z_link,  x_link )
```

## 4. 官方 D455 TF 树

`use_nominal_extrinsics:=true` 时（仿真；真机由 driver 发标定 TF）：

```text
parent
  └─ camera_bottom_screw_frame     三脚架 1/4-20
       └─ camera_link              左 IR / 深度原点
            ├─ camera_depth_frame                 identity
            │    └─ camera_depth_optical_frame    rpy(-π/2, 0, -π/2)
            ├─ camera_infra1_frame                y = 0
            ├─ camera_color_frame                 y = -0.059
            │    └─ camera_color_optical_frame    同一光学旋转
            ├─ camera_infra2_frame                y = -0.095   (95 mm 基线)
            └─ accel/gyro_frame                   (-0.01602, -0.03022, +0.00740)
```

壳体相对安装孔（URDF 注释指向 datasheet Rev.009 Fig. 4-4）：

| 量 | 值 |
|---|---|
| 壳体 | 124 × 29 × 26 mm |
| 安装孔到中心 | 15.8 mm |
| 零深度到玻璃 | 4.55 mm |
| 玻璃相对前面板内凹 | 0.1 mm |

真机运行时 `/camera/camera/extrinsics/depth_to_color` 覆盖这些名义值。

## 5. 深度点云、RGBD、`camera_link`

| 数据 | 真机 `frame_id` | 原点 |
|---|---|---|
| 未对齐深度 / 未对齐点云 | `camera_depth_optical_frame` | = `camera_link` |
| RGB 图像 | `camera_color_optical_frame` | 相对 link 约 −59 mm Y |
| `align_depth` 后的深度、RGBD、由对齐深度生成的点云 | `camera_color_optical_frame` | RGB 光心 |
| IMU | `camera_*_imu_optical_frame` | IMU 杠杆臂 |

要点：

- Wrapper 只做 **depth → color**，不做 color → depth。
- RGBD 话题要 `enable_rgbd`、`enable_sync`、`align_depth.enable`。
- 未对齐时，用 RGB 像素去索引深度点云会有大约 6 cm 的水平偏差（D455；D435 约 1.5 cm）。
- 对齐后像素对齐了，3D 原点也改到 RGB。手眼若标的是 `camera_link` / 深度光心，不能把对齐点云当深度原点用。

## 6. 本仓库仿真陷阱（不是 D455 行为）

Gazebo Fortress `rgbd_camera` 挂在 `camera_link` 上，`gz_frame_id` 标成 `camera_depth_optical_frame`：

- 图像和 `camera_info`：光学系，标签正确。
- `/camera/depth/points`：坐标在 **`camera_link`（+X 朝前）**，header 仍写光学系。

预处理器因此用 `input_cloud_data_frame: camera_link`，输出改到 `camera_depth_optical_frame`。真机 `realsense2_camera` 的点云已经在光学系；硬件 overlay 不得沿用这个仿真参数。见 `docs/architecture/sensor_data_pipeline.md`。
