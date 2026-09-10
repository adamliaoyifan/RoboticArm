# 如何把仿真 D435 换成已装 D555

本页讨论仿真/契约向 **D555 PoE** 对齐，不是把腕部再换成第三种机身。架构在 HB-1/2/3 和 F3 共识之前保持 D435。

## 1. 现在接的是什么

| 层 | 现状 |
|---|---|
| 真机 | D555 PoE，`192.168.11.55`，FW 7.56.37776.6014 |
| 支架 | 按 D555 设计；`arm_realsense_v1.3` |
| 安装 TF | `eef_mount_adapter` → `camera_link` = `0.013 0.097 -0.021`，rpy `0.03770 1.36345 1.57080`（GUI 目视，D435 原点） |
| 机体 | `camera_link` → `d555_link` 单位阵 |
| 仿真 | 单个 gz `rgbd_camera`，640×480@30，near 0.105 m，HFOV 87° |
| 仿真外参 | identity；color/depth 同一光心 |
| 真机 launch | `realsense_d435.launch.py`，USB 假设，`camera_name:=camera`，话题 `/camera/...` |
| 真机 RViz | `/camera/d555/depth/color/points`，Color Transformer RGB8 |
| 观察位 | `pickup_observe`：光学系世界 `(-1.0, 0.0, 1.9)`，+Z 朝下 |

`docs/architecture/motion_compensation.md`：位姿来自 FK，不用相机 IMU deskew。D555 IMU 可记日志，不要接进预处理。

## 2. 硬约束

### 2.1 机械

安装已确认。不要再论证「167 mm 进不进 90 mm 口袋」。待办是 **机体原点**：当前 `camera_link` 按 D435 左 IR 在 90 mm 壳体里的位置调的。D555 左 IR 在 167 mm IP65 壳里的位置不同，单位阵 `d555_link` 只是命名，不是测量。HB-3 测残差，不写新 TF。

### 2.2 近距

`pickup_observe` 下 0.80 m 箱顶约 **0.24 m**。

| 模式 | Min-Z | 相对 0.24 m 箱顶 |
|---|---|---|
| D435 仿真 | 0.105 m | 覆盖 |
| D555 VGA | ~0.26 m | **可能进盲区** |
| D555 满分辨率 | ~0.52 m | 盲区 |

D555 比 D455 满分辨率近，但 **没有取消** 抬高/后移观察位的需要。不要只把仿真 `depth_near` 改成 0.26 却保持同一关节角。VGA 工作距应 ≥ 0.26 m，理想 ≥ 0.6 m。

### 2.3 流与传输

- 画面 **16:9**（640×360），不是仿真 4:3。
- 稳定 **15 Hz**。PF-R8/R9/R10 的 30 Hz 墙钟条要重推；B3 是比率，可保留。
- 点云走以太网 DDS。无法靠 composable node / 共享内存消掉整云传输。这是 F3（深度优先、少发点云）的硬件理由，与 PF-R9 B3 条无关。
- Humble 上 D555 DDS 未验证。快照 bring-up 是 **Jazzy** + librealsense **2.58.4**。

### 2.4 帧与话题名

架构表是 `/camera/color/image_raw` 等。硬件 RViz 是 `/camera/d555/depth/color/points`。对齐需要 **namespace + profile 层**，不是只改 yaml 里的一个 frame 字符串。

`depth/color/points` 按 RealSense 命名是 **对齐到彩色的着色点云**。是否真如此由 HB-1 的 `frame_id` vs K 和 HB-2 的像素偏差判定。F3 若改成深度主契约，未对齐深度光学系才是规范输出。

## 3. 选项

### 选项 A — 仿真保持 D435，硬件用 adapter（最短路径）

仿真不动。硬件 overlay：话题 remap 到 `/camera/...`、`input_cloud_data_frame` 用 driver 光学系、`extrinsics_source: config`、profile 锁 640×360@15。

得到：PF-R 仿真验收不被换相机打断。得不到：FOV 纵横比、Min-Z、15 Hz、59 mm 光心。只适合当 HB 事实还没回来时的过渡。

### 选项 B — 仿真名义 D555，仍一个 `rgbd_camera`（推荐在几何门之后）

改：壳体碰撞盒 167×42×48 mm、质量 337 g、HFOV 87°、画面 **640×360**、clip near **0.26 m**、基线 yaml 0.095。传感器仍是一个针孔。

得到：分辨率、近距、壳体碰撞与真机同量级。得不到：90° RGB、−59 mm 光心、DDS。`extrinsics_source` 仿真仍 identity。gz 点云陷阱不变。

前提：HB-3 残差可接受或已计划重标；`pickup_observe` 箱顶 ≥ VGA Min-Z。

### 选项 C — 仿真双光心 + 真机 D555 契约（完整）

两个 gz 相机，color 相对 depth 平移约 −59 mm Y，RGB HFOV 90°。Filter 仿真也走 `config`。代价：打破 gz 同源同戳，PF-R9 pairing 要重测。排在 F3 和 HB-2 对齐判定之后。

**阶段（选项 B 或 C）**

0. HB-1/2/3 事实（只读）。几何：观察位箱顶距离、FOV 是否切 0.80 m 箱。
1. 描述：D555 碰撞盒；保留 `camera_link` 作为 URDF 安装系；`d555_link` 接到 driver 帧；光学 TF 只由 driver 发。
2. 硬件 launch：PoE/DDS，不要 USB `realsense_d435.launch.py`。锁 640×360@15。MTU 9000。namespace 映射到契约话题。
3. 预处理硬件 overlay：不要 `input_cloud_data_frame: camera_link`。输出帧与 HB-1 的 frame-versus-K 判定一致。
4. 仅当 HB-2 证明像素误差不可接受时，做仿真双光心。

**不要做**

- 按 D455 目录改 124 mm 壳体或 USB 路径。
- 用 D555 IMU 代替 FK。
- 默默升到 896×504@30。
- 在未测工作距时把 `depth_near` 改成 0.26。
- 订阅相机原生 `rt/realsense/D555_*` DDS；必须走 `realsense2_camera`，否则 `frame_id` 会变成帧计数器。
- 在 URDF 里复制一份光学 TF，同时让 driver `publish_tf:=true`。

## 4. 替换后帧怎么接

若 HB-1 确认 `depth/color/points` 为彩色对齐：

```text
RGB ── d555_color_optical_frame
着色点云 / 对齐深度 ── 同一光学系（原点在 RGB）
camera_link / d555_link ── 左 IR，URDF 安装
```

YOLO 在 RGB、点云在对齐深度时，仿真 `extrinsics_source: identity` 对真机是错的。

若 F3 规定深度主、点云未对齐：

```text
深度 / 点云 ── d555_depth_optical_frame  (= camera_link 原点)
RGB ── d555_color_optical_frame
投影必须带 depth_to_color（名义 −59 mm，以实测为准）
```

## 5. 文件清单（实现时，现在不要改）

- `src/luggage_description/urdf/realsense_d435.urdf.xacro`（壳体、clip、注释）
- `src/luggage_description/config/realsense_d435.yaml`（K、基线、range、质量）
- `src/luggage_description/config/camera_mount_origin.xacro`（HB-3 之后重标）
- `src/luggage_description/config/robot_poses.yaml.example`（观察位）
- `src/luggage_description/test/test_d555_d435_mount.py`
- `src/luggage_perception/launch/realsense_d435.launch.py` → D555 PoE launch
- `src/luggage_perception/config/sensor_preprocessor.yaml` 硬件 overlay
- `src/luggage_perception/config/semantic_segmenter.real.yaml`
- `docs/architecture/sensor_data_pipeline.md`（仅在契约真正切换后）

CAD 用官方 D555 STEP，不要用 D435/D455 mesh 冒充碰撞。

## 6. 验收（实现阶段）

几何：VGA 下箱顶 ≥ 0.26 m（理想 ≥ 0.6 m）；0.80 m 箱仍在 FOV 内。

事实：HB-1 九项；HB-2 三距离有符号像素偏差；HB-3 三姿态平面残差。

仿真：单测仍过；K 与 live 16:9 `camera_info` 一致，不是 337.22 缩放。

真机：namespace 映射后预处理吃得到流；`frame_id` 与 K 一致；15 Hz 下 PF 墙钟条已重推或显式豁免。

吞吐：以太网整云成本按 F3 处理，不要假设 USB 零拷贝。
