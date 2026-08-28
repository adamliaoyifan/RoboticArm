# M2 感知链问题记录：DetectLuggage 结果与 GT 严重不符（已定位根因）

日期：2026-08-20
状态：**根因已定位，修复未实施**
关联：[sim closed-loop plan](../plans/ros2_sim_closed_loop_plan.md) M2；原始数据见 `evidence/m2_occlusion/`

## 现象

Fortress 仿真中，机械臂已到 `pickup_observe` 位姿（FJT 执行 SUCCEEDED），取货平台上已生成箱子，
`/luggage_detector/detect_luggage` 返回 `success=True, conf=1.00`（走了感知路径，非 GT 回退），
但估计结果与真值完全不符：

| | 尺寸 W×D×H (m) | 中心 (m) |
|---|---|---|
| GT（spawner） | 0.621 × 0.443 × 0.264 | (-1.000, 0.000, 0.992)（gz dynamic_pose 实测） |
| 检测（bug 存在时） | 0.298 × 0.068 × 0.971 | (-0.583, 0.082, 1.346) |

初步排查时曾怀疑相机被机械臂自身结构遮挡：ROI（取货源 ±0.5 m）内 21338 个点全部位于
z∈[1.70, 1.90]，箱顶（z≈1.12）与平台顶（z=0.86）的点的数量为 0。**该"遮挡"结论是误诊**，
真实原因是下面两条 bug。

## 根因（两条叠加）

### Bug 1：点云坐标约定错位（主因）

gz Fortress 的 `rgbd_camera` 传感器**按 sensor 坐标系约定（+X 前向，SDF 相机约定）发布点云**，
而我们在 xacro 里设置的 `gz_frame_id=camera_depth_optical_frame` 只改写消息头的 frame_id 标签，
不改变数据的实际坐标约定。于是"标签说 optical、数据是 camera_link"，用
`world ← camera_depth_optical_frame` 变换点云得到的世界系数据完全错位。

决定性数值证据（`evidence/m2_occlusion/frame_convention_probe.txt`，640×480 全量点云）：

| 变换所用 frame | z>1.7（相机上方，几何不可能） | z∈[0.8,1.2]（平台0.86/箱顶0.99–1.12 高度带） |
|---|---|---|
| camera_depth_optical_frame（标签） | **140039** | **0** |
| camera_link（实际约定） | **0** | **74536** |

即：箱子一直看得见，是变换用错了坐标系。此前"相机被自身结构挡死"的推断由此推翻——
那批 z∈[1.6,2.0] 的点其实就是错变换下的箱/平台点。

佐证：URDF→SDF 固定关节合并后，sensor 挂在 `elfin_link6` 上，lumped pose
`0.00333 0.16710 0.16282 / -3.07336 -1.36336 1.54092` 与 camera_link 的 visual pose 一致
（`evidence/m2_occlusion/sdf_sensor_pose.txt`）——传感器 frame 即 camera_link 本体。

TF 本身验证无误（`evidence/m2_occlusion/tf_frames_at_pickup_observe.txt`）：
camera_link 位于 (-0.800, 0.000, 1.700)，+X=(0,0,-1) 正下方；
camera_depth_optical_frame +Z=(0,0,-1) 正下方；与 noetic IK 目标
"camera_depth_optical_frame at (-0.8, 0, 1.7), optical +Z down" 完全一致。

### Bug 2：inf 点未过滤

远平面（3.0 m）未命中的像素在点云中产生 **inf** 值（本次全量点云 x/y/z 范围均含 inf）。
`sensor_msgs_py.point_cloud2.read_points(skip_nans=True)` 只滤 NaN 不滤 inf，
inf 进入 `estimate_box` 的 RANSAC/PCA 后污染统计量（诊断脚本里 np 统计直接变 nan 可证）。

## 修复方案（2026-08-20 审计后修订）

先做全量"消息↔坐标系"审计（`evidence/m2_occlusion/pixel_frame_audit.txt`），
用已知世界坐标点（箱顶中心）反查有组织点云的像素位置，并与两种投影约定对比：

- 箱顶 (-1.0, 0, 1.12) 实测落在像素 (u=202, v=239)，世界误差 0.003 m；
  用实际像素射线验证：u = cx − fx·y_cl/x_cl（202≈320−337.2×0.20/0.58=204），
  v = cy − fx·z_cl/x_cl（平台点 v=185≈240−337.2×0.12/0.72=184）。
- 即**图像与 camera_info 是标准 optical 约定**（x_opt=−y_cl, y_opt=−z_cl, z_opt=x_cl），
  相对 `camera_depth_optical_frame` 完全正确；
- 深度图 32FC1 米制 z-depth，与点云逐像素对齐（同像素 0.578=0.578）。

**结论：gz Fortress rgbd_camera 自身就不一致**--image/camera_info 按 optical 约定发布
（标签正确），points 却按 sensor（camera_link，+X 前向）约定发布。`gz_frame_id` 一个
设置管全部话题的标签，因此**不能**把它改成 camera_link（会反过来弄坏图像/相机内参的标签）。

据此修复：

1. **保持** `gz_frame_id=camera_depth_optical_frame`（图像、camera_info 标签正确）。
2. `luggage_detector_node.py`：新增参数 `cloud_data_frame`（默认 `camera_link`），
   点云世界变换按该 frame 查 TF，并在节点内注释记录 gz 的这一不一致；header 里的
   frame_id 只用于诊断日志。
3. `luggage_detector_node.py`：点数组构造后 `pts = pts[np.isfinite(pts).all(axis=1)]`
   （远平面未命中=inf，本次实测 307200 像素中 104853 个非有限值）。
4. 后续移植语义链（semantic_point_filter 等）时，凡消费 points 的节点都按
   `cloud_data_frame=camera_link` 处理；消费 image/mask 的节点按 optical 约定即可。
5. 重测验收：DetectLuggage 位置/尺寸 vs GT 偏差（对齐 noetic size_uncertainty 口径）。

## 附：被误诊排除的假设

- ~~机械臂/吸盘遮挡相机~~（camera_link 变换下平台高度带 74536 点可见；且几何上
  camera(-0.8,0,1.7)→box(-1.0,0,0.99) 射线不经过 panel（panel 在 y∈[-0.44,-0.16]）。
- ~~gz 模型位姿与 RSP TF 不一致~~（model spawn pose (0,0,0.86,yaw90°) 与
  world_base patch 一致；`/joint_states` 同源）。
- ~~suction_panel collision 未剥离导致渲染遮挡~~（gz 传感器渲染 visual 几何；
  noetic 也只剥 collision，不构成差异）。
