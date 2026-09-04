# 仿真闭环移植方案（Fortress · 真实感知 · 单箱抓放）

目标：在 ROS 2 Humble 上打通 noetic `sim_full` 的等价闭环——Fortress 仿真世界里，
机械臂通过**真实感知链**（gz 相机 RGB-D → 语义分割 → 点云过滤 → 箱子检测）识别单箱，
MoveIt 2 规划执行抓取-搬运-放入集装箱，吸盘仿真完成吸附/释放，单箱 pick-place 可重复。

不在本方案范围：orchestrator 完整状态机与多箱装箱循环（Phase 9 主体）、GUI、
MuJoCo/Isaac（7B/7C）、真机（Phase 8）。

依据：docs/plans/ros2_humble_mvp_and_migration_plan.md（Phase 7A/6/5 子集 + 单箱验收）、
noetic 侧参考 `elfin_noetic_ws`（只读）。

## 里程碑 M1：Fortress 仿真后端（对应 Phase 7A）

新建 ROS 2 `luggage_gazebo`（ament_cmake，替代 catkin 版；旧包保留为参考）：

1. `elfin_description` 扩展：把 noetic `luggage_description/urdf/` 中的
   `elfin_s20_with_camera.urdf.xacro`、`realsense_d435.urdf.xacro`、
   `vacuum_gripper.urdf.xacro`、`suction_panel_mount` / `eef_sensor_mount` / mount 链
   合入现有 S20 + ros2_control 描述；关节名保持 `elfin_joint1–6`；
   `hardware_plugin` 参数新增 `gz_ros2_control::GazeboSimSystem` 分支。
2. D435 加 Fortress sensor 标签（`ignition::gazebo::systems:: sensors::RgbdCamera`）。
3. `airport_loading.world` → Fortress SDF；集装箱/台座几何由 `scene_tf.yaml` +
   `generate_real_container_gazebo.py` 管线改出 SDF（STL URI 用 model:// 正确解析）。
4. spawn/delete/get state 用 `ros_gz_sim create/delete` + bridge 替换 Classic service；
   bridge 只桥接 clock、相机、必要的 model state。
5. launch 启动前跑 GPU 检查，`llvmpipe` 直接失败（复用 `scripts/check_gpu_renderer.sh`）。
6. 验收：同一 MoveIt/controller 配置 mock↔Fortress 可切换；observe 位姿可达；
   `/joint_states` 六关节稳定；renderer 为 NVIDIA。

## 里程碑 M2：真实感知最小链（Phase 5 子集，抓取检测路径）

gz 相机出的是 depth image，noetic 链吃 `/camera/depth/points`——用
`depth_image_proc` 的 point_cloud_xyz（+TF 修正）补齐话题形态，frame 与 URDF 相机 frame 对齐。

按依赖顺序移植为 rclpy（Torch/CLIP/YOLO 推理逻辑不动，vendor 目录不动）：

1. `semantic_segmenter_node`（mask/overlay/info 输入输出保持语义）。
2. `pickup_box_pointcloud_filter_node`、`robot_self_point_filter`、
   `known_scene_point_filter`（URDF 自mask 由 scene_tf/container 已知几何生成）。
3. `luggage_detector_node`：`DetectLuggage` 服务；按计划第 9 节把
   `rospy.set_param("/luggage/perception/...")`、`/luggage/current_box` 参数状态
   改为 typed topic/service。
4. QoS 契约：SensorData（best_effort, keep_last 1）、message_filters 按戳配对、
   TF2 查询 fail-closed（计划第 10 节表格）。
5. 验收：固定场景下 DetectLuggage 的箱子位姿/尺寸与 Gazebo GT 偏差在容差内
   （对齐 noetic `size_uncertainty` 的指标口径）；P95 延迟/丢帧不劣于目标。

## 里程碑 M3：运动执行最小链（Phase 6 子集）

1. 薄 MoveIt 2 运动适配器：实现 Phase 2 已定义的 `PlanMotion` / `GoToJointValues` /
   `GoToRobotPose` action（C++ `MoveGroupInterface` 优先，承载后续业务约束）；
   PlanningScene 从 `scene_tf.yaml` 加载集装箱碰撞体。
2. `scene_manager_node` 最小子集：`SyncStaticScene`、attach/release
   AttachedCollisionObject（吸盘 attach 时箱子进 ACM）。
3. `waypoint_generator_node`（纯逻辑已在包内，补 rclpy 壳）：approach / lift / retreat。
4. `settle_criterion` / `vacuum_retention` 判据随壳接入。
5. 验收：observe → approach → 抓取位 → 提升 → 集装箱槽位 全链 20/20 成功，
   轨迹误差 ≤ 0.01 rad（与 MVP 同预算）。

## 里程碑 M4：吸盘仿真 + 单箱闭环驱动

1. 吸盘仿真（对应 noetic `vacuum_simulator_node` 的 gazebo_follow 模式）：
   `VacuumCommand` 服务保持同名；Fortress 实现优先 gz `DetachableJoint`/
   joint 释放 plugin（写入箱子 SDF），退路为 kinematic-follow 节点
   （通过 gz bridge 写 model pose）；attach 条件复用 `vacuum_attach_utils`。
2. 单箱闭环驱动节点（测试驱动，非 orchestrator）：
   spawn 箱 → DetectLuggage（真实感知）→ ComputePlacement → PlanMotion → 抓取 →
   VacuumCommand → 搬运 → 释放 → VerifyPlacedBox → 循环。
3. 验收：Fortress 下连续 ≥20 个单箱 pick-place 循环，成功率、周期时间、
   释放沉降记录到 `docs/status/`（对齐 noetic physical_closed_loop 指标口径）。

## 工程约束（沿用总计划）

- 所有改动只在 `elfin_humble_ws`；`elfin_noetic_ws` 只读。
- 每个里程碑独立提交、独立验收后才删对应 ROS 1 包的 `COLCON_IGNORE` 语义。
- 新增 ROS 2 测试（launch test + replay 固定数据），不恢复 ROS 1 测试。
- 不在业务代码里加仿真分支；后端切换只通过 `hardware_plugin`/launch 参数。

## 风险

| 风险 | 缓解 |
|---|---|
| Fortress 点云生成与 noetic `/camera/depth/points` 语义不一致 | M2 先做话题形态对齐 + 固定场景数值对比，再接检测 |
| gz 相机噪声/视场与 D435 标定差异导致检测偏差 | 用 `size_uncertainty` 口径量化；必要时给 sensor 加 noise 配置 |
| 吸盘 DetachableJoint 与 MoveIt ACM 状态不同步 | attach/release 同时驱动两侧；失败视为闭环失败，不静默 |
| Fortress 渲染性能（GPU 硬门） | 每次启动跑 renderer 检查；headless 矩阵跑法保留 |
