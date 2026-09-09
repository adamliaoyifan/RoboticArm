# Plans

- [Agent owner-closed subtask efficiency experiment](agent_subtask_efficiency_experiment.md)
  (compares queue wait, lead time, handoffs, rework, and escaped defects
  against the previous reviews-eng-test stage workflow)

Implementation plans for migrating this workspace from ROS 1 Noetic to ROS 2
Humble belong here.

- [Elfin ROS 2 Humble MVP and migration plan](ros2_humble_mvp_and_migration_plan.md)
- [ROS 2 Humble migration TODO](ros2_migration_todo.md)
- [ROS 2 simulation backends](sim_backends_ros2.md)

闭环验收（YOLO → 估高 → plan pick → retreat）按 todo 拆成四份实现说明。
共同的现状与缺口分析在仓库外的 Cursor plan
`~/.cursor/plans/closed-loop_gap_analysis_4401de44.plan.md`：

- [Todo 1 — YOLO 语义节点与 cargo 点云](closed_loop_yolo_nodes.md)
- [Todo 2 — 检测 vs GT 精度门](closed_loop_detection_gt_gate.md)
  （门槛是初值，第一轮 P95 回填后才成契约；这就是检测指标）
- [Todo 3 — pick / retreat 的 ROS 2 壳](closed_loop_pick_retreat_nodes.md)
  （rospy 脚本先迁 `scripts/ros1_reference/`，Humble 沿用原节点名）
- [Todo 4 — 闭环评测驱动](closed_loop_eval_driver.md)
- [走廊约束架构 — 多箱 place 的路径与约束分层](corridor_constraints.md)
  （规范：进=出=同一条开口走廊，槽位否掉而不是绕行；学习只换候选生成）
- [装箱全流程评测指标体系 v1](packing_eval_metrics.md)
  （A 效率/B 节拍/C 可靠率/D 抓放质量/E 走廊安全/F 资源；33 项，
  含 volume_fraction 与落盘契约，切片 D 验收的最小集已标注）
  （残留进程：评测用独立 `ROS_DOMAIN_ID`；手动 Ctrl+C 同样会留下 gz/bridge）
- [Todo 5 — place + 占据栅格 + 顺序装箱](closed_loop_place_pack.md)
  （Todo 4 在 retreat 后关真空回 observe；本 todo 接着放入集装箱直到 BIN_FULL）
  - [切片 A 详细设计 — place 动作状态机与 debug](todo5_place_action.md)

Phase 1 Gates 1–5 acceptance is in [mvp_gates.md](../status/mvp_gates.md). Phase 2 interface acceptance is in [phase2_interfaces.md](../status/phase2_interfaces.md). Phase 3–10 remain documented until separately approved.
- [Todo 5 切片 B–D 验收指南](pack_eval_test_guide.md)
  （P1–P5 代码已交付未验收；B1 占据栅格 / B2 槽位求解 / B3 走廊高度 /
   B4 塞到满 / B5 零回归，含 dumps 保留验证与已知限制）
- [仿真/实机一致性审查与 pickup support 改造计划](sim_real_parity_pickup_support.md)
  （记录当前仿真特权信息风险，并规划将 pickup ROI/platform_z 从
  `scene_tf.yaml` 真值改为显式测量/标定输入）
- [无 platform_z 箱体高度估计研发 TODO](platform_free_height_eng_todo.md)
  （同帧顶面/局部支撑面估计，支持 TOP_ONLY/FULL_3D，在线节点不读仿真真值）
- [无 platform_z 箱体高度估计测试与验收](platform_free_height_test_plan.md)
  （单元、stamp/TF、故障注入、Gazebo、rosbag 和性能门槛）
- [无 platform_z 箱体高度估计修正方案](platform_free_height_remediation.md)
  （修复 ROS adapter、raw-only 安全降级、时间一致性、评测口径和性能路径）
- [Gate 5 rosbag 数据契约与就绪检查](platform_free_height_gate5_bag_contract.md)
  （PF-A2：合成 manifest 校验；通过就绪检查不等于 Gate 5 精度验收）
- [sim_world launch profile 参数说明](sim_world_launch_profile.md)
  （用一个分组 YAML 配置 launch 参数，人工改 profile 后即可调整启动组合）
- [sim_world launch profile 测试流程与验收标准](sim_world_launch_profile_test.md)
  （验证 profile YAML 生效、CLI 覆盖、异常配置报错和可选 Gazebo smoke）
- [真实 container 内部空间统一计算计划](true_container_inner_geometry.md)
  （七面 hull 统一几何核、在线候选/走廊、EMS/replay、atlas 和评测口径；
   AABB 仅保留为索引与 broad phase）
- [SIM-R1 production orchestrator 与可插拔探索计划](sim_r1_production_orchestrator_exploration.md)
  （显式 Start、人工上箱 request-ID 握手、stop-and-look NBV、策略插件硬门、
   仿真/实机同算法与持久三箱闭环）
- [D555 替换仿真 D435](d555_replace_d435/README.md)
  （已装 D555 PoE 的 Datasheet / TF / 替换讨论；官方无 D555 URDF；
   机内光学按 D450；未过 HB 与几何门之前不改架构）
- [D555 真机 bring-up 核实](d555_hardware_bringup_verification.md)
  （HB-1/2/3 只读：话题与 TF dump、RGB–深度对齐、相对 Mid-360 安装残差）
- [D455 替换腕部 D435（参考，已误导向）](d455_replace_d435/README.md)
  （按 D455 写的材料；对齐目标已改为 D555）
- [D555 眼在手标定](d555_handeye_calibration.md)
  （CAD 种子 + ChArUco 手眼；HB-3 测到约 2° 安装旋转误差；
   HE-1 全离线，HE-2 需实体标定板）
- [PF-R8 / PF-R9 感知验收](pf_r8_r9_perception_acceptance.md)
  （PF-R8 已通过；PF-R9 generation 1 仅 B3 未达，像素空间掩膜预案已触发）
- [PF-F3 depth-primary 传感契约](pf_f3_depth_primary_contract.md)
  （取消整幅点云传输，彩色对齐深度为唯一栅格；含 PF-R9/PF-R10 generation 2；
   共识第二轮未过前不可派发）
