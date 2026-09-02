# Plans

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
