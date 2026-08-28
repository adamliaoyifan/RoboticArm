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
  （残留进程：评测用独立 `ROS_DOMAIN_ID`；手动 Ctrl+C 同样会留下 gz/bridge）

Phase 1 Gates 1–5 acceptance is in [mvp_gates.md](../status/mvp_gates.md). Phase 2 interface acceptance is in [phase2_interfaces.md](../status/phase2_interfaces.md). Phase 3–10 remain documented until separately approved.
