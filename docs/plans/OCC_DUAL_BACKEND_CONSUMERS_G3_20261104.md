# 双后端使用方能力矩阵 G3

- date: 2026-11-04
- status: inventory_seed_not_certification
- plan: docs/plans/OCC_DUAL_BACKEND_COMPARISON_G3_20261104.md
- grid: a4ac889e
- octomap: cbdd5364
- common_consumers: cbdd5364

本表为基于源码的初始清单，不把存在代码等同于运行通过。实现者依据固定版本安装/launch/调用图填实际入口和测试证据；活跃节点零遗漏，ROS1未安装参考明确排除。消费者版本两侧必须一致。

| 使用方/入口种子 | 所需能力/合同 | G现状线索 | O现状线索 | 必须核验/初始状态 |
|---|---|---|---|---|
| luggage_packing/scripts/placement_planner_node.py，ComputePlacement/ComputePlacementV2 | surface_2d、SurfaceContractV2、committed、固定快照身份、候选合法性 | 订阅surface_2d/committed，store投影 | free-space/Surface桥接与reset admission已实现 | 共同C真实订阅+服务，正确读入目标图；待测 |
| luggage_packing placement_solver及其known-free验证调用链 | 支持高度、完整候选ROI、unknown阻断、hull/间隙 | grid表面和查询栈 | uld_occ_observer/freespace_query、ROI/Surface | 同solver/同体积判官，两后端适配不能偷偷生成free；待测 |
| luggage_planning/luggage_planning/occupancy_place_paths.py，from_surface_2d | container_local、height datum、geometry_descriptor、占用约束 | 实际解析上述字段 | 桥接需显式提供语义 | 独立几何场景及真实planning调用边界；待测 |
| luggage_planning/luggage_planning/hardware_place_planning.py | surface/目标身份、地图时效、规划结果接纳 | 实际map接口引用 | validity board/placement_admission可供接入 | 离线固定机器人样本，不发硬件命令；入口/调用图待核 |
| luggage_packing/luggage_packing/placement_admission.py及其实际节点调用者 | INVALID/BUILDING/VALID、generation/epoch、新图+新机器人样本、提交前复核 | 是否有等价失效信息需核 | reset合同已实现并验收 | 正常/失效/恢复都经过真实消费路径；G适配待核 |
| waypoint_generator_node / motion_planner_node / place_prior_node / trajectory_selector_node | surface、committed或GetCargoVolumeV2相关引用 | 搜索发现源码引用 | 是否安装/启动及ROS2移植状态待核 | 先判活跃/ROS1参考；仅活跃者纳入10状态；禁止按文件名推定已运行 |
| cargo mapper写服务的真实调用方 | add/remove/verify/geometry锁/实例及reset副作用 | store及ledger原有功能 | 不能用空ledger模拟有箱事实 | 完整调用图登记；若活跃且O缺失，兼容BLOCKED，独立公共读比较可继续 |
| 查询读者/状态监控/距离查询的其他实际调用方 | acquire/read/release、分页、freshness、frontier/ROI/distance、错误码 | grid query registry及legacy facade | O服务/句柄或对应API | 依安装与运行入口补全行；可选未使用接口可N/A，必需项不能unsupported当PASS |

每行最终增加：active证据、consumer文件+函数、G/O准确接口映射、10状态case IDs、结果、共享/后端特有成本、缺口修复SHA。不新增与实际使用方无关的算法要求。纯测试工具、RViz显示、ROS1参考及未启用实验不冒充生产消费者。
