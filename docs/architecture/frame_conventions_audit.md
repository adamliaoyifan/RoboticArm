# 坐标系与约定审计（2026-09-29）

两路并行审计（感知/制图侧、放置/运动侧）的合并结论。行号为当日工作树。

## 1. 实际在用的坐标系（6 个，不止 3 个）

| 帧 | 定义 | 关键值（scene_tf.yaml） |
|---|---|---|
| world | 执行/规划帧 | — |
| container_link | atlas、mapper、selector 的"map"帧 | world (1.5,0,0)，rpy 0；**deploy 场景 rpy z=π** |
| elfin_base_link | SlotSpec 契约帧 | world (0,0,0.86)，**yaw +π/2**（含基座） |
| map-floor-relative | surface_2d 高度基准 | 内底=0，箱顶=箱高；floor_z 字段恒 0.0 |
| volume-center-local | solver 候选 center_local | 体心=0，内底=−inner_h/2 |
| camera_depth_optical | 点云入口 | TF→container_link（时间戳查询） |

## 2. 高危发现（S 级：契约级歧义）

### S1 · SlotSpec.place_pose 无帧字段，三处消费方三个假设
- ComputePlacement 文档：elfin_base_link（placement_planner_node.py:16,19）
- cargo_volume_mapper `AddPlacedBox`：按 **world** 解（`_to_container`，mapper_node:208-221）
- scene_manager：按 **world** 解（scene_manager_node.py:160-175）
直通转发的调用方会把箱子放偏整个 base↔world 变换（0.86 m + 90°）。且 msg 未声明
place_pose 是箱**中心**（只在 waypoint_generator.py:294 注释里）。

### S2 · 同一个 surface_2d 契约，三个发布者三个帧
- Humble mapper：center_base=container_link [0,0,1.27]，yaw 0
- **GT dump（place_gt_dump）：center_base=elfin_base_link (−0.0,−1.5,0.41)，yaw −π/2**
  ——`[-0.0,-1.5,0.41]` 之谜的答案：容器原点在 base_link 里是 (0,−1.5,−0.86)，
  加体心 1.27 沿容器 z
- ROS1 参考节点：base_link
dict 里 `"frame": "container_local"` 标签对两个发布者是错的；混用即偏 (0,−1.5,−0.86)+90°。

### S3 · `center_base` 名不副实 + `center_local` 两种基准
- center_base 实际是 container_link（不是 base_link）；ROS1 运动过滤器按 base_link 读
- `center_local` 在 solver 候选里是体心基准（`lz=−inner_h/2+peak+h/2`），在
  place_only 评测驱动里是地板基准（`local[2]−floor_z`）——**同名字段差 0.74 m**

## 3. z 基准家族

| # | 发现 | 位置 |
|---|---|---|
| Z1 | `floor_z` 双语义：map 契约字段（恒 0.0）vs 场景绝对地板（0.53），同一名字两个量 | placement_solver:313 / solver_contact_point:164 |
| Z2 | 两处过期注释仍声称"高度是 container 绝对 z"——live 契约是地板相对；照注释"修复"即全线 −0.53 | free_space_model.py:248-250、scene_tf_config_utils.py:622-628 |
| Z3 | `atlas_contact_point` peak 默认 0（solver 候选没有 peak 键）→ 用它会查地板而非堆顶；正确版 solver_contact_point 并存 | placement_reachability.py:43-53 vs 164-175 |
| Z4 | 未知格 height=0.0 陷阱：不看 state/confidence 会当成平地板 | cargo_volume_mapper:617-622 |
| Z5 | 0.35/0.80 双常量不同基准（driver 容器系 + 绝对 0.80 兜底 vs 生成器 world 系） | motion_occ_eval_driver:206 / waypoint_generator:343 |
| Z6 | 体素顶面 vs 中心的半格偏差（height=(top+1)*res） | cargo_volume_mapper:649 |

## 4. yaw 家族（本次新发现，与 z 同等严重）

| # | 发现 | 位置 |
|---|---|---|
| Y1 | **base_link yaw 直接当容器 yaw 用**：driver `_quat_yaw`（SlotSpec base_link 朝向）→ sweep/图搜索（容器系数学）。当前被足印 π 对称掩盖；宽≠深即 90° 错 | motion_occ_eval_driver:97-103 → path_candidates:37,88 |
| Y2 | **atlas 的 yaw bin 是按 base_link 姿态采的，查询用容器/world yaw**：builder `_construct_base_pose` 只把位置映到 base，yaw 按 base 四元数施加 | reachability_atlas_builder:387-402 |
| Y3 | 候选 yaw 容器系被当 world yaw 标注（`yaw_world`），base_link 换算只减了机器人基座 yaw、没加容器 yaw——**deploy 场景（容器 yaw=π）会暴露**，且 π 差会传进 nearest_box_yaw 的腕分支选择 | placement_planner_node:276-281、scene_tf_deploy.yaml:94 |
| Y4 | π 折叠 vs 2π 环绕不一致：`snap_yaw_to_bin` 折 π，`query._nearest_yaw_index` 不折（yaw≈π 即 mismatch）；同一约定只有一半调用点遵守 | atlas_runtime:589 vs reachability_atlas:305 |
| Y5 | 放置往返的 yaw 判别死区不一致（solver 无死区 vs _placed_aabbs ±10°）→ 再规划看到的 AABB 与首次不同 | placement_solver:176 vs planner_node:417 |
| Y6 | atlas seed 链把 world 系 tool_down_yaw 直接喂容器系 atlas（与 Y1 同类） | motion_planner_node:392-398 |

## 5. 其他

- `query_offset`（容器系）与 `target_z_lift_m`（world 系）两个实机补偿分居两帧，调用点不可发现（D11）
- driver 的 `_runtime_atlas` 没包 QuerySemantics——图搜索行与 placement 注解对同一格可能判不一致（D12）
- 容器旋转只转位置不转 yaw（deploy 前masked）；mapper `_to_container` 假定 yaw-only 容器

## 6. 已验证一致（无需动）

base↔container 位置往返（全 RPY）；箱中心→接触点链（CENTER→+h/2→atlas 查询）；
executor world 帧自洽（含 align/fallback）；`suction_ceiling_z`/`column_top_map_z`/
`inner_ceiling_map_z` 全容器绝对；hull 门的重锚；TF 链（时间戳查询、无恒等假设）。

## 7. 对统一化提案的影响

审计**强化**了统一方向，且扩大了范围：不只 z——**yaw 需要同样统一**（建议容器系
yaw 为唯一内部标准，SlotSpec 契约帧显式化）。优先级排序：
1. 契约层（S1/S2）：SlotSpec 帧声明（或固定一帧）+ GT dump 发布者改容器系；
2. 语义层（Z1/Z2/Z3、Y1–Y4）：删双函数、修过期注释、atlas bin 帧对齐、π 折叠统一；
3. 数据层（z 绝对化迁移，双字段渐进）。
Y3 在 deploy（实机）场景是**活跃风险**——容器 yaw=π 时 nearest_box_yaw 会选错腕分支。
