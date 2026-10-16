# 方案 B：OccupancyMap 独占地图写入与状态转换（规格 + 实施状态）

状态（2026-10-16）：**B1 已落地**（mapper 事务 + 快照合同 + 离线合同门可测行
全绿）。B2 消费者单源迁移、B3 执行失效检查、B4 完整流程门待派单。
基线分支：`agent/eng/master-pack-baseline-g3`。

## 规格准绳（用户 2026-10-16 交付，B 波次的验收依据）

OccupancyMap 独占地图写入和状态转换，以不可变快照贯穿 placement、路线搜索、
认证与执行；地图变化时明确旧路线能否继续使用。首版覆盖静态、事件式装箱；
动态障碍在线避障另立任务。

1. **类边界**：OccupancyMap 唯一写入者（占用/支撑/确认态/版本/变化记录）；
   感知整合提交带时间、TF、覆盖范围与来源的观测；Select/认证器只读快照、
   输出绑定快照身份；执行协调器上报释放等事实事件并检查执行资格；消费者
   缓存只保存快照引用。Select 决定"计划放哪"，观测说明"实际在哪"，mapper
   决定"哪些占用已确认"。驱动本地 placed 列表仅展示，规划输入必须来自
   mapper。
2. **状态与版本**：SnapshotIdentity = epoch + revision + content_digest；
   digest 覆盖规划相关内容及确认态（柱高变化可检测）；同内容重复观测不使
   路线失效；时间流逝不失效。状态机：未确认→有效观测→已确认；已确认→
   释放/移除/碰动/容器位姿变化→待确认；待确认→覆盖变化区域的有效观测→
   已确认。遮挡≠空闲证据；移除需覆盖该区域的空闲观测确认。
3. **写入事务**：统一内部 apply_update()，现有 ROS 服务作为适配器。同一
   operation_id 同内容重试只生效一次、同 id 异内容拒绝；生产确认要求可关联
   证据（缺失/非有限拒绝、容差服务端定）；版本冲突不许自动改最新重发，必须
   重读检查前提；释放等事实事件迟到不丢弃——记 pending 阻止"确认未变"声明。
4. **快照与缓存**：最新快照 + 活跃任务持有快照；原子替换；旧消息不覆盖新
   版本、同身份异内容报协议错误；surface/ledger/确认态同源同时提交；可配置
   上限、不淘汰在用数据；碰撞柱缓存键含机器人模型/载荷/阶段/ACM。
5. **规划与执行**：placement/Select/认证共用一个冻结快照、输出保留身份；
   搜索中变化→取消、迟到结果提交前拒绝；两段之间→暂停提交、新快照重规划；
   段执行中→标失效阻后续段。检查在执行提交的串行入口。本箱释放显式阶段
   转换（carrying/unloaded 双场景、释放事件绑定本次执行）；下一箱始终等待
   占用确认。
6. **验收三道门**：离线合同门（行为矩阵全过、并发两序+同时×20 零违反）→
   进程集成门（真实 ROS 消息、发布/重试/重启/版本一致，每场景×3）→ 完整
   流程门（冻结代码、预登记 n=2 功能见证；只证一次链路，不签稳定性）。
   每次保存请求、前后快照身份、证据关联、更新结果、路线与场景身份、逐段
   提交记录、版本/安装摘要。

## B1 落地内容（2026-10-16，提交 816ac464→79f475c6）

- `occupancy_map_store.py`（新，ROS-free）：`OccupancyMapStore` 包装
  CargoVolumeMapper + PendingConfirmation，唯一变更入口 `apply_update`
  （幂等账/逐腿前提/服务端锚门/迟事实记 pending/mapper 校验转拒绝）；
  `MapSnapshot` 冻结 2.5D 投影+账本+确认态+来源；content_digest 覆盖每列
  (state, confidence, 量化高度)+排序后账本几何+确认态，排除修订/epoch/时间
  戳/提交顺序（重观测 digest 不变、乱序收敛、柱高/来源/确认变化可检测）。
- mapper 三处手术：`map_revision` property、`note_external_change()`、
  `fill_unoccupied_as_free` 补计量。
- 线合同增量：SurfaceContractV2 + epoch/content_digest/occupancy_confirmed/
  change_event（空 epoch=未声明，64 例序列化矩阵字节兼容）；5 个 srv 增
  operation_id/expected_epoch/expected_content_digest 与 status/reason/epoch/
  content_digest 回显。
- node 六端点适配器化 + `_publish_all` 单快照投影（v1 JSON/committed/
  surface_v2 同源），顺带修 `commit_revision` 恒 1 下游 bug；stamp 去重先于
  幂等账（重复物理采集=STAMP_ALREADY_INTEGRATED）。
- **离线合同门（B1 可测行）**：65 项离线 + 12 项 live 全绿——缺证据 NaN/负
  容差拒绝、重复与并发（同 id 竞争/冲突前提/A→B vs B→A，各×20）、重启
  （双实例 EPOCH_MISMATCH）/reset 轮换 epoch、pending 与观测（含遮挡≠空闲、
  时间不失效）、快照冻结、缓存上限与淘汰安全、生产一致（三投影同身份+
  commit_revision 修复）。live 套件增 op-id 重试一次、reset→EPOCH_MISMATCH。
- 回归：gazebo+perception 零新增失败 ID（对照 stash 基线）；planning 2
  存量 atlas flake；description 4 存量。
- 证据：docs/status/evidence/occ_owner_b1/（Gate-1 矩阵行记录）。

## B2 待派：消费者单源迁移

- pack/place_only/motion_occ 驱动双缓存→单快照引用；4 份本地 placed 账收敛
  为展示用途，ComputePlacementV2 的 placed 输入改由 planner 侧快照供
  （R3/R4）。
- trajectory_selector 的 v1-latch 覆盖 bug（`_select_v2` 用自家 v1 副本覆写
  请求 v2 surface 的 occupancy_confirmed——两源一判）→ 读 v2 快照字段。
- placement_planner/motion_planner/waypoint_generator 迁 v2 快照（Occupancy-
  Snapshot.from_surface_2d → v2）；v1 JSON 降为确定性派生导出。
- Select 请求携带完整快照身份 + route_producer `_same_admitted_map` 换
  epoch+digest 栅栏；Select换槽/路线失效/跨节点生产一致/正常释放（释放需
  observed-free 确认规则，含 footprint 携带的 disturbance 事件）四行矩阵补
  齐。
- 碰撞柱场景缓存键补机器人模型/载荷/阶段/ACM（moveit_edge_adapter 的
  SceneState marker 已含前三）。

## B3 待派：执行失效检查接入

- place/pack 路径 wiring `_execution_admission`（参考 motion_occ_eval_driver
  :298-390：Select 后 verify_selected→leg admissions→plan_motion；handoff/
  transition/executor 的既有闸门随即生效）。
- 三时机失效：搜索中（_revalidate 已有）、两段间（handoff admission 轮询
  已有）、提交边界（motion_planner `_admission_still_live` 已有）——补
  "检查后提交前"窗口收窄到串行入口。
- 释放绑定执行实例（TransitionAdmissionPhase 已实现 admission 侧）+ 下一箱
  等待占用确认门（驱动侧新 gate，超时策略待定）。

## B4 待派：完整流程门（需用户授权 n=2 见证）

冻结代码与安装配置 → 预登记一次 n=2 功能见证：两箱均完成放置、退出、确认、
下一箱交接。保存全程证据束。性能优化与局部重规划排在此一致性链之后。

## 决策记录（不变）

- 不合并感知与 placed 校验成一个大接口（数据性质不同，分 kind 各自强度）。
- 不引入 pull 式 GetSurface(at_revision)（mapper 重启时规划不可用的新故障
  模式）；栅栏先行，pull 最后再议。
- 不动 60s/2s 预算、认证门、handoff 策略（RTF/时钟问题另行处理）。
- B1 幂等语义：同 id 重试在 reset 后仍返回 ALREADY_PROCESSED（幂等优先于
  前提重放——跨 epoch 重放旧写是错误，调用方须用新 op id）。
- v1 语义保留：v1 端点合成内容导出 op id；重试报成功（地图含箱即契约）。
