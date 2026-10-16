# Plan B TODO: occ-owner 重构（保留待派单，未实施）

状态：设计备忘。方案 A（commit 单一信息源 + verify 锚门）已落地并全绿；
本文件是剩余结构债的清单，供后续单独授权。基线：agent/eng/master-pack-baseline-g3
（方案 A 合入后）。

## 目标形态

1. occ 写入单一服务入口 `/cargo_map/update`，typed record：
   `{kind: sensor_view|placed|removed|reset, payload,
     provenance: {source_id, stamp, expected_revision, geometry_hash,
                  route_or_verify_id}}`；update() 按 kind 分派校验，
   感知路径保留现有严格准入（stamp/新鲜窗/去重），placed 路径保留
   方案 A 的 verify 锚门。内部 `CargoVolumeMapper.mark_*` 已经是
   "只能经 update 修改"的形状，改的是服务层，不是类。
2. 读取契约（revision 栅栏）：
   - 发布只保留一条权威通道（v2 契约为准；v1 JSON 由 v2 确定性导出或废弃）；
   - Select 请求声明所基于的 map_revision；
   - 执行侧把现有 map_admission（终点一次）扩成每腿 admission 校验
     revision 未前移。
3. 驱动侧 placed 账本收敛为 1 份，来源改为订阅 mapper ledger 主题。

## 已核查的冗余/读取契约问题清单（B 的工料）

| # | 位置 | 现状 | 风险 | B 的处置 |
|---|---|---|---|---|
| R1 | pack 驱动 `_surface_2d` + `_surface_v2_msg` | 同进程两份 surface 缓存（v1 JSON + v2 契约各自订阅） | 两份可停在不同 revision；`_frame_on_detected` 已用 v2 优先、v1 兜底的合并读取 | 单通道化后消除 |
| R2 | placement_planner_node | 同样双订阅两通道（:266/:270） | 与驱动各自的快照分歧 | 同上 |
| R3 | Select 请求内嵌 surface | 请求携带驱动快照副本 → producer `occupancy_boxes_from_surface`（route_producer.py:357）→ 认证场景，共 5 级副本 | req-2b 实证：bin 落后 fixture 一个 revision（commit→请求 freshness 竞态） | revision 栅栏 + 执行期校验 |
| R4 | 驱动 4 份 placed 账 `_frame_placed_records`/`_committed_ledger_boxes`/`_placed_slots`/`_placed_records` | 同一 commit 块内同步 append（pack:596/606/608/609），4 个读者，永不对账 | 账本漂移；compute 的 `placed=` 参数吃本地副本而非 mapper ledger | 收敛为 mapper ledger 单源 |
| R5 | v1/v2 双发布 | `_publish_all` 同点双发（mapper_node:625-654） | 无写侧竞态，但读侧每通道一份缓存 | v2 权威、v1 派生 |
| R6 | MoveIt 规划场景 | 产线单写者（moveit_edge_adapter apply_diff + _verify_scene 回读 + marker 缓存）；world_objects 同步只在 G2 harness（isolated_moveit_worker） | harness≠产线差异已被 witness 根因证实会掩盖产线缺陷 | harness 注明与产线的分叉点，或改喂产线同源 |
| R7 | gz 物理世界 vs 地图 | verify 实测不回写（方案 A 只在 commit 时一次性使用）；**2026-10-16 补**：锚定 commit 已在 ledger 落 `verify_anchor{request_id, delta_m, tolerance_m}` 审计踪迹，mapper 端拒非有限/超顶(0.30m)容差 | 坐标仍是调用方自述（无 verify 记录身份/箱体/时间关联），感知缺失时地图无二次锚 | placed ledger 升级为持久 measured anchor + verify 会话身份绑定 |

## 不做的事（决策记录）

- 不把感知与 placed 的校验合并成一个大接口：数据性质不同（实测 vs
  断言），分 kind 保留各自强度。
- 不改 pull 式读取（GetSurface(at_revision)）：引入 mapper 重启时规划
  不可用的新故障模式；栅栏先行，pull 最后再议。
- 不动 60s/2s 预算、认证门、handoff 策略（RTF/时钟问题另行处理，
  见 verdict §5）。
