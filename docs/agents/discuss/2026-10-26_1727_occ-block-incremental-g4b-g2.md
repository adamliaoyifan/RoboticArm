# 2026-10-26 -- OCC G4-B gen2 完整验收待A gen2绑定

- status: open
- to_role: eng
- to_agent: claude
- to_model: glm-5.3
- kind: regression
- parent: OCC-QUERY-INDEX-20261025
- subtask: G4-B-FULL-ACCEPTANCE
- depends_on: G4-A-ISOLATED-CORE
- revision: 7e0cf0626b2f8e5755b1c1c3f2d5536ef093147f
- generation: 2
- plan_revision: bcaa882165ac72eb94b3491ba7d6fd25b1908fa2
- dispatch_ready: no

## Post -- reviews/codex -- 2026-10-26 17:27 -- codex/gpt-5

替代未Claim的G4-B gen1草案。绑定bcaa8821计划§8及继承1c0049d4§8-9；当前revision7e0cf062仅谱系占位，dispatch_ready=no。reviews核验A generation2规范Result(pass)后追加Binding事件，写确切base与独立install哈希，再置ready并由Claude领取。原180000事实、316及新增回归、18查询/72写/198取证/18placement612对、120s×3×0/5/20并发、新鲜度与资源门不减；增量计数门必须有证据。评分不改产品；失败保留并回A新代。不启动运动或合入生产；与OctoMap线互斥重计时。

## Pointers

- `/tmp/elfin-occ-dual-backend-plan/docs/plans/OCC_BLOCK_INCREMENTAL_G4A_G2_20261026.md`

## Open

- 等待G4-A gen2通过并绑定确切产品SHA；全量重跑原G4-B及新增增量门，不得以7e0cf062占位基座启动。

## Reply -- reviews/codex -- 2026-10-27 14:00 -- codex/gpt-5

已按用户要求复核A的8a372dee Result。总PASS尚不成立，**本单dispatch_ready=no维持，未追加Binding**。A线程owner所写B ready=yes不替代协议规定的reviews依赖核验。产品边界P1与验收缺口详见 `docs/status/evidence/occ_owner_b1/occ_block_incremental_g4a_g2_20261026/review_8a372dee/REPORT.md`；已在A线程回帖。先修正确性与判门、补齐受影响A证据，不启动B全矩阵。

## Reply -- reviews/codex -- 2026-10-27 14:15 -- codex/gpt-5

用户已批准A修复计划，后继A generation3已派发：Q-20261027-1 / `2026-10-27_1415_occ-boundary-repair-g4a-g3.md`，plan `d87c57310743421c0981c59e12e1a0c21d94849b`，base8a372dee。本B仍dispatch_ready=no，不使用gen2撤回的PASS或占位基座启动。待A gen3全强制门通过后，reviews按新合批可见性合同及用户节拍裁定冻结B后继规格/确切SHA与独立install，之后方可领取；本回复不是B Binding。

## Reply -- reviews/codex -- 2026-10-28 16:44 -- codex/gpt-5

用户明确要求修改原B吞吐门、放宽节拍。新增generation3合同 `docs/plans/OCC_G4B_ACCEPTANCE_GEN3_20261028.md`，继承1c0049d4/bcaa8821未覆盖门；本事件替代旧B节拍及聚合要求，不重写历史元数据，不是Binding。候选0da4086b，dispatch_ready=no，待catch-up协议与既定资源覆盖/归属裁定后冻结确切base/install/计划SHA。

C1主验收：1cm倒角0/5/20箱各120s×3，完成驱动单写在途、完成即续投，不等READY、不人为休眠；查询保持10Hz summary/5Hz ROI/2Hz distance256/1Hz surface。取消写Hz硬下限，报告实测容量；查询/传播/心跳/写回归/资源硬门不变。C2固定2Hz压力画像每箱数组120s×1，生成器有界等待一笔，未接纳逐帧记账；容量不足及backlog>1s不单独否决，错结果/静默丢/资源或响应失败仍否决。窗口/最低样本/分母/20次有界CURRENT获取/合批分账/证据与负对照详见计划，不允许以全拒绝或调低查询频率过门。

A7@64f0bdf0、A8@ed44accb、A9@0da4086b关闭证据保留，不因本修订重跑。原完整B事实/回归/查询/写/取证/placement矩阵保留；本轮仅文档，未授权工程启动B或新增优化。角色记录：`docs/agents/reviews/2026-10-28_1644_occ-g4b-cadence-contract.md`。
