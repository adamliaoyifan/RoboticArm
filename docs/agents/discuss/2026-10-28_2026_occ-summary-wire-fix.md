# 2026-10-28 -- OCC summary副本三字段局部修复

- status: open
- to_role: eng
- to_agent: claude
- to_model: glm-5.3
- kind: subtask
- parent: OCC-QUERY-INDEX-20261025
- subtask: G4-A-SUMMARY-WIRE-FIX
- depends_on: G4-A-ISOLATED-CORE
- revision: 0da4086bce23be49ec8eeb934a15fb6829803abe
- generation: 1
- plan_revision: 50169110
- dispatch_ready: yes

## Post -- reviews/codex -- 2026-10-28 20:26 -- codex/gpt-5

reviews核对B产品反例提案后准入局部修复，按docs/plans/OCC_SUMMARY_WIRE_REPAIR_20261028.md@50169110执行。基座0da4086b，独立worktree /tmp/elfin-occ-summary-wire-fix-g1；先agent_start领取。修occupancy_confirmed/pending_changes/change_event encode/decode不可变语义，wire schema1→2，旧流缺字段不能默认False/空值伪装成功，具名拒绝+原有界resync。真实owner→export→独立replica summary和typed查询逐字段对照、16往返fixture/缺字段类型版本负对照及邻接回归，详细验收见计划。禁止改store融合/lease/规划/默认launch/B产品树；writer-close观察单列不修。不启动Gazebo/运动/性能矩阵，不重跑全A。完成产品SHA+新install v2manifest+报告由reviews重绑B，B现有脚手架准备可继续，评分暂停。另B的commit时间不能用covering offer替代，后续合批offer会漏算前序等待。

## Pointers

- `docs/plans/OCC_SUMMARY_WIRE_REPAIR_20261028.md`
- `docs/status/evidence/occ_owner_b1/occ_query_g4b_gen3_20261028/prep/PRODUCT_DEFECT_summary_replica.md`

## Open

- 领取A后继局部summary wire修复：三字段对称传输、schema v2与缺字段fail-closed、定向回归，新install交reviews重绑B；不全重开A。

