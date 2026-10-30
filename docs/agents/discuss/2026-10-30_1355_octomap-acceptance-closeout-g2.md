# 2026-10-30 -- OctoMap O6 O7收口与Surface有限诊断 gen2

- status: open
- to_role: eng
- to_agent: cursor
- to_model: grok-4.7
- kind: regression
- parent: OCC-QUERY-INDEX-20261025
- subtask: OCTOMAP-O6-O7-EVALUATION
- depends_on: none
- revision: 4c8f1862ec357cb01616f4b04be636227461cf44
- generation: 2
- plan_revision: 633d3fd8e9c8c25cd1b7dee3a377e79872737ff7
- dispatch_ready: yes

## Post -- reviews/codex -- 2026-10-30 13:55 -- codex/gpt-6

按OCTOMAP_ACCEPTANCE_CLOSEOUT_G2_20261030执行。先ack gen1停止和证据交接，再Claim本代。固定产品4c8f1862，逐组核平144来源、独立复算原门，追踪过载77个ID，O5仅代表覆盖。Surface两组D0/D1，每臂3预热30测量，新增诊断最多90分钟，持perf锁；分清调度/构建/转换/序列化与unknown，不预归因积分。本代只改脚本测试，插桩独立诊断树not_scored；不修产品、不换绑、不重跑全矩阵。提交完整来源账/窗口账/判官/FINAL_REPORT及Result，任务交付和产品验收分判。Q-20261030-1及Q-20261030-2暂停，禁止自动进入公平对比。

## Pointers

- `docs/plans/OCTOMAP_ACCEPTANCE_CLOSEOUT_G2_20261030.md`
- `docs/agents/discuss/2026-10-29_2038_occ-o6-o7-evaluation-g1.md`

## Open

- 用户收紧：先停止公平对比，仅收口O6/O7；核平116+28逐组来源及O7账本，保留FAIL，Surface有限诊断，不修产品不重跑全矩阵。

