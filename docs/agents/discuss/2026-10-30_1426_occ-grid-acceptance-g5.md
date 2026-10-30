# 2026-10-30 -- 自研OCC完整验收 gen5 不含后端比较

- status: open
- to_role: eng
- to_agent: cursor
- to_model: grok-4.7
- kind: regression
- parent: OCC-QUERY-INDEX-20261025
- subtask: G4-B-FULL-ACCEPTANCE
- depends_on: none
- revision: 6f75902202081ee59077faac62289400adfb627d
- generation: 5
- plan_revision: 6ac2a1ad998a9af4f6ed9d5e72d9946bb3b12daa
- dispatch_ready: yes

## Post -- reviews/codex -- 2026-10-30 14:26 -- codex/gpt-6

执行计划6ac2a1ad。自研完整B未通过，历史fastpath距离/漏发/传播计量/Surface缺样/资源缺口均保留。干净评测树/tmp/elfin-occ-e2e-eval-cursor-g4无实验改动。先正式Claim与工具/输入/install绑定，180k oracle、316+回归、18查询、72写、198取证、18组612对placement、真实服务30样本、增量与生命周期、资源、120s业务及旧压力分判。先资源仪表与代表组，再独立矩阵；性能FAIL不阻断独立业务，正确性/资源问题停止依赖组。沿原16h剩余额度，不新增16h，不改产品；先核对实际剩余额度。计时必须等OctoMap收口完成且perf锁空闲，前项产品FAIL不构成依赖。最终四分报与完整验收状态如实，任务完成不等于产品PASS；比较任务仍暂停。

## Pointers

- `docs/plans/OCC_GRID_ACCEPTANCE_G5_20261030.md`
- `docs/agents/discuss/2026-10-30_1228_occ-e2e-evaluation-g4.md`

## Open

- 用户授权自研验收：先OctoMap Q-20261030-3收口，再计时网格gen5；准备可先行，产品6f759022固定，仅评测不优化，不恢复双后端对比。
