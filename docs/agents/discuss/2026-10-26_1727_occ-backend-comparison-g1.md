# 2026-10-26 -- 双后端共同观测验收与迁移决策 待依赖

- status: superseded
- to_role: eng
- to_agent: codex
- to_model: gpt-6
- kind: integration
- parent: OCC-QUERY-INDEX-20261025
- subtask: OCC-BACKEND-COMPARISON
- depends_on: G4-B-FULL-ACCEPTANCE,OCTOMAP-OBSERVATION
- revision: b33dda919521ef043fa9e30b07dcc79c87f76fce
- generation: 1
- plan_revision: bcaa882165ac72eb94b3491ba7d6fd25b1908fa2
- dispatch_ready: no

## Post -- reviews/codex -- 2026-10-26 17:27 -- codex/gpt-5

冻结bcaa8821第二计划§10。当前base b33仅谱系占位，未可执行；reviews绑定两侧PASS commit/install与共同fixture后置ready。36组共同观测输入各3warm+30，域门、各自概率oracle、更新/快照/ROI/surface延迟/CPU/RSS及业务能力差异逐项报告；不以不同概率profile硬求hash全同，不要求OctoMap必须更快。交付occ_backend_compare.py和可重放比较报告，结果不能冒充planning闭环或生产迁移授权。不得merge两条未验证分支，不改产品，不启动运动；遇到不可对齐输入记not_evaluated。

## Pointers

- `/tmp/elfin-occ-dual-backend-plan/docs/plans/OCC_OCTOMAP_OBSERVER_G1_20261026.md`

## Open

- 等待G4-B gen2与独立OctoMap均PASS后绑定两侧精确SHA，以共同观测矩阵比较完整成本/能力，不自动合并生产。

## Supersede -- reviews/codex -- 2026-10-30T12:29:00+08:00 -- codex/gpt-6

用户批准新计划 `5e287b6a0ccef42f1ded07d1a6bddffc1be04230`，本未Claim gen1由 `2026-10-30_1228_occ-backend-comparison-g2.md`（Q-20261030-2）替代，owner eng/cursor/grok-4.7。不再等待双方全部性能PASS才允许形成比较报告；待gen4交付最终证据清单后绑定确切工具/证据并启用gen2。当前gen2 dispatch_ready=no，禁止提前Claim；历史结果与本gen1要求保留。
