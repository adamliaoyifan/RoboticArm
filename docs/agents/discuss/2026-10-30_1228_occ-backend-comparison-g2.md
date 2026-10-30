# 2026-10-30 -- OCC双后端最终比较与业务裁定 gen2

- status: open
- to_role: eng
- to_agent: cursor
- to_model: grok-4.7
- kind: integration
- parent: OCC-QUERY-INDEX-20261025
- subtask: OCC-BACKEND-COMPARISON
- depends_on: none
- revision: 6f75902202081ee59077faac62289400adfb627d
- generation: 2
- plan_revision: 5e287b6a0ccef42f1ded07d1a6bddffc1be04230
- dispatch_ready: no

## Post -- reviews/codex -- 2026-10-30 12:28 -- codex/gpt-6

本代替代旧比较gen1等待两侧全PASS的启动条件。最终集成消费G4-B gen4原始证据、工具与input/install manifest，不自动重测。depends_on=none用于避免任务系统将产品PASS错误设成报告前提；dispatch_ready=no仍严格禁止提前Claim。reviews在gen4最终证据清单交付（包括FAIL/预算耗尽的not_evaluated）后绑定确切工具/证据SHA再置ready。Cursor完成独立复算、来源及能力缺口核对、共同全射线与当前部署表、业务链验收/原压力结果/比较完成度/生产迁移资格四项结论。16小时是同一测量预算，不为本集成任务另加额度。禁止将OctoMap观察surface冒充planning合同、禁止静默改门或生产迁移。

## Pointers

- `docs/plans/OCC_END_TO_END_BACKEND_EVALUATION_20261030.md`

## Open

## Reply -- reviews/codex -- 2026-10-30 13:55 -- codex/gpt-6

用户暂停公平比较，dispatch_ready=no保持。先完成Q-20261030-3的OctoMap验收收口；收口不自动解锁本任务，须用户另行恢复。

- Cursor/Grok4.7最终集成任务：待gen4完成证据交付后复算双口径与四项结论；不以产品全PASS作为报告前提，当前不可领取。
