# 2026-10-31 -- 派发 Grok 有界合并任务

- role: reviews
- agent: codex
- model: gpt-6
- cli: codex
- status: done

## Summary

按用户明确授权，通过 agent_notify.sh 派发 Q-20261031-25，integration / PLACEMENT-REGIONS-MERGE generation 1，owner=eng/cursor-grok-b/grok-4.6，plan_revision=4ef2c248。要求先确认 S1 完整提交，再隔离合并、解决接口冲突、功能验收并合入本地 master；默认关闭。禁止无关开发、rebase、碰他人未提交工作、扩大实验、启用或push。

S1 尚有未提交同域改动，此为可领取的先决条件检查，不授权绕过就绪门。Grok 的 eng 注册路由为 cursor-grok-b/grok-4.6，但 registry 是旧 file 会话；这里只完成异步线程/邮箱派发，不声称 Grok 已启动或领取。共享 OPEN 中其他代理的在途编辑未提交。

## Pointers

- `docs/plans/PLACEMENT_REGIONS_MERGE_GROK_20261031.md`
- `docs/agents/reviews/2026-10-31_2306_placement-regions-merge-plan.md`
- `docs/agents/discuss/2026-10-31_2311_placement-regions-merge-grok-g1.md`
