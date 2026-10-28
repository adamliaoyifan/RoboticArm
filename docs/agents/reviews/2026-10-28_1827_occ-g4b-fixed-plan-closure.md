# 2026-10-28 -- G4-B固定计划反例关闭

- role: reviews
- agent: codex
- model: gpt-5
- cli: codex
- status: done

## Summary

定向复核93ef25e7：独立31测通过（0.02s），变更仍仅scripts核心及专用tests。early_start/attempt_before_start与重复计划拒绝有机器和判官双层实现，接受前轮两个具体反例关闭，不再要求重复该组审查。Binding保留，Stage2真实进程/资源仪表准备继续；不是评分准入。

## Acceptance

评分脚本冻结时需处理两个阈值口径，均为原合同一致性而非新产品门：新增SCHEDULE_TOLERANCE_S=1.0在冻结计划中无依据，不将其解释为产品新增通过/失败门；实际scheduled/start差保留为诊断值。如运行器保留此值作为仪表调度异常护栏，应在准备manifest显式登记，触发只说明日程失真/测量not_evaluated，不判产品性能失败、不择优重跑。

RETRY_TOLERANCE_S=0.005仍可把50ms重试提前到45ms；前轮已明确不批准提前重试豁免，接线阶段改为不早于排定next_retry（仅数值舍入精度，不用毫秒业务宽限），补45/50ms边界行为测试即可。不增加性能测量或另开优化单。正常OS晚唤醒允许原样记录，2s总期限不延长。

## Pointers

- `docs/plans/OCC_G4B_ACCEPTANCE_GEN3_20261028.md`
- `/tmp/elfin-occ-g4b-gen3/scripts/g3/occ_g4b_capacity_core.py:137`
- `docs/agents/discuss/2026-10-26_1727_occ-block-incremental-g4b-g2.md`
