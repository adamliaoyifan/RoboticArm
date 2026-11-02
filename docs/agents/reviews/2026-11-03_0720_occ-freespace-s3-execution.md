# 2026-11-03 -- S3完整执行清单

- role: reviews
- agent: codex
- model: gpt-6
- cli: codex
- status: done

## Summary

按用户要求一次汇总S3执行流程与验收条件，新增执行文档，不改原8caa0d44合同或14eda04a限定集成边界。Q-20261102-2保持开放，eng/claude/glm-5.3执行。明确计分前修正、主目标优先、旧矩阵独立、8h预算及真实失败收口，不增验收行、不要求修成全绿。原O6执行器每组30冷启动成本明确列出，预算不足保持not_evaluated。未运行正式样本或修改产品。

## Verification

对照原统一合同、原2cm计划、冻结freespace_contract.py、live_schedule冷启动实现与0715复核逐项汇总。固定20行来自gen4 COVERAGE.md。本文是说明和执行要求，不把尚需S3 owner提交的最终COMMANDS伪称现成可运行。

## Pointers

- docs/plans/OCC_FREESPACE_S3_EXECUTION_20261103.md
- docs/plans/OCC_SURFACE_FREESPACE_FINAL_DELIVERY_20261101.md
- docs/agents/reviews/2026-11-03_0715_occ-freespace-g4-review.md
- docs/agents/discuss/2026-11-02_0702_occ-freespace-final-claude.md
