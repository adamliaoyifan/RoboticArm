# 2026-10-31 -- S0 gen2复核：数值与缺观测路径仍阻塞S1

- role: reviews
- agent: codex
- model: gpt-6
- cli: codex
- status: done

## Summary

不开放S1。复核工具e7b8b7a7fdc2e753e2fc36d40cc014ca7282fceb，独立工作树干净。
原11测试通过，上轮5个反例均显式非pass，认可这部分修复；gen2报告的35个合成场景不等于全面满足补修合同。
新测试3个健康对照通过，但6个违例中3个误通过、3个抛异常，违反05739a10补修合同Acceptance第4条。
这只是离线工具缺陷，不是产品性能失败。产品未评分，捕获未认证。

## Findings

| 输入边界 | 实测 | 预期 |
|---|---|---|
| O8首run第三段RSS中位数NaN | pass | fail，具名非法数值 |
| 同字段-1 | pass | fail，具名非法数值 |
| 同字段true | pass | fail，具名非法数值 |
| 同字段null | TypeError | not_evaluated，缺观测 |
| O6缺surface_first_ms | KeyError | not_evaluated，缺观测 |
| O9首ROI配对缺off_ms | KeyError | not_evaluated，缺观测 |

源码机制：O8直接做RSS减除比较，未调用统一数值校验；O6/O9直接下标取必需计时字段。
独立生成输入、输出和完整复现命令已持久化到s0_g2_review；历史目录不覆盖。

## Acceptance

S0 gen3继承主计划c02c1cfbd9ba16a1c4a6c6169695f1daf8eb6a76及05739a10c9c7c3854ad7dab0a4842fce7ffea32a的Acceptance 1–8。
owner eng/cursor/grok-4.7，base e7b8b7a7fdc2e753e2fc36d40cc014ca7282fceb。
仅补齐离线工具、重放、行为测试及最近说明，不修产品、不运行ROS/observer或评分。

1. 修复表中6个违例：非法数值fail，缺观测not_evaluated，均返回具名原因且无未处理异常。3个健康对照仍pass。
2. 将同一规则覆盖O5/O6/O7/O8/O9的计时、年龄、health和资源数值入口，包括缺字段/None、NaN、正负Infinity、bool、负数；对必需序列及元素缺失返回明确非pass。不要只修本次举例的字段，也不要用顶层异常捕获伪装为验收通过。
3. 参数化行为测试逐字段验证上述非法值及缺值，完整健康集合及原11测试、上轮5反例保持正确。允许重构测试，报告实际展开用例数、覆盖字段表和预期/实测；健康与故障必须有不同判定。
4. 重放与直接判定一致，保留旧schema的显式缺观测行为，不能通过重放填默认值掩盖缺字段。阈值规则不变。
5. 独立证据目录s0_g3，保留s0_g2及review。记录完整提交、dirty状态、精确命令、逐反例结果、SHA和未覆盖项，agent_complete按实际结果关闭。未完成不得声明pass。

## Dispatch decision

将S0 gen2标superseded，保留原Claim/Result，创建同谱系gen3补修任务。
S1/S2/S3继续dispatch_ready=no。最高代S0通过并复审后，绑定实际提交开放S1产品实现；S2预演/O9和S3正式评分继续依赖。
本记录首次提交的完整SHA是gen3 plan_revision；不改写已领取的gen2要求。

## Pointers

- `docs/agents/discuss/2026-10-31_0746_octomap-2cm-repair-s0-g2.md`
- `docs/agents/reviews/2026-10-31_0746_octomap-s0-review-rework.md`
- `docs/plans/OCC_OCTOMAP_REPAIR_2CM_G1_20261031.md`
- `docs/status/evidence/occ_octomap_repair_2cm_20261031/s0_g2_review/MANIFEST.json`
- `docs/status/evidence/occ_octomap_repair_2cm_20261031/s0_g2_review/RESULTS.json`
- `docs/status/evidence/occ_octomap_repair_2cm_20261031/s0_g2_review/COMMANDS.sh`
