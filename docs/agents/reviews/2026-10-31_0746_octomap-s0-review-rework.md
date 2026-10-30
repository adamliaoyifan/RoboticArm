# 2026-10-31 -- OctoMap S0审查：不开放S1，判官离线补修

- role: reviews
- agent: codex
- model: gpt-6
- cli: codex
- status: done

## Summary

用户要求查看S0结果并决定是否开放S1。本次裁定：**不开放S1**。
S0 gen1 Result为done/pass，工具bf388e31a6009ff9f6ad003d4f71f510fc3d473e，独立工作树干净；
原9个pytest用例复跑9/9通过，计划96组manifest已交付，产品未改、实机评分未运行。
但本次5个独立负例全部被判为pass，不能满足已批准的逐门判官前置。
这不是产品性能失败，也不要求新增1cm或ROS负载。

## Findings

| ID | 可重放反例 | 直接观察 | 源码机制 |
|---|---|---|---|
| R1 | recovery 60输入中第2帧明确QUEUE_FULL拒绝 | pass | recovery仅拦全拒绝，未要求60帧全部完成 |
| R2 | normal第2查询替换成第1查询，120条中重复一个ID、缺一个计划ID | pass | normal只检查条数与OK，未比对计划ID集合 |
| R3 | O6同一合法组重复96次 | pass | campaign只检查数量与各子组判定，没有验证计划集合、唯一性及参数身份 |
| R4 | normal首个查询covers_observation=false | pass | 正常查询没有审查提供的快照正确性标志；错误内容/身份不能仅凭OK通过 |
| R5 | recovery首查latency_ms=NaN | pass | 只做数值类型和上界比较，NaN绕过上界；缺有限值验证 |

证据仅支持上述反例和源代码机制，不声称完整审查O8/O9或证明实际产品故障。
现有21条合成排练能重放其自身判定，不足以证明判官对合同违例具有区分能力。

## Acceptance

本记录作为S0 gen2补修合同，继承c02c1cfbd9ba16a1c4a6c6169695f1daf8eb6a76全部要求，
owner仍为eng/cursor/grok-4.7，base改为bf388e31a6009ff9f6ad003d4f71f510fc3d473e。
仅修改S0离线判官/重放/测试及最近工具说明，不修产品、不启动observer/ROS、不评分。

1. 正常和恢复输入逐ID要求完成，任何单帧拒绝都非pass；过载仍允许明确拒绝，不错误收紧过载合同。
2. O5/O6组和O7窗口、输入、查询按冻结manifest的唯一计划集合核对。重复、缺失、替换为陌生ID、参数不匹配不得通过；96个重复组不是96组覆盖。
3. 正常与恢复查询校验同一链的快照身份、epoch/版本和内容证明。显式false或缺必需字段不得通过；提供真实适配器字段前仍不得认证捕获。
4. 所有计时/年龄/health/资源数值统一验证类型、有限性和非负；NaN、正负Infinity、bool、负数、缺字段返回具名非pass，不抛未处理异常。明确错误为fail，缺少所需观测为not_evaluated；健康完整样本仍通过。
5. 每个R1–R5至少一对健康/违例行为用例；扩展覆盖O5重复4组、O7重复9窗、缺样/陌生ID/参数错配、缺身份、数值边界。阈值恰好相等按原合同通过；health恰好1秒不通过。不可仅检查源码文本。
6. 原9个测试保持通过，更新合成ledger必要字段后重跑完整离线套件；5个复现反例均返回显式非pass、无异常。合法的完整4组/96组/9窗健康集合均通过，不用全拒绝掩盖正常合同错误。
7. 重放负例与直接判定一致；修改schema时保留旧证据，明确schema版本及迁移/拒绝规则。重放不得默默补齐缺失字段或覆盖错误标志。
8. 新证据写docs/status/evidence/occ_octomap_repair_2cm_20261031/s0_g2/，保留gen1及s0_review。
   提交工具后记录完整SHA、dirty状态、精确命令、用例数、逐反例预期/实测与剩余限制。
   owner实现/测试/修复/提交端到端完成。结果仍区分工具通过、捕获未认证、产品未评分。

## Dispatch decision

S0 gen1已完成，无在途Claim需要停机；保留原Claim/Result，标superseded并链接S0 gen2。
S0 gen2使用本记录首次提交的精确SHA作为plan_revision；S1现有gen1保持dispatch_ready=no，
S2/S3同样不开放。只有最高代S0补修通过后才重审S1实际提交绑定。
不以历史done/pass绕过本次复现缺陷。

## Pointers

- `docs/plans/OCC_OCTOMAP_REPAIR_2CM_G1_20261031.md` @ `c02c1cfbd9ba16a1c4a6c6169695f1daf8eb6a76`
- `docs/agents/discuss/2026-10-31_0035_octomap-2cm-repair-s0-g1.md`
- `docs/status/evidence/occ_octomap_repair_2cm_20261031/s0/FINAL_REPORT.md`
- `docs/status/evidence/occ_octomap_repair_2cm_20261031/s0_review/RESULTS.json`
- `docs/status/evidence/occ_octomap_repair_2cm_20261031/s0_review/reproduce.py`
- `docs/status/evidence/occ_octomap_repair_2cm_20261031/s0_review/MANIFEST.json`
- `bf388e31a6009ff9f6ad003d4f71f510fc3d473e:src/uld_occ_observer/test/benchmark/repair_s0_judge.py`
