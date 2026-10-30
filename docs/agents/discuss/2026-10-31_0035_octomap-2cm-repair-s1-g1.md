# 2026-10-31 -- OctoMap 2cm S1 产品修复与兼容接口 gen1

- status: open
- to_role: eng
- to_agent: cursor
- to_model: grok-4.7
- kind: subtask
- parent: OCC-QUERY-INDEX-20261025
- subtask: OCTOMAP-2CM-REPAIR-S1
- depends_on: OCTOMAP-2CM-REPAIR-S0
- revision: 57d8c0bfe23fee54920c89c06a2b6bdcb97de976
- generation: 1
- plan_revision: c02c1cfbd9ba16a1c4a6c6169695f1daf8eb6a76
- dispatch_ready: no

## Post -- reviews/codex -- 2026-10-31 00:38 -- codex/gpt-6

用户已批准本计划并指定eng/cursor/grok-4.7端到端执行。精确计划c02c1cfbd9ba16a1c4a6c6169695f1daf8eb6a76，工具谱系基线57d8c0bfe23fee54920c89c06a2b6bdcb97de976、原产品4c8f1862ec357cb01616f4b04be636227461cf44。完成计划A-D：已有快照读不等store锁，入口有界串行适配与独立回调、逐ID终态账、GetObservationStatus、WaitSnapshot、GetResourceUsage。保留概率/射线/单线程与旧立即查询语义。锁持有注入、reset/TTL/release、重复与拒绝、等待超时/epoch变化、资源所有权行为测试全部通过；不启动正式评分。

遵守计划第6节任务绑定。S0从工具基线创建独立工作树；后续在前置done/pass及精确Result提交绑定后继续，不复制主工作区未提交改动。共同证据根docs/status/evidence/occ_octomap_repair_2cm_20261031/；本阶段提交代码/工具、最近合同、测试、角色笔记和Result，包含完整SHA、dirty状态、实际命令与证据指针。领取与关闭使用agent_start.sh/agent_complete.sh；下游当前dispatch_ready=no，不绕过依赖。

不新增1cm测试、不接planning、不启动Gazebo、不迁移生产或解锁公平比较。旧接口取证gen2保持自身离线边界；历史FAIL不覆盖。新恢复等待合同只作用于新评分，旧GetSnapshot的合法STALE保留。失败证据持久化后才可拆卸/重跑；计分期间不改受验产品。

本次通过文件异步派发，未声称Cursor已领取；前置完成后在Result中列出下阶段交接的精确提交与开放请求。

## Pointers

- `docs/plans/OCC_OCTOMAP_REPAIR_2CM_G1_20261031.md`
- `docs/plans/OCC_RESOLUTION_TARGET_2CM_20261030.md`

## Open

- 执行批准计划c02c1cfb S1：产品修复与兼容接口。等待OCTOMAP-2CM-REPAIR-S0提交并由reviews绑定实际基线后开放，当前不可领取。
