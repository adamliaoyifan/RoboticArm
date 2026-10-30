# 2026-10-30 -- OCC全链路评测与双后端矩阵 gen4

- status: open
- to_role: eng
- to_agent: cursor
- to_model: grok-4.7
- kind: regression
- parent: OCC-QUERY-INDEX-20261025
- subtask: G4-B-FULL-ACCEPTANCE
- depends_on: none
- revision: 6f75902202081ee59077faac62289400adfb627d
- generation: 4
- plan_revision: 5e287b6a0ccef42f1ded07d1a6bddffc1be04230
- dispatch_ready: yes

## Post -- reviews/codex -- 2026-10-30 12:28 -- codex/gpt-6

批准计划5e287b6a为唯一新合同。产品网格6f759022、b33基准、OctoMap4c8f1862；干净评测树/tmp/elfin-occ-e2e-eval-cursor-g4已建立，工具从eefb4822及现有OctoMap工具定向移植，产品不改。先agent_start正式Claim、确认旧B停止并与已Claim O6/O7串行，再准备/自测/冻结COMMANDS.sh、输入和install后评分，无需重复请示同一授权。144组/后端全射线、36组/后端部署及冷启动、18组真实选位、持续业务与旧压力、继承正确性回归；所有门/停止规则见计划。旧distance50ms压力失败不阻止业务比较；正确性错误停止依赖配置。证据根docs/status/evidence/occ_owner_b1/occ_end_to_end_compare_20261030/。提交工具、输入manifest、原始事件和判官、四项结论及最终证据清单；不把任务归档写成产品全PASS。最终比较另有gen2集成任务，凭完整证据启用。当前dispatch_ready=yes仅表示准备/执行授权，不代表工具/install已冻结或验收通过。

## Pointers

- `docs/plans/OCC_END_TO_END_BACKEND_EVALUATION_20261030.md`

## Open

- 用户已批准：Cursor/Grok4.7领取gen4，准备工具并冻结后执行OCC到真实选位及双后端矩阵；16小时测量预算，业务/压力分判，不启动运动。
