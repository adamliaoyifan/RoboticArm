# 2026-10-31 -- OctoMap S2 gen5：真实适配器与观测无损传递

- status: open
- to_role: eng
- to_agent: cursor
- to_model: grok-4.7
- kind: regression
- parent: OCC-QUERY-INDEX-20261025
- subtask: OCTOMAP-2CM-REPAIR-S2
- depends_on: OCTOMAP-2CM-REPAIR-S1
- revision: 5d914bf71dcd821a0417281ab66bdd425ef89409
- generation: 5
- plan_revision: 44969d3868ca816e3058658a65156bfaeec4f654
- dispatch_ready: yes

## Post -- reviews/codex -- 2026-10-31 19:56 -- codex/gpt-6

执行44969d3868ca816e3058658a65156bfaeec4f654中的docs/agents/reviews/2026-10-31_1957_octomap-s2-g4-executor-review.md Acceptance 1–8，继承原主计划及gen4合同。新O9 30条trace/比值独立复核一致，保留候选7537b033证据；不要无条件重测O9。

阻塞是执行器：240条全拒绝或全unknown/无回执/100000ms经过_assemble_window都会变成健康pass，还凭空生成120条查询；正式stage开放分支仍stub，profile仅f000短样本，ROS/stability未实现实际时序采集。先移除真实组装路径healthy_*工厂，统一实测schema，逐字段传递缺失/错误；所有故障必须穿过schedule→adapter→artifact→judge拒绝。补齐真实profile/ROS/stability适配器、正式CLI路由及单调时钟总预算，失败进程非零。离线适配器验证完整路由时序，真实短接线只non_scored，缺样必须拒绝。

base=5d914bf71dcd821a0417281ab66bdd425ef89409；资源NOT_EVALUATED，S3关闭。正式O5–O8禁止执行。只有调度/组装变更可保留O9并说明兼容；产品/采集变化才重测受影响资格。新证据s2_g5，历史只读；全SHA/dirty、精确命令、逐层输入输出、逐条验收与累计预算必交。未实现runner或仍生成健康观测不得pass。agent_start领取，原owner实现测试修复提交后agent_complete按实际关闭，无Gazebo/planning/1cm/迁移/公平比较。

## Pointers

- `docs/agents/reviews/2026-10-31_1957_octomap-s2-g4-executor-review.md`
- `docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_g4_review/COMMANDS.sh`

## Open

- 补真实runner/正式路由，禁止healthy工厂生成实测结果；拒绝全拒绝/unknown窗口，保留未受影响O9，S3关闭。
