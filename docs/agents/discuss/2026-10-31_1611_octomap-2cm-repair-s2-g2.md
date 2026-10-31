# 2026-10-31 -- OctoMap S2 gen2：绑定S1 gen3，预演与捕获资格

- status: open
- to_role: eng
- to_agent: cursor
- to_model: grok-4.7
- kind: subtask
- parent: OCC-QUERY-INDEX-20261025
- subtask: OCTOMAP-2CM-REPAIR-S2
- depends_on: OCTOMAP-2CM-REPAIR-S1
- revision: bfece5c32a0b4595827dd10993b88d4638c4f106
- generation: 2
- plan_revision: c02c1cfbd9ba16a1c4a6c6169695f1daf8eb6a76
- dispatch_ready: yes

## Post -- reviews/codex -- 2026-10-31 16:11 -- codex/gpt-6

S1 gen3 done/pass，reviews独立重建行为测试、原资源probe和S0 12测试均通过；owner在线22项及二进制hash已核对，本次未重复在线测试。按原主计划c02c1cfbd9ba16a1c4a6c6169695f1daf8eb6a76第6节绑定base=bfece5c32a0b4595827dd10993b88d4638c4f106，S2升gen2，依赖S1最高代gen3。审查记录docs/agents/reviews/2026-10-31_1611_octomap-s1-g3-review-s2.md（be5887ab）。

领取后先执行原S2非计分预演：启动/就绪/TF/独立发送/终态守恒/恢复/真实故障捕获和重放，2小时预算不含等锁。必须先解决审查交接的预演项：以可控交错验证wake_waits局部pending仍计入最多8个在途、第9个WAIT_PRESSURE；若违例先修复提交回归；客户端4线程，评分禁用行为hook；修正bash执行环境、库路径和可复现构建。资源缺项仍NOT_EVALUATED，列覆盖表及对后续资源门的阻塞，不用部分统计宣称通过。

预演通过后按原O9门：ROI/30k融合/首次Surface各30对ON/OFF，AB/BA交替，P95比≤1.10，无缺失错配，全部真实注入失败可重放。记录实际RMW/DDS、CPU/频率/线程/RSS/外部负载；持/tmp/elfin-occ-backend-perf.lock串行测量，等其他负载，不终止他人。首次Surface不得用缓存命中代替。

冻结产品/工具/配置、最终输入manifest/hash、安装产物hash、精确环境和无占位符COMMANDS.sh（构建→行为→预演→O9→O5→O6→O7→O8→判官→重放；本阶段只执行至O9）。一旦资格测量开始不修改受验候选；失败先保存，需修复则结束旧候选提交新版本并重跑受影响门，不跨版本拼成绩。T0/T1、T2窗口与完整重建前缀、capture_complete/replay_possible/缺失项/SHA及原大小预算遵守主计划；预算耗尽明确not_evaluated。

沿用独立工作树，协调仅主工作区docs/agents。agent_start.sh领取，owner端到端实现/测试/修复/提交并agent_complete.sh按实际关闭；证据s2新目录不覆盖历史。S3保持关闭，S2不能启动O5–O8正式评分，也不新增1cm/Gazebo/planning/生产迁移/公平比较。交接完整候选SHA/dirty、逐项状态和未覆盖阻塞，由reviews再绑定S3。

## Pointers

- `docs/plans/OCC_OCTOMAP_REPAIR_2CM_G1_20261031.md`
- `docs/agents/reviews/2026-10-31_1611_octomap-s1-g3-review-s2.md`

## Open

- 从bfece5c3执行S2预演，先验证等待并发上限并修复；通过后O9及冻结。资源仍NOT_EVALUATED，S3关闭。
