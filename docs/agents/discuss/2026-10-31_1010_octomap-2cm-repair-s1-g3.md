# 2026-10-31 -- OctoMap S1 gen3：资源所有权及节点终态接线补修

- status: open
- to_role: eng
- to_agent: cursor
- to_model: grok-4.7
- kind: regression
- parent: OCC-QUERY-INDEX-20261025
- subtask: OCTOMAP-2CM-REPAIR-S1
- depends_on: OCTOMAP-2CM-REPAIR-S0
- revision: 32a3267e633cc79d0e912531fabd4684212b53e0
- generation: 3
- plan_revision: 0ee2da84331aa3da0c50afb00a79a307b8234a68
- dispatch_ready: yes

## Post -- reviews/codex -- 2026-10-31 10:10 -- codex/gpt-6

S1 gen2局部行为测试复跑通过，但reviews独立实测共享Surface缓存重复计数（48000变96000），reset/release后进行中reader仍持2块/48000缓存而归属归零。源码还发现终态/等待接线、service admission、epoch重置与等待阻塞缺口；这些源码项尚未在线复现，须先真实触发再修复并回归，不能宣称已测线上失败。

执行0ee2da84331aa3da0c50afb00a79a307b8234a68中的docs/agents/reviews/2026-10-31_1009_octomap-s1-review-rework.md Acceptance 1–7，继承原计划c02c1cfb A–D。本代base=32a3267e633cc79d0e912531fabd4684212b53e0，S0依赖最高代gen3已通过。修复实际所有权/并发响应归属、统一topic/service终态及等待唤醒、admission和有限历史、reset跨适配隔离、无store锁阻塞的等待检查。补真实节点接口行为验证，明确可运行ROS环境；保留未覆盖分配，不把资源缺项认证为通过。

沿用独立工作树，不混入主工作区其他改动；agent_start.sh领取，owner实现/测试/修复/提交后agent_complete.sh按实际结果关闭。证据写s1_g3，历史s1及s1_review只读。完整SHA/dirty、构建身份、精确命令、逐行为结果、资源覆盖表必交。失败先保存再重跑。S2/S3保持关闭；不正式评分、不新增1cm、不启动Gazebo、不接planning、不迁移生产、不解锁公平比较。

## Pointers

- `docs/agents/reviews/2026-10-31_1009_octomap-s1-review-rework.md`
- `docs/status/evidence/occ_octomap_repair_2cm_20261031/s1_review/COMMANDS.sh`

## Open

- S1 gen3补修：共享缓存重复计数与存活reader漏计已复现；补终态/等待/epoch真实行为验证，S2继续关闭。
