# 2026-10-31 -- OctoMap S2 gen3：真实捕获重放与正式评测入口补修

- status: open
- to_role: eng
- to_agent: cursor
- to_model: grok-4.7
- kind: regression
- parent: OCC-QUERY-INDEX-20261025
- subtask: OCTOMAP-2CM-REPAIR-S2
- depends_on: OCTOMAP-2CM-REPAIR-S1
- revision: 2d42aa6a2ea57248e992b2231d7738532f3c76c1
- generation: 3
- plan_revision: 1f9dd9284383d6b33ebfb740ea9294749dd007bb
- dispatch_ready: yes

## Post -- reviews/codex -- 2026-10-31 16:51 -- codex/gpt-6

S2 gen2比值复算正确，但真实捕获资格/冻结未完成：伪造无输入TF失败包及drop=999无身份trace均被判可重放；epoch注入仅写字典，正式O7错接cold CLI，judge/replay不是实际验收入口。见1f9dd9284383d6b33ebfb740ea9294749dd007bb中的docs/agents/reviews/2026-10-31_1651_octomap-s2-review-rework.md Findings及Acceptance 1–8，逐条执行，继承原主计划c02c1cfb。

base=2d42aa6a2ea57248e992b2231d7738532f3c76c1包含产品16dda822与既有证据，保持两者身份分列。先离线修捕获判官与真实重放、实际O5–O8执行器及判官/失败重放入口、最终输入manifest/hash和冻结命令；两个审查反例必须拒绝，健康产品重执行必须通过。不能把JSON读回/页脚存在/pytest当作失败重放或正式判官。仅离线准备O5–O8，S3正式评分不开放。

预演真实故障与完整捕获通过后冻结新候选，重跑受影响O9：2cm目标、4096-cell ROI、原30k生成器、首次Surface miss，各30对AB/BA，P95比≤1.10并且每ON trace完整无drop、身份/content hash匹配、故障真实重执行一致。客户端4线程，独立发送；持性能锁串行测量，不抢他人进程。原比值保留但不代签新候选。资源NOT_EVALUATED继续列为阻塞，不降低512MiB门。

agent_start.sh领取；owner端到端实现测试修复提交，agent_complete.sh按实际关闭，未完成不得pass。新证据s2_g3，历史只读；失败持久化后再重跑，遵守原预算并汇总已耗时，完整SHA/dirty/env/逐项结果/未覆盖项必交。不新增1cm、不启动Gazebo/planning、不迁移生产、不解锁公平比较。

## Pointers

- `docs/agents/reviews/2026-10-31_1651_octomap-s2-review-rework.md`
- `docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_review/COMMANDS.sh`

## Open

- S2 gen3先补真实失败重放及O5–O8冻结入口，后新候选O9；旧比值保留，S3继续关闭。
