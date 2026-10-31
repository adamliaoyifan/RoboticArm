# 2026-10-31 -- OctoMap S2 gen4：真实执行器与严格捕获判官

- status: open
- to_role: eng
- to_agent: cursor
- to_model: grok-4.7
- kind: regression
- parent: OCC-QUERY-INDEX-20261025
- subtask: OCTOMAP-2CM-REPAIR-S2
- depends_on: OCTOMAP-2CM-REPAIR-S1
- revision: 967d01a63eb6862b838d370659d859958b1d868a
- generation: 4
- plan_revision: 38dc898601fdf1bc994a966aa7287d193c05d746
- dispatch_ready: yes

## Post -- reviews/codex -- 2026-10-31 18:11 -- codex/gpt-6

执行38dc898601fdf1bc994a966aa7287d193c05d746中的docs/agents/reviews/2026-10-31_1811_octomap-s2-g3-review-rework.md Acceptance 1–8，继承原主计划和gen3合同。上轮两个反例已正确拒绝，epoch重执行认可；但新4反例误通过（footer-only完整、100000ms/NaN/bool窗口通过），且O5–O8函数仍仅print阶段描述。

先实现真正CLI/ROS执行器、调度及S0完整判官接入；在同一调度路径注入离线适配器证明矩阵/次数/全局预算/O5停止行为，再做non_scored最小真实接线，缺样不得算正式通过。trace对预期ID/操作/边界集合认证，失败包脱离原进程仍可重建并比较完整身份/状态/原因/内容。实际COMMANDS需阶段、判官、失败重放入口和完整SHA/hash校验，默认check不评分。详见8条关闭条件，不以描述或返回字典冒充执行。

全部前置通过后先用新判官复核已有2cm原始O9；仅判定工具变化且证据充分可保留原数值，捕获或候选变化才重测受影响O9。不能先重测再补脚手架，也不跨版本拼成绩。资源仍NOT_EVALUATED，S3关闭，不执行O5–O8正式样本。

base=967d01a63eb6862b838d370659d859958b1d868a，owner eng/cursor/grok-4.7，依赖S1 gen3已通过。agent_start.sh领取，owner实现测试修复提交后agent_complete按实际关闭；证据s2_g4，历史只读，失败先保存，累计预算/完整SHA/dirty/结果/未覆盖项必交。无Gazebo/planning/1cm/迁移/公平比较。

## Pointers

- `docs/agents/reviews/2026-10-31_1811_octomap-s2-g3-review-rework.md`
- `docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_g3_review/COMMANDS.sh`

## Open

- 先补实际O5–O8执行器/调度/判官，拒绝footer-only和超门/非法数值；前置通过后复核O9，S3关闭。
