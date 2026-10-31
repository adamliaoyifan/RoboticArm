# 2026-10-31 -- S2 gen3复核：真实重执行有进展，执行器与判官仍未交付完整

- role: reviews
- agent: codex
- model: gpt-6
- cli: codex
- status: done

## Summary

S3继续关闭。工具d653c9c84f2c47dc0daa0046ac57f26b30a9bb3b，证据967d01a63eb6862b838d370659d859958b1d868a，产品C++仍16dda82286b345b0a6f4208ac9c3f43621b8d8fd，独立工作树干净。
复跑test_s2_g3_offline.py 4项通过，其中epoch99测试确实运行profile CLI两次并比较EPOCH_MISMATCH；上轮伪造包/drop999反例均正确拒绝，认可这部分修复。TF两节点重执行按源码及owner证据记录，本次未重跑ROS。
但四个新反例均误通过，且正式入口仍只打印描述，不能认可S2全部pass。保留2cm O9测量与旧5cm结果，不宣布本轮O9数值失败；捕获完整性资格仍不充分。

## Findings

### 实际复现

- capture_replay.certify_trace：只含一个合法格式、drop=0的trace_footer，无任何操作事件，返回capture_complete=true。
- campaign_entry.judge_recorded_window：30个积分样本均为100000ms且resource_certification=pass，返回pass；同样NaN、True均pass。既没有门限检查，也绕开S0已修好的数值校验。输入/输出及脚本在s2_g3_review。

### 源码与交付事实

- campaign_entry.resolve_stage只返回字典；没有调用profile CLI或ROS执行器。COMMANDS.sh的o5/o6/o7/o8仅print(resolve_stage(...))；case只支持check/rehearsal/o9/s3拒绝，不存在实际评分执行入口。continue_after只返回布尔字典，没有与执行器的停止行为绑定。四组/96组/九窗等只是正确描述，不能等同于可运行交付。
- 新judge_recorded_window不检查查询、max/P95、ID守恒等合同；不得作为认证捷径替代S0逐组逐窗判官。
- certify_trace只检查字段/序号/页脚，未对冻结的预期ID集合、必需开始结束边界及顺序、run一致性、内容进行匹配。因此完整字段、零drop不代表捕获完整。
- replay_failure虽执行产品，但sensor_id、reason、tf及map_prefix重建未完整使用/比较；只比较status/observation_id/epoch/tree_nodes。不能把仅适用空地图epoch拒绝的实现当作任意失败重放。TF测试在同一Python进程复用内存message，持久化文件只有stamp/frame/data_sha；离开该内存后如何无损重建输入需要实际验证。
- 资源NOT_EVALUATED保留为独立阻塞，不是上述误通过原因。

## Acceptance

S2 gen4继承主计划c02c1cfbd9ba16a1c4a6c6169695f1daf8eb6a76及gen3补修合同1f9dd9284383d6b33ebfb740ea9294749dd007bb。Owner eng/cursor/grok-4.7。
base=967d01a63eb6862b838d370659d859958b1d868a。先完成离线实现/验证，不再先跑O9后补入口。

1. 四个反例必须非pass，健康完整数据仍pass。窗口判定复用/接入S0完整判官：每组P95/max、每窗身份/计数/状态、缺样、有限非负数值及资源覆盖按原门计算。增加健康、超门、NaN/Inf/bool、缺字段、单窗失败/少帧/重复终态、恢复timeout/错误快照、资源缺项的行为正负对照，不仅验证字典数值。
2. 真正实现profile_cli_groups、ros_windows、ros_stability执行器和统一调度入口，能生成输入、启动相应产品、按计划调度、采集逐输入/查询/health/资源结果、持久化并调用判官。阶段解析与执行分开但必须实际接通。仅打印字典、注释或未实现runner名称不是完成。
3. 在离线可注入的进程/时钟/传输适配器上调用同一调度路径，验证全部矩阵、次数、独立发送、九窗AND、排空、恢复首WaitSnapshot、O8三轮/hold/饱和、全局8小时预算、O5非pass时O6/O7启动次数=0。保存调用账和逐阶段结果。再执行明确标non_scored的最小真实CLI/ROS接线冒烟，证明执行器实际产生可解析结果；缩减样本应被正式判官拒绝为缺样，不能用冒烟代签正式门。S3仍关闭，不执行正式O5–O8样本。
4. trace认证输入必须包含预期操作/输入/请求身份集合与必需边界；验证一一匹配、合法顺序、同run、连续序号、非负有限时间、唯一末尾页脚和drop=0。仅页脚、缺起止、遗漏整个操作、ID替换、重复、串run都非pass。对每个O9 ON采样接入该检查，不由trace自己声明自己完整。
5. 重放用持久化输入/生成manifest及hash、TF状态、配置、原始回执与必要完整地图前缀重建；新进程只读失败包，不能复用原Python内存对象。比较完整sensor/observation/epoch身份及状态/原因、适用内容证据。sensor/reason/TF/prefix修改必须被拒绝或导致具名不匹配。暂仅支持的故障类型显式白名单，其他标not_evaluated，不假装通用可重放。
6. COMMANDS.sh提供真正可调用的阶段/全部入口和独立判官/失败重放命令，默认check安全、正式执行仍需S3开放标志。核实完整工具/产品/生成器/接口及所有执行器SHA、dirty与输入/config/install hash。不能遗漏新增执行器或依赖/tmp未冻结的旧脚本。提交可校验生成manifest合法，但必须证明能无损物化并符合对应实际输入hash，不能仅有阶段数量。
7. 上述离线门与非计分接线通过后再冻结。先用新捕获判官复核旧2cm ON trace及故障包：若仅工具判定变更且原始证据充分、产品/采集配置未变，可保留原数值并补独立复算；若捕获不完整或采集实现/配置变化，原候选资格不通过，提交新候选重跑受影响O9。不得拼跨版本证据；不默认无条件重测30对。原30对P95≤1.10、首次miss、4096 ROI、原30k及2cm约束不变。
8. 新证据s2_g4，失败先保存，历史只读；原预演/测量预算按实际累计报告，缺样/超预算/未实现明确not_evaluated或blocked。资源512MiB缺项仍NOT_EVALUATED。完整SHA/dirty、命令、机器可读结果、剩余阻塞及角色笔记交付后agent_complete按实际关闭；仍只有描述而无执行器不得pass。

## Dispatch decision

S2 gen3原Claim/Result不改写，标superseded，派发同谱系gen4；本记录首次提交SHA为plan_revision。
S3继续dispatch_ready=no。不新增1cm、不启动Gazebo、不接planning、不迁移生产、不解锁公平比较。

## Pointers

- `docs/agents/discuss/2026-10-31_1651_octomap-2cm-repair-s2-g3.md`
- `docs/plans/OCC_OCTOMAP_REPAIR_2CM_G1_20261031.md`
- `docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_g3/FINAL_REPORT.md`
- `docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_g3/COMMANDS.sh`
- `docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_g3_review/RESULTS.json`
- `docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_g3_review/COMMANDS.sh`
