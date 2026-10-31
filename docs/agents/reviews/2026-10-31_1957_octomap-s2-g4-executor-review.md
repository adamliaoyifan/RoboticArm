# 2026-10-31 -- S2 gen4复核：保留O9证据，修复执行器伪造健康结果

- role: reviews
- agent: codex
- model: gpt-6
- cli: codex
- status: done

## Summary

S3继续关闭。候选7537b033daf20561c247bb7e09b6ee8805358a93，证据HEAD 5d914bf71dcd821a0417281ab66bdd425ef89409，独立工作树干净。
离线套件复跑9 passed、1 live smoke deselected，本次未跑ROS或新计时。核对持久化30条ON trace，按预期观察ID/操作/run全部完整；P95比复算ROI 1.0156359705334954、融合0.9809603064420823、首次Surface 1.0257049963063405一致。认可并保留这一候选的O9证据，不要求无条件重测。
但执行器仍未实际交付正式路径，而且组装层把错误输入转换成健康结果。新反例两例均误通过，S2全部pass不能认可。

## Findings

### 已实测的数据丢失

调用同一_run_ros使用的_assemble_window，将240条适配器输入全部设为rejected/QUEUE_FULL，terminal_count=1，最终normal判官pass。
再将240条设为unknown、receipt_received=false、integrate_ms=100000，最终仍pass。
两个artifact均带120条适配器根本未提供的健康查询。健康对照也pass，判别失效。代码从healthy_o7_*测试工厂开始，仅覆盖terminal_count等少数字段，输入拒绝、状态、耗时、回执与查询观测被忽略。证据s2_g4_review保存输入生成器、首条输出及判定。

### 源码确认的未交付项

- main(mode=stage)在S3关闭时拒绝是正确的，但环境开放后仍只返回s3_open_but_runner_not_started_by_this_repair及退出2，没有执行器选择/调用。这不是“为安全暂不运行”，而是可运行入口缺失。此项通过读源码确认，本次未设置开放变量或执行正式样本。
- LiveProfileAdapter是单帧smoke：每次取f000、复用live-profile目录，surface_first/after_update/repeat都取同一个first值，init=None。不是33帧累计/首次构建/冷启动独立观测的完整采样器，不能冻结成正式runner。
- ROS只提供1次service模式smoke，其发送后立即join再取结果。没有完整topic九窗/10Hz过载/排空/恢复的发送与查询调度适配器。_run_ros也没有用窗口duration/hz控制时间。
- _run_stability只调用30饱和帧和hold次数，完成后直接healthy_o8()；没有采集180秒时间序列、60秒RSS中位、节点/概率变化与新鲜度事实。
- 正式和smoke都默认ManualClock(step=0)，真实时间不会推进8小时总预算。judge/replay CLI无论结果失败都退出0。
- 全局formal_pass=false避免了当前总报告冒充正式通过，但不能使伪造的阶段pass或缺失的正式实现符合交付条件。

## Acceptance

S2 gen5仅补执行器/观测转换/调度与验证，继承主计划c02c1cfb、gen4合同38dc898601fdf1bc994a966aa7287d193c05d746。Owner eng/cursor/grok-4.7，base=5d914bf71dcd821a0417281ab66bdd425ef89409。

1. 先修两个已复现反例：正常窗任意拒绝/未知/无回执均非pass。真实组装路径不得调用healthy_*或_resources_ok等测试工厂；这些只留测试fixture。缺查询/health/身份/资源/延迟必须保留缺失，不能生成默认成功。逐字段原样传递/明确单位转换，保存来源ID，完整健康实测fixture可通过。
2. 统一schema与单一适配器协议，profile/ROS/stability采集全部实际观测；组装层仅验证/转换，不制造测量。测试覆盖拒绝、丢回执、错误查询/STALE、超门、重复ID、无health、RSS增长、节点/概率变化、资源缺项，每个故障穿过同一schedule→adapter→artifact→judge路径被拒绝，不只直接测试判官。
3. 实现正式模式选择真实适配器并执行阶段；保留默认closed，未授权不会启动。在测试中注入进程/传输/时钟适配器调用实际CLI路由，开放分支应确实dispatch到指定runner，不允许仍无条件返回stub。此测试不得启动正式O5–O8。judge/replay/执行器失败返回非零，并保存具名结果。
4. 完整profile runner按冻结seed序列、group对应geometry/config执行3预热+30累计、30首次构建及要求的独立冷启动，区分首次/更新后/重复Surface，各记录不同真实调用；唯一run路径保留全部输入和产物，不覆盖f000/单一目录。正确读取并聚合实际RSS/初始化，不补数。
5. 实现ROS topic完整适配器，独立发送调度不等待积分回执，按2Hz/10Hz与120/60/30秒窗、30秒排空、query链、首WaitSnapshot及后续29查询采集；终态守恒、健康间隔、队列年龄来自观测。stability跑180秒×3，首60秒预热，采集后两段RSS与节点/概率/几何/新鲜度；hold实测10秒×3，非仅调用次数。用可控时钟离线证明完整时间轴、依赖停止及跨阶段总预算，真实模式用monotonic时钟并给子进程剩余超时。
6. 完整路由实现后做明确non_scored的短CLI/ROS接线，实际adapter/组装器与正式一致，缺样由正式判官拒绝；不得运行正式O5–O8。实际测试报告区分“离线协议fixture”“真实短接线”“未跑正式”，不声称短接线证明长窗通过。
7. 更新冻结COMMANDS及执行器/接口/依赖/输入hash，默认check安全，实际阶段调用完整runner。只改调度/组装且产品及O9采集不变时，保留7537b033测得的O9并列工具新SHA与兼容检查；若改变采集/产品再结束旧候选并重测受影响项，不默认重跑30对。
8. 新证据s2_g5，历史只读；失败持久化、精确命令、全SHA/dirty、逐层输入输出、真实累计预算和剩余项必交，资源仍NOT_EVALUATED。验收逐条列pass/fail/not_evaluated；未交付完整真实runner或仍伪造观测不得关闭pass。S3继续关闭，不用修复任务偷偷运行正式样本。

## Dispatch decision

保留S2 gen4 Claim/Result，标superseded；同谱系gen5使用本记录首次提交SHA为plan_revision。
任务继续由原owner端到端实现测试提交关闭。不新增1cm，不接planning/Gazebo，不迁移生产，不解锁公平比较。

## Pointers

- `docs/agents/discuss/2026-10-31_1811_octomap-2cm-repair-s2-g4.md`
- `docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_g4/FINAL_REPORT.md`
- `docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_g4_review/RESULTS.json`
- `docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_g4_review/O9_RECHECK.json`
- `docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_g4_review/COMMANDS.sh`
- `docs/plans/OCC_OCTOMAP_REPAIR_2CM_G1_20261031.md`
