# 2026-10-31 -- S2复核：比值算术成立，真实捕获资格与评测冻结未完成

- role: reviews
- agent: codex
- model: gpt-6
- cli: codex
- status: done

## Summary

不开放S3。认可等待在途修复的已交付证据及三组30对O9比值算术：0.9963607021704047 / 0.9887859269007071 / 1.007934487148608。
独立复算官方判官确实pass，但该判官同时输出capture_certified=false；它只检查所给数值与标志，不能证明真实故障重放。
产品16dda82286b345b0a6f4208ac9c3f43621b8d8fd，证据HEAD 2d42aa6a2ea57248e992b2231d7738532f3c76c1，工作树干净。本次只离线审查，未启动ROS或性能测量。
S2 gen2的全部通过结论不成立，派发gen3补修；不覆盖历史测量或原Claim/Result。

## Findings

1. 独立提取并执行s2_rehearsal.py原函数体（AST，无改动）：伪造从未执行的失败JSON，只带ID/epoch/status/reason/sensor/capture_complete，replay_bundle返回true；没有输入、TF、可重建地图或真实重执行。replay_trace对仅3条无身份事件且footer报告999条drop也返回true。两个反例/输出/程序已持久化s2_review。
2. s2_o9.py把epoch99故障写成字典，读回相等即算replay_real；没有把epoch99输入送入产品。incomplete_replay则是硬编码false。O9循环没有验证每个ON trace的完整性，content为None/None也会当作匹配。故当前性能比值不能认证原计划的捕获/故障重放资格。
3. COMMANDS.sh声称冻结，但官方point-cloud bytes明确not_emitted；O5/O6都调用旧fusion_benchmark baseline，O7调用cold，O8调用memory。旧benchmark自己输出product_acceptance=not_evaluated；cold是逐CLI冷启动，不是ROS正常九窗/过载/恢复。judge只是pytest，replay只是O9字典重算。这些不是计划要求的可执行正式验收链。
4. 入口ROOT由证据位置推导主工作区，不核实产品SHA；仅在/tmp存在install/样本并非冻结。O9采用旧5cm cuboid配置和独立随机30k生成器；本结果不外推为2cm目标与原seed/outside/cross输入的捕获资格。应明确认证操作、配置和输入范围。
5. 资源缺项已经如实NOT_EVALUATED，保留。它是额外阻塞，不是本次两个误重放或命令错接的原因。

## Acceptance

S2 gen3继承c02c1cfbd9ba16a1c4a6c6169695f1daf8eb6a76及S2已授权预演修复范围，owner eng/cursor/grok-4.7。
base=2d42aa6a2ea57248e992b2231d7738532f3c76c1（包含产品16dda822与证据）；候选产品另列准确SHA，不把文档提交当性能实现变化。

1. 先离线修复捕获/重放判定与脚手架，不立即重跑30对。上述两个反例均明确非pass，健康真实重放正例通过。缺/重复/错配ID、非连续或无效事件、drop>0、缺页脚、缺输入/TF/地图前缀、SHA错误均不得认证；计数/状态不得默默补齐。
2. 为实际失败保存无损输入或可校验生成manifest、同戳TF/缺TF条件、实际配置和命令、epoch/完整ID、必要地图前缀、输出、边界事件及hash。重放必须重新调用相应产品边界/节点，在干净状态重建，再比较身份、状态/原因、内容；JSON读回相等只能叫文件校验。真实触发epoch拒绝、TF拒绝和原计划适用故障，区分触发/保存/校验/执行结果，不能手写预期结果冒充观测。
3. ON每个样本trace验证完整且无drop，OFF/ON输入/config/二进制身份一致，内容hash必须存在且匹配；首次Surface真实miss，全部30样本纳入分母。O9总资格为延迟门AND捕获完整AND故障可重放，不以旧S0数值判官pass单独认证。
4. 实现实际O5四组、O6 96组及30冷启动、O7九个正常窗/600过载/60恢复、O8三轮稳定与hold/saturation的可运行入口，精确绑定原几何/seed/outside/cross及2/5cm范围。判官读取实际产物逐组逐窗判定，replay重执行具体失败；不能用baseline/cold/memory别名或pytest代替。仅离线生成全部最终输入或无损可校验生成manifest并冻结hash，不执行O5–O8正式样本。
5. COMMANDS.sh从冻结候选干净工作树/明确安装路径运行，校验完整产品/工具SHA、dirty、构建/安装与输入/config hash；明确阶段选择，S2默认不会调用S3。离线测试证明每阶段解析到正确矩阵/时长/速率/预算/依赖/输出，O5非pass停止O6/O7，缺样/未知/资源缺项不通过；不得靠grep脚本文字。
6. 预演修复结束后重新冻结再测受影响O9，不拼多个候选。以2cm目标、固定4096-cell ROI、原生成器30k及首次Surface验证当前捕获配置；对旧5cm结果保留独立标签，不代签新目标。每操作30对AB/BA，P95比≤1.10，实际env/RMW/DDS/逐线程CPU/上下文切换/频率/负载/RSS与锁记录齐全；不以逻辑核数代替利用核数。保持客户端4线程、独立发送、评分禁用行为hook。
7. 遵守原2小时预演和8小时正式测量预算（等锁不计），本代单列耗时并连同历史消耗汇总，耗尽not_evaluated；失败先保存，不抢跑S3。新证据s2_g3，保留s2/s2_review。T0/T1、T2±5秒或冻结扩大窗口、完整前缀、512MiB/例、2GiB/run、8GiB总预算，SHA/缺失项/capture_complete/replay_possible按真实结果填写。
8. 资源覆盖不足保持NOT_EVALUATED并列S3阻塞，不能靠RSS或部分字节小于512MiB代替。按实际完成结果agent_complete，未满足捕获资格/冻结要求不能标pass。

## Dispatch decision

保留S2 gen2原Claim/Result并标superseded，同谱系gen3的plan_revision为本记录首次提交SHA。
S3继续dispatch_ready=no。不新增1cm、不接planning/Gazebo、不迁移生产、不解锁公平比较。

## Pointers

- `docs/plans/OCC_OCTOMAP_REPAIR_2CM_G1_20261031.md`
- `docs/agents/discuss/2026-10-31_1611_octomap-2cm-repair-s2-g2.md`
- `docs/status/evidence/occ_octomap_repair_2cm_20261031/s2/FINAL_REPORT.md`
- `docs/status/evidence/occ_octomap_repair_2cm_20261031/s2/COMMANDS.sh`
- `docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_review/RESULTS.json`
- `docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_review/COMMANDS.sh`
