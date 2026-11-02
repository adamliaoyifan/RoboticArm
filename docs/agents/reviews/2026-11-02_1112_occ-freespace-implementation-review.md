# 2026-11-02 -- 统一 free-space 实现资格复核

- role: reviews
- agent: codex
- model: gpt-6
- cli: codex
- status: done

## Summary

候选6576650297518c35861349a702ce3869d10ff743尚不满足8caa0d44实现关闭条件，S3 gen3保持dispatch_ready=no。不新增指标，不重开S2，不运行正式样本。实现工作树HEAD eb6f5a1b只比候选多证据文件，产品/工具无差异且clean；6个冻结二进制SHA全部匹配。F/C及原三操作O9的owner结果保留，本轮未独立重跑F360或实机计时，不据此否定核心算法。

## Acceptance

以下是现有合同缺陷的一次性补齐清单；须走实际adapter→artifact→judge，并同时提供健康对照。不是追加性能目标。

1. **P/H/R/L判官完整性**。独立audit.py实测：P的300发送/240 INTERNAL_ERROR查询仍reasons=[]；R的600空发送行、仅一个无关COMPLETED、无恢复输入、NaN首Wait仍reasons=[]；L零秒/零流量/零循环/NaN资源峰仍reasons=[]。实现逐ID/恰一终态、完整成功查询、输入350/500ms、查询250/500ms、调度20/50ms（现P放宽十倍且无max）、年龄/health/截止闭合、过载真实压力/30秒边界/同图恢复、L120分钟及110分钟评分/原始RSS桶/所有权循环全部合同门。缺失、布尔、非有限、错身份及超门均不能通过；不能只用样本数挡住坏例。
2. **完整查询与租约**。full_query仅累加cell_x长度、不核对候选内容/全数组长度/完整页/游标进展/跨页与租约身份/释放状态。独立假服务返回1个候选却报100、错误身份、release INTERNAL_ERROR，调用仍OK；注入查询TimeoutError时release未调用。改为finally释放、全候选物化及canonical内容证据、全身份/计数/分页校验、状态入判官，并测试超时/坏页/释放失败。保留完整候选，不能仅存最后页身份和总数。
3. **同一调度器接线及TIMEOUT定位**。stage_r先send_inputs再run_queries，恢复亦先发完再Wait，不是并发首查；排空后collect_terminals可再等，不是30秒截止判定。stage_l相对sleep把查询耗时加进周期、丢查询返回内容且无逐帧终态/RSS时间序列。统一实际绝对时钟P/R/L，4线程executor（当前默认未指定）；输入stamp固定每帧+50ms与2.5Hz实际+400ms不符，时间越跑越旧，需用真实采集/计划时间及匹配TF。短R TIMEOUT不能先归因产品吞吐，保存发布/回调/完成/Wait/快照时间线先定位，验证正确并发和新鲜度。终态收集只认COMPLETED/REJECTED等合法终态，QUEUED不能提前移出pending。
4. **可运行评分、冻结与失败重放**。COMMANDS score即使开门仍not implemented并exit3，不能冻结后让S3再开发runner；在默认关闭状态实现完整正式路径和有界预演同源入口。judge实际重算文件而非只信summary.wiring_ok或O9 performance_certified。replay当前用|| echo把任何失败转成功，必须保留非零，并以真实失败输入再次执行产品。freeze/启动校验覆盖工具/静态库/typesupport/消费者/输入manifest/配置/命令；当前仅6程序+2fixture，require_fixtures只检查descriptor自报字符串，不能代表文件SHA。错误SHA/dirty/缺依赖/缺输入行为测试不得启动评分。
5. **完整链O9边界**。profile_cli.cpp:130的chain_start在repo acquire/get/ROI之后，和注释“acquire→全页→release”不符。现freespace_full=1.0021只适用于实际测到的后半段，不能给完整链资格代签。按实际正式完整链常开采集设置边界，从acquire前至全候选物化和释放，修复后冻结再测四操作30对，并由判官独立复核trace身份、全样本及失败重放。

## Verification

- python3 docs/status/evidence/occ_freespace_final_20261102/implementation_review/audit.py：三个正式判官坏例均误过、坏完整页误报OK、超时未释放，已保存RESULTS.json。这些是离线注入，不是实机故障记录。
- binary_hashes.json：6/6匹配。工作树候选后仅新增证据提交。
- 源码机制定位：benchmark/freespace_judge.py:206/279/319；freespace_campaign.py:106/173/269/364/420；profile_cli.cpp:130；COMMANDS.sh:177/219。
- 未启动正式长窗，未修改冻结候选，未宣称当前达到2.5Hz/2Hz或无泄漏。

## Pointers

- docs/plans/OCC_SURFACE_FREESPACE_FINAL_DELIVERY_20261101.md
- docs/status/evidence/occ_freespace_final_20261102/implementation_review/audit.py
- docs/status/evidence/occ_freespace_final_20261102/implementation_review/RESULTS.json
- docs/status/evidence/occ_freespace_final_20261102/implementation/FINAL_REPORT.md
