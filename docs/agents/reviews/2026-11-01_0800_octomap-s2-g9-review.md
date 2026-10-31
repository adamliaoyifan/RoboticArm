# 2026-11-01 -- OctoMap S2 g9 live接线复核

- role: reviews
- agent: codex
- model: gpt-6
- cli: codex
- status: done

## Summary

复核a70929be，owner工作树干净。确认LiveCampaign.wire已实际接入累计profile及ROS短链，不再仅返回标签。接受有限非计分接线进展，不接受完整正式runner或S2整体通过，S3保持关闭。独立22 passed/3 live deselected；owner25项含live，本轮未重跑ROS或正式样本。两条保存内容指纹独立复算一致。

## Findings

1. query_started只在两条链之前设置一次；query_s在两条链及sender.join后计算，包含等待发送、终态查询及完整租约读链。0.717092秒是这段总耗时，不是单个ROS服务往返，也不是单次查询实测。第二条queries.latency_ms同样累计。实际发送间隔0.500402秒保留为owner实测；需要逐请求起止和时序重叠证据才能进一步宣称慢查询下的独立发送保障。
2. 独立离线注入NaN/Inf到三种Surface时刻，_wire_profile仍executed=true，和健康对照相同；正式judge均false，并未误判正式验收通过。原因finite只检查数值类型。修正为有限非负校验，并检查每帧receipt，不能以缺组拒绝掩盖坏采样。证据s2_g9_review/audit.py与RESULTS.json。
3. wire是固定两帧与短采样；stage live仍调用wire，尚无适用全矩阵/长窗执行分支。--stage作用、O5失败停止O6/O7、O7过载/恢复、O8固定射线和hold，以及冷启动/首次构建样本都须落实于正式runner。当前无正式样本启动的陈述准确。
4. O7缺callback入口时刻属采集接口缺项，非已证明消息丢失。publish_end_ns当前写的是云stamp，不能充当publish返回实测时刻；terminal_count按是否收到回执合成0/1，无法发现重复回执。正式入口应保存原始同ID回执事件及明确的时间域/边界，不用布尔值制造计数。发布QoS目前depth10也须按冻结合同明确对齐。
5. O8 executed目前只依赖判官拒绝和not_run标签，不要求有效且不同的health样本/RSS；需要健康与缺采样反例。_save_failure无条件capture_complete=true，details仅截断异常字符串；作为启动失败提示可用，不能宣布任意运行时失败捕获完整，须按实际文件/输入/事件清单计算并在teardown前持久化。

## Next

先修短接线自己的采样判定和逐边界计时，再完成正式runner及资源归属；非计分预演后冻结并重测O9，随后审核S3开放。7537b033仍只作历史。placement转换另列，不作为本轮已交付内容；不启动规划或正式评测。本轮未改owner代码或任务历史Result。

## Pointers

- `/tmp/elfin-occ-octomap-repair-2cm-g1/docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_g9/FINAL_REPORT.md`
- `docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_g9_review/audit.py`
- `docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_g9_review/RESULTS.json`
- `docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_g9_review/pytest.log`
- `docs/agents/discuss/2026-10-31_1956_octomap-2cm-repair-s2-g5.md`
