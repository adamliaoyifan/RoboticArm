# 2026-11-01 -- S2 gen6 复核：S3 保持关闭

- role: reviews
- agent: codex
- model: gpt-6
- cli: codex
- status: done

## Summary

复核候选4fcd35c6da0901e416a1ca4e4cff5e3238459116（g2工作树clean）。不接受S2 gen6全部验收通过，S3不开放。owner Result原文保留；下一代S2交原owner修复。O9已有30/30 trace完整、冻结两二进制SHA匹配；本轮未重测性能、未启动ROS长窗。O8 owner预演PASS只能表示现有判官结果，尚不能认证原合同。

## Verification

独立audit.py实际调用候选adapter/判官：

1. O8注入全部30条回执缺失，stability_run跳过缺回执，输出概率键/块均0；在健康hold对照下，三轮输出经judge_o8仍PASS。单个70秒health样本也足以给node_count_stable=true。此为离线注入，不是说实机丢了30帧。
2. O5/O6给缓存样本加索引标记，正式index=3..32实际选中0..29：3个预热帧混入、最后3个评分帧遗漏。有限预演跳过预热，未覆盖此错误。
3. 构造文件名齐全但manifest引用不存在missing.yaml且SHA为fake的包，_save_failure仍capture_complete=true、replay_possible=true。此为证据声明误判；没有宣称真实产品重放成功。

现有测试显式固定OCC_S2_PROFILE_CLI/EVAL_NATIVE/NODE到s2g6-install后54 passed、6 live deselected。第一次仅source安装环境时，两个旧测试默认调用/tmp/s2-install CLI而失败（stack smashing）；初次日志及输入保留，不归因于本候选产品。COMMANDS.sh未设置这些覆盖变量，不能据其直接复现新候选测试。

## Source-established blockers

- live_schedule.py profile_frame减3造成上述索引错误；累计进程峰值RSS被单帧冷启动RSS覆盖，资源取最后document当前值而非整个运行峰值。需分别保留累计/冷启动值并按合同取实际峰值。
- ros_send每新window创建_WindowRun，其_open_session启动新observer；恢复不是在过载/排空后的同节点/同地图继续。正常窗也改走WaitSnapshot，不能替代原立即acquire查询延迟。ros_drain在固定截止点后最多再等30s，扩大了合同排空窗口。
- O8 30帧被铺在180s中，stability_run只看末5条且没有等最后帧完成；未实现原计划“另30帧饱和检查”。RSS按数组等分而非带时间戳的60s窗口；遮挡用重复ID重发，证明的是去重而非遮挡输入保持。保留重复ID测试，但不能代替遮挡语义。
- SnapshotRepository::usage中shared已含旧视图独占块，read_view再加old_bytes重复计数；read_view已含surface，profile又单列cache_bytes，判官read_view+cache再次计缓存。目录数来自store/snapshot块目录却乘handle Entry估算式，队列/完成节点用64等常数；没有独立分配清单证明实际容量/全覆盖。采样时更新peak也不是分配生命周期峰值证明。不能仅把certification默认设READ_VIEW_CACHE_ATTRIBUTED即宣称资格完成。
- 正式stage异常路径finally adapter.close()，未调用_save_failure；后者的文件清单检查也没有验证输入存在/SHA/身份/TF。wire路径的测试不能为正式runner代签。
- COMMANDS.sh o9使用未定义EVAL（set -u），rehearsal把profile CLI作为emit generator；FREEZE未锁定observer_eval_native及完整正式生成输入manifest。已保存O9数值保留，不等于该脚本可复现。

## S2 gen7 acceptance supplement

继承21d3145c全部1–8、原矩阵、阈值、非计分2h/冻结/O9、证据预算及S3关闭。基于4fcd35c6修复以上问题，具体要求：

1. 正式路径3预热不计分，30评分索引恰为3..32；标记反例穿过LiveScheduleAdapter→artifact→judge。累计峰值与冷启动分别实测，资源用覆盖完整运行的峰值；缺字段拒绝。
2. O7过载→固定30s排空→恢复复用同一observer实例及地图，保存PID/session/epoch/observed revision和截止边界；不得截止后补等后宣称闭合。正常查询沿原立即接口，恢复仅首查WaitSnapshot，其余29立即。离线生命周期探针+非计分短连续实机验证；不得跑正式600/60评分。
3. O8保留真实180s×3/首60s预热；另30帧检查均有匹配成功终态后逐帧验证0变化，不仅末5条。全部/一条回执缺失、拒绝、缺计数、非有限值、仅一个health样本、不足时间段RSS均须失败。RSS按真实时间窗，保存样本时间及各窗样本数；遮挡用明确几何阻挡/不穿目标区域的有效输入，保存射线与旧区域oracle；重复ID单列，不能代签。未接纳新数据时新鲜度不得前移，未观测旧区域概率保持。健康对照通过。
4. 资源修正旧块/cache重复计数，按各实际容器类型/分配容量及生命周期追踪；用独立所有权/分配清单核对多句柄、两版本、旧读者、TTL/release/reset、最后释放及采样间瞬态峰值。覆盖不足保持NOT_EVALUATED，S2不得全通过。范围外项明确保留，不能扩大通过范围。
5. 实际stage注入缺程序、发布后异常、查询失败、截断，各自拆卸前保存包。验证每个输入存在/真实SHA/身份/TF/完整前缀；本次伪包必须两项false。新进程实际重放与原失败比对；只有JSON验证不得算重放。
6. COMMANDS.sh显式固定所有二进制与测试环境变量，EVAL为实际observer_eval_native。冻结HEAD/clean、工具/生成器/共享运行库/fixture/最终输入manifest的SHA及精确命令。先非计分检查所有入口及离线全矩阵，冻结后重测受影响O9；不得覆盖当前历史证据。报告工程交付与产品验收分开。

## Pointers

- docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_gen6_review/audit.py
- docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_gen6_review/RESULTS.json
- docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_gen6_review/pytest_pinned.log
- /tmp/elfin-occ-octomap-repair-2cm-g2/docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_gen6/FINAL_REPORT.md
- docs/plans/OCC_OCTOMAP_CLAUDE_HANDOFF_20261101.md
