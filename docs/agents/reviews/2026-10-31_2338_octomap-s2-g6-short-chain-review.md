# 2026-10-31 -- S2短链复核：保留接线证据，修复等待句柄与身份判定

- role: reviews
- agent: codex
- model: gpt-6
- cli: codex
- status: done

## Summary

复核3c8b1e9b，实现工作树干净。终态状态解释修复成立，已保存单条topic→终态→等待→另取句柄→分页/ROI→release短链以及两帧profile证据可保留。不是正式产品验收。S3保持关闭，placement桥接未实现，资源NOT_EVALUATED。src/include/CMakeLists相对7537b033无差异，原O9保留，本次不重跑。

独立加载ROS及/tmp/s2-install环境后，离线18 passed/1 deselected；owner的19项包含本轮未复跑的live冒烟，不混为独立复跑19项。本轮不启动ROS/正式窗口。

## Findings

1. WaitSnapshot.srv定义handle，但observer_node.cpp send_wait在OK时仅填写identity，不登记/返回快照租约。这是已观测且源码确认的实现缺陷，应记fail而不是not_evaluated。后续GetSnapshot是第二次获取，不是WaitSnapshot租约；单输入静态场景读到相同身份不能替代并发/reset语义验证。
2. _read_snapshot_chain先比较wait与snapshot身份并置identity_stable=false，最后chain.update又仅根据Surface/ROI彼此相同覆盖这个false。_identity_tuple还省略map_revision、last_integrated_stamp_ns和clock_domain。
3. 独立离线audit调用实际_read_snapshot_chain，健康对照与三项错误均identity_stable=true：等待epoch1却读epoch2；等待observed7却读observed6（未包含目标）；map_revision不同而其余被选字段相同。反例是注入服务响应，不声称真实服务曾返回这些错误。它证明短链判定没有拒绝能力，不能作为“正确快照”验收门。
4. 当前证据保存分页数量/状态和ROI数量/选取身份，无Surface/ROI内容hash或oracle内容比对；只可称接口接线及部分身份核对，不能宣称内容正确性。age_within_2s仅检查wait年龄上界，尚未综合校验GetSnapshot年龄、非负/有限值、目标覆盖与各页状态。
5. 正式stage的live分支仍仅返回LiveCampaign字符串和official_long_window_not_started，未构造/调用完整runner。累计profile短函数已存在，正式_run_profile仍走profile_frame。此前完整runner交付阻塞未解除。

## Next todo

1. 产品修复WaitSnapshot：成功返回有所有权的非空句柄，identity/age与该句柄实际快照绑定，证明目标终态已被observed_revision包含。获取/复核须处理reset、TTL、SNAPSHOT_PRESSURE、等待截止与失败释放，保留deferred响应、8等待限额和不阻塞health的合同。不可仅填写一个未登记的字符串，也不可重新引入持锁等待导致超时不响应。
2. 短链直接消费WaitSnapshot句柄，取消用额外GetSnapshot掩盖缺句柄。使用完整identity，校验期望epoch、目标覆盖、年龄、每页状态/游标前进/数组长度与全量覆盖、ROI内容/独立oracle、finally释放。若另测“较新快照允许”路径，要明确同epoch且包含目标的规则，不能把任意身份相同或不同都默许通过。
3. 将本次三个反例、空handle、STALE、目标未完成、并发reset/新积分、分页身份错误纳入同一校验路径；健康对照通过、各错误拒绝。证据需保存可复算内容或hash和重建输入，而非仅手写汇总。
4. 保留已有终态修复与短样本拒绝结果；完成真实正式适配器接线后再讨论S2整体关闭/S3开放。产品WaitSnapshot改动会结束“产品未变”的前提：保留7537b033历史O9，但不得自动给新候选代签，按受影响产品/采集范围重新确认资格。

## Pointers

- /tmp/elfin-occ-octomap-repair-2cm-g1/docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_g6/short_chain.json
- /tmp/elfin-occ-octomap-repair-2cm-g1/src/uld_occ_observer/src/observer_node.cpp (send_wait)
- /tmp/elfin-occ-octomap-repair-2cm-g1/src/uld_occ_observer/test/benchmark/campaign_executor.py (_read_snapshot_chain)
- docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_g6_review/audit.py
- docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_g6_review/RESULTS.json
- docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_g6_review/pytest.log
