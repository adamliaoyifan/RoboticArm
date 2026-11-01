# 2026-11-01 -- S2 gen7：接受已修复项，资格仍未完成

- role: reviews
- agent: codex
- model: gpt-6
- cli: codex
- status: done

## Summary

5643039bca744de2abb5ac5837921170bfe93c0a工作树clean；冻结5个二进制/库SHA全部一致。O9已有比值1.021460/0.990544/1.012460，独立重判30/30 trace完整，本轮没有重测性能。固定s2g7安装后独立75 passed、9 deselected（排除live及持锁实机测试）；不代签owner实机结果。采样3..32、伪包拒绝、真实遮挡输入及过载/恢复同进程已有实质修复。S2全部通过仍不成立，S3继续关闭；不将已知正常窗延迟超门风险本身作为资格阻塞。

## Reproduced findings

本轮audit.py以保存的真实stage_rehearsal/o8/runs.json为健康对照，健康PASS。下面每种独立离线注入仍PASS；不声称实机发生这些错误：

- 将每轮saturation_frames_expected/verified改0并清空saturation_receipts，判官仍PASS。预期数取自被评artifact而非固定合同30。
- 30条观察ID全改相同错误ID、disposition=unknown，仍PASS；events改2（重复回执）亦PASS。只检查status=COMPLETED及events>=1，不校验唯一身份/匹配的INTEGRATED终态。
- health时间改成30个60.0秒，仍PASS；没有真实覆盖60–180秒或去重复判定。
- 调用真实_hold_probe→hold，注入status=INTERNAL_ERROR且空数组的Surface响应；探针返回空列哈希，两个空哈希相等，probability_unchanged=true。替换实际artifact中no_input holds后judge仍PASS。注入duration_s=0亦未被拒绝。

另由源码直接确认：_hold_probe仅hash (x,y,active_count,known_count,has_occupied,top_z)，没有任何log_odds。6534列几何hash不变不能证明旧区域概率保持；不跨状态阈值的概率变化不会改这些字段。探针也未验证全页完整/分页身份/释放状态。这是既有“旧概率保持”合同缺口，不是新增性能门。

## Resource qualification

重复计数修复有效，但资源门仍需NOT_EVALUATED：

- owner明确块/目录/cache/read_view峰值只是usage调用的采样高水位，未覆盖采样之间创建后释放的对象；这不满足上一代已要求的生命周期峰值与瞬态测试。
- ObservationLedger::accounted_bytes_unlocked仍是sizeof(Record)+若干string.capacity，遗漏map节点及key string对象；SnapshotWaitBoard同类。SSO容量混加、堆字符串终止字节未区分，不能称node-inclusive实际分配。
- ingress deque使用消息数×(sizeof(Item)+两指针)，没有测块分配/空队列保留容量。shared对象控制块、读视图对象等需纳入明确所有权清单或按门相关未覆盖处理，不能由resource_scope_complete默认true完成认证。
- 测试用相同估算公式交叉求和只能证明加法与去重，不是独立分配计量。修复不要求统计进程全部内存；tf/middleware/fingerprint范围外仍明确保留。

## S2 gen8 bounded acceptance

继承57e3a0a9、21d3145c及原计划全部门限/预算；只收口下面三个失败边界，不重写历史成绩：

1. O8验证器固定每轮30帧；验证实际发送身份与原始终态匹配、唯一ID、epoch/sensor、COMPLETED/INTEGRATED及disposition、零重复终态；0/29/31帧、bool expected、错误ID、重复事件、unknown均拒绝。health/RSS保存原始值/单调时间并复算，覆盖真实规定窗口，重复时间/缺覆盖/缺样不通过。hold要求实测10秒、同租约完整分页、身份、成功释放；错误/空/少页/重复页/NaN/短时hold不得通过。用本次保存artifact逐项扰动与健康对照验证，不能只添加字段存在测试。
2. 概率保持oracle按明确旧区域读取完整已观测体素state+log_odds（或可独立复算的完整概率摘要），包含未观测状态，保存输入和内容。测试在Surface几何完全不变时改变至少一个旧格log_odds，必须拒绝；no_input和真实遮挡均复核概率，空oracle拒绝。保留重复ID单列和已修复连续性。至少一个非计分真实hold接线及三种hold实测完整性通过；O8资格按原规定窗口复测，不启动正式评分。
3. 资源按实际分配/释放生命周期统计并去重；对map节点/key/string、deque保留容量、视图/控制块/cache对象及buffer等门相关所有权逐类核对。采样间创建/释放缓存和旧视图必须抬峰；最后所有者释放应回基线。独立计量对照不能复用被测估算公式。无法覆盖的类保持NOT_EVALUATED，并以blocked/未完成如实收口，不能pass_with_scope绕过同一门。更新COMMANDS、冻结输入工具产品SHA，受影响测试及冻结后O9重测；历史gen7结果不覆盖。

Owner仍eng/claude/glm-5.3，基线5643039b。S3 dispatch_ready=no；修复/验证/证据/提交/关闭同owner完成。预演≤2h不含等锁，正式O5–O8不开跑。无需再次为已授权的补测申请用户批准。

## Pointers

- docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_gen7_review/audit.py
- docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_gen7_review/RESULTS.json
- docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_gen7_review/pytest.log
- docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_gen7_review/o9_rejudge.json
- docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_gen7/FINAL_REPORT.md
- docs/agents/reviews/2026-11-01_1205_octomap-s2-gen6-review.md
