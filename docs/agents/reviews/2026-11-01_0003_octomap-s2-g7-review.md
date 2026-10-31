# 2026-11-01 -- S2租约补修复核

- role: reviews
- agent: codex
- model: gpt-6
- cli: codex
- status: done

## Summary

复核be844542，实现工作树干净。认可WaitSnapshot登记并返回租约、短链直接读该句柄、完整身份比较和空句柄拒绝的修复。owner短链报告15页Surface、4096格ROI、release OK、年龄12366233ns及内容摘要；本轮不重复实机短链。加载安装库后独立19 passed/1 live deselected，owner的20项不能写成独立复跑20项。

S3继续关闭。历史7537b033 O9不为本产品改动代签；正式live runner、placement桥接、资源认证未完成。

## Measured offline findings

执行当前_read_snapshot_chain，服务响应为离线fixture，无ROS或时间测量：

- 4096格ROI健康对照identity_stable=true。
- 请求4096格但仅返回1格，仍identity_stable=true。当前代码保存count却不判断数量，也未检查ROI的log_odds长度/done/游标与内容oracle；有任一occupied即可通过内容存在性检查。
- 提供min_observed_revision=7、响应observed=6时正确拒绝；省略minimum时相同响应被接受。真实smoke ros_query的expected只有epoch/sensor_id/observation_id，未从匹配终态回执传入目标observed_revision，所以当前真实链没有独立验证目标覆盖。此前页身份不匹配反例已修，但不能替代这一检查。

上述反例只证明客户端判定缺口，不声称实机返回过缺样或旧快照。内容sha256是指纹，不能独自证明内容符合oracle；chain.json未保存生成该摘要的全量数组，本轮不能从该汇总独立复算摘要。

## Source-established concurrency risks (not reproduced live)

send_wait在OK路径调用repository.acquire→store.snapshot，后者取得store mutex。释放wait_mu避免了该互斥锁长持有，但没有消除store锁等待。poll通过后，新的integrate可持有store锁；acquire结束后没有重新检查started+timeout_ms，因此可能在截止时间后返回OK。max_age验证用的是调用acquire前取得的now时间，也需要在延迟路径检查实际响应年龄。不能仅以“没持有句柄表锁等store”宣称deferred等待仍满足期限。

即时ready分支解锁后send_wait，既未加入waiters也未增加waits_checking；若此时被store锁阻塞，这个在途等待不在8请求计数内。queued路径的checking计数仍在。需要覆盖即时ready→租约获取这个新阶段的全程限额，而非只测试排队状态。

本轮未测得上述超时/超额，只按明确调用链标风险；应先用既有持锁hook与屏障触发，再修复和验证，不跑正式样本。

## Next todo

1. 真实短链从完整身份匹配的COMPLETED/INTEGRATED回执传入目标observed_revision；同时固定期望geometry/config/clock，拒绝目标覆盖不足。保存可校验的输入及内容以复算摘要。
2. ROI要求4096完整格、状态/概率数组及分页合同正确；Surface核对全域列数、索引唯一/范围、游标前进和最终完成；用独立oracle核对内容，不能仅检查有occupied。补缺1格、重复/遗漏列、错值及健康对照，走同一校验路径。
3. 新WaitSnapshot租约路径补并发行为测试：目标已完成且另一个integrate持store锁，等待截止仍可响应；即时ready在途计数达到8后第9个压力拒绝；reset、TTL、句柄压力、异常和超时释放无泄漏。按实测结果修非阻塞租约获取/截止复核与计数，保留已完成修复。
4. 上述过门后继续完整runner交付，再冻结并确认新候选捕获资格。不能用历史O9或单帧短链代替产品验收。

## Pointers

- /tmp/elfin-occ-octomap-repair-2cm-g1/docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_g7/chain.json
- /tmp/elfin-occ-octomap-repair-2cm-g1/src/uld_occ_observer/src/observer_node.cpp (send_wait、WaitSnapshot callback)
- /tmp/elfin-occ-octomap-repair-2cm-g1/src/uld_occ_observer/src/observer.cpp (ObservationStore::snapshot)
- docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_g7_review/audit.py
- docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_g7_review/RESULTS.json
- docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_g7_review/pytest.log
