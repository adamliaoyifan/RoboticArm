# 2026-10-31 -- S1 gen2复核：资源归属与节点接线补修，S2关闭

- role: reviews
- agent: codex
- model: gpt-6
- cli: codex
- status: done

## Summary

不开放S2。产品32a3267e633cc79d0e912531fabd4684212b53e0，基线f28c0cf598faf90b9bec28a3a0168f65ecc65d72，记录8fc83241，独立工作树干净。
新建/tmp/observer-s1-review从源码构建，原observer_repair_s1_behavior通过；独立资源probe退出1，三个实际所有权断言失败。
不否定已证明的持锁读取、TTL、队列等局部行为；但局部类测试不能替代节点接线和所有权合同。产品未评分，捕获未认证。

## Findings

### 直接测得的资源缺陷

1. 两个句柄指向同一个Snapshot对象；同一Surface缓存报告从48000增为96000字节，未按实际所有权去重。
2. reset并release两个句柄后，外部读取者仍持有2个块及48000字节缓存，read_cells仍返回8个cell；usage却报告块/缓存均0。遍历句柄表漏掉合法进行中读取。
3. 原测试只检查共享块不增与release后不大于原值；未检查共享缓存去重，也未检查最后所有者释放的准确生命周期。原“read started before reset”断言实际放在reset调用之前。

精确输入probe.cpp、构建命令、完整输出和SHA在s1_review；不是合成判官或线上性能测量。

### 源码确认的接线缺口（尚未在线复现，不声称实测失败率）

- observer_node.cpp on_cloud/adapt拒绝分支只写ledger/发布拒绝，不调用board.note_completion；只有finish_receipt填等待板。service模式成功也不通知board。因而类测试手工note_completion(REJECTED)不能证明真实QUEUE_FULL/TF拒绝会唤醒WaitSnapshot。
- adapt使用store_->published_epoch()转换原始队列消息，而终态仍以item.epoch入账。reset跨越适配时可能把旧epoch输入写入新地图；需用暂停/重置行为用例证实并修复。
- integrate_observation忽略ledger.admit的accepted，失败仍mark/enqueue，complete用operator[]插入。不能以admit类的4096容量测试证明service入口遵守容量/去重。enqueue失败catch也没有闭合先前PROCESSING记录。
- ledger.expire在节点无调用，query不检查期限；类测试显式expire不能证明线上EXPIRED。等待板completions_仅插入，未见清理/容量/归属，8等待上限不等于完成历史有界。
- wake_waits持wait_mu_调用store.snapshot，即使没有waiters也每5ms执行；该调用获取store锁。已有deferred入口不证明等待检查不阻塞回调线程。应验证持锁期间等待超时/压力/health，并避免空等待时访问store。
- inflight字节以单一store覆盖、回调填充后立即归零；不能表示并发响应总占用及仍存活响应。目录/队列/账只输出条数，不是已分配字节。uncovered=true披露了缺项，应保留；它不能使错误的已覆盖计数正确，也不能宣布资源门通过。

在线类型支持库加载未完成单列为验证缺口，不把它冒充产品性能失败。最小服务行为验证属于S1接口交付，长预演与O9仍在S2。

## Acceptance

S1 gen3继承主计划c02c1cfbd9ba16a1c4a6c6169695f1daf8eb6a76 A–D与全部边界；本记录补充缺陷闭合方式。
owner eng/cursor/grok-4.7，base=32a3267e633cc79d0e912531fabd4684212b53e0。仅S1修复/构建/行为测试，不正式评分。

1. 所有权跟踪覆盖store、句柄和进行中读取/响应，块/目录/缓存按对象去重，统计实际容量。独立probe三断言通过：第二共享句柄不增同一缓存；reset/release但仍有reader时保留归属；最后所有者释放后归零或明确当前视图基线。补两版本、TTL、reset后的实际读取、并发响应及最终释放用例。未覆盖分配逐类列明，资源认证仍not_evaluated直到覆盖充分。
2. 用统一终态闭合路径处理topic和service的完成/拒绝，并在发布前持久化；WaitSnapshot必须从同一终态事实获知完成/拒绝，不靠测试手工注入独立成功。真实触发入口队列满、worker队列满、TF失败、service完成/失败，验证目标等待返回合同状态（拒绝为REJECTED并保留原因），不是TIMEOUT。所有终态带原完整身份，query与发布内容一致；NOT_SEEN不得变成成功。
3. 两入口遵守admission/去重/4096容量，不在拒绝后继续积分，不漏PROCESSING记录；账容量满新ID明确拒绝，在途不淘汰。重复ID不重复更新；保留120秒前可查询，期限后实际查询EXPIRED，清理不破坏合同。完成历史同样有界并计入资源，reset后无旧epoch残留无限增长。
4. 真实暂停原始适配、执行reset再恢复，旧epoch消息不得写入新epoch地图；验证回执/查询身份一致。保留原单线程、全部有效射线与概率模型。
5. 等待唤醒不持回调线程等待store锁；无pending时不做快照工作。真实持锁期间能处理health、WAIT_PRESSURE/epoch/timeout；持锁超过请求timeout也能按截止处理，不能等锁释放后才检查。成功必须依赖目标完成及observed revision，保留2秒恢复合同；8个实际pending请求后第9个明确压力。
6. 修复可重复的ROS构建/运行环境，保存精确source/env/库路径及构建身份；至少完成GetObservationStatus、WaitSnapshot、GetResourceUsage真实服务调用并验证上述适用路径。补充原核心回归与S0 12测试，无Gazebo/planning，不提前跑O5–O9正式样本。失败保存后再复跑，不能仅grep源码证明行为。
7. 证据新目录s1_g3，保留s1/s1_review。完整SHA、dirty、命令、逐行为机器可读结果、资源覆盖表与剩余未覆盖项，owner端到端提交关闭。未完成条目不能标pass。

## Dispatch decision

S1 gen2原Claim/Result保留并标superseded，派发同谱系gen3，plan_revision取本记录首次提交完整SHA。
S2/S3继续dispatch_ready=no。最高代补修通过并复核后再绑定S2；不改变历史失败、不新增1cm、不解锁公平比较。

## Pointers

- `docs/plans/OCC_OCTOMAP_REPAIR_2CM_G1_20261031.md`
- `docs/agents/discuss/2026-10-31_0857_octomap-2cm-repair-s1-g2.md`
- `docs/status/evidence/occ_octomap_repair_2cm_20261031/s1/FINAL_REPORT.md`
- `docs/status/evidence/occ_octomap_repair_2cm_20261031/s1_review/MANIFEST.json`
- `docs/status/evidence/occ_octomap_repair_2cm_20261031/s1_review/COMMANDS.sh`
- `32a3267e:src/uld_occ_observer/src/observer_node.cpp`
- `32a3267e:src/uld_occ_observer/src/snapshot_repository.cpp`
