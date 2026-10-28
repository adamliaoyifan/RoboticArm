# SEQ_GAP 有界 catch-up：不变式与协议处理规格

- date: 2026-10-28
- author: eng/claude (gen3, review_5c5c1842 Finding 2 候选修复的先行规格)
- status: approved（用户于2026-10-28明确批准；reviews准入记录见§7）
- atomicity: **单代原子**（见 I6 修订）
- scope: cargo_query_replica SEQ_GAP 恢复路径；不改 CURRENT、不放宽
  校验、不无限保留历史

## 1. 问题

现行为（cargo_query_replica.py poll）：最新发布 manifest 的 seq 跳过接收端
下一期望 seq 时，一律请求 FULL resync（~1s@1cm 全量重载）。接收端一次轮询
停滞（GC/调度）即可被放大为整场重建，窗口内提交的可见性延迟落入 1-2s
（A7 D1 b0 FAIL 的量化根因）。

## 2. 提议

SEQ_GAP 时先尝试**有界 catch-up**：从 generations/<inc>/<seq>/manifest.json
逐个读取被跳过的中间代，按既有 DELTA 校验链连续应用；只有链不可用/不完整
时才退回 FULL resync。

## 3. 不变式（全部必须成立，任一不满足→具名 resync，绝不静默跳过）

I1 同世界：中间代与当前持有的 producer_incarnation、map_epoch、
    wire_schema_version、geometry_hash 完全一致。
I2 连续性：从期望 seq+1 到最新 seq 逐 +1，无缺（文件缺失→SEQ_GAP 具名
    resync）。
I3 链锚：每个中间 DELTA 的 base_identity == 前一 manifest 的
    target_identity 且 base_manifest_sha == 前一 manifest 的 checksum
    （第一环锚定接收端当前持有的 manifest）。FULL 中间代视为自包含新锚，
    可直接应用（同现行 FULL 语义，含 MANIFEST_SEQ_AHEAD 守卫）。
I4 对象验证：每个 manifest checksum 自洽（validate_manifest）且逐对象
    digest 校验（复用 _load_payload 语义，含 heal hint）。引用条目必须
    在已持有对象集合中存在（不存在→OBJECT_MISSING 具名 resync）。
I5 引用保护：仅内容寻址对象，无路径外写；manifest 内 entry 表自包含。
I6 有限性与原子性（修订 2026-10-28，review_d9a28e4e P2）：**单代原子**。
    每一环先完整校验（锚点/表/对象 digest）再交换 snapshot/manifest——
    已成功应用的前环是完整的有效代并**允许保留**；后环失败时接收端停在
    最后一个已验证代（绝不是半应用代），CURRENT 具名拒绝（落后），
    已发出的 pinned 句柄仍绑定其获取时的有效代、不漂移，错误身份绝不
    READY，随后走具名 resync。不是整链原子（无 staging-后一次发布）。
    限额：单次至多 CATCHUP_MAX_GENERATIONS（32）环、CATCHUP_MAX_BYTES
    （512MiB wire 对象字节）——这两个是**环数与 wire 字节上限**，不是
    耗时预算也不是统一内存预算；超限在链构造阶段（应用前）退出→具名
    回退 resync。阻塞 IO 不承诺可中断。
I7 记账不变：512MiB 派生预算口径不变；catch-up 增量对象计入同一对象缓存
    与清理逻辑，不新增无限保留。
I8 可观测：catch-up 尝试/成功/环数/字节数记入事件日志（事件
    'catchup'），失败原因具名；ACK/head/ownership 语义不变。
I9 竞态：重读 published.json 于 catch-up 完成后进行；若期间指针前移，
    下一轮 poll 正常处理（无假设单写者以外的串行性——写者本就唯一）。
    清理竞态（文件在读时被删）→ OSError→具名 resync。

## 4. 协议处理

- SEQ_GAP 事件仍记录（不抑制）；仅恢复动作从"必然 FULL"变为"先 catch-up
  后 FULL（失败时）"。
- resync 请求与 heal hint 机制不变；catch-up 失败的 resync 理由带
  'CATCHUP_FAILED:<reason>' 前缀以便归因。
- 现行其余守卫（PRODUCER_CHANGED/BASE_MISMATCH/MANIFEST_SEQ_AHEAD/
  MANIFEST_CONTENT_CONFLICT/DELTA_WITHOUT_BASE）优先级不变，先于 catch-up。

## 5. 测试矩阵（review 指定七项）

T1 链完整：3 中间代全部在盘→逐环应用、READY、内容全指纹等价、事件
   'catchup' 成功（环数=3）。
T2 链缺失：中间代文件被删→具名 resync、恢复、内容等价。
T3 链损坏：中间 manifest 改字节/checksum 不符→具名 resync、不部分应用。
T4 同 seq 冲突：中间代 seq 与磁盘另一内容共存/伪造→MANIFEST_CONTENT_
   CONFLICT 路径仍拒。
T5 世界切换：中间代跨 epoch/producer→不 catch-up，直接既有 PRODUCER_
   CHANGED 处理。
T6 清理竞态：catch-up 读对象时删除文件→OSError→具名 resync、恢复。
T7 持续发布：catch-up 期间写者继续发布（指针前移）→完成或重评，最终
   READY 于最新代、内容等价、无半应用状态。

外加确定性微测（review Finding 2 要求）：屏障 hold 接收端→发布 3 代→
释放→观察 catch-up（而非 FULL）恢复，记录 received/base/poll/READY 时刻
与提交覆盖。

## 6. 非目标

不改 CURRENT 严格性；不加无限历史保留；不抑制 SEQ_GAP 事件；不把失败
包装为成功；不动 512MiB 预算口径。

## 7. 正式准入记录（2026-10-28 16:57 +08:00）

用户明确批准 catch-up 协议和资源门归属裁定。本规格由 proposed 转为 approved，准入实现基点为 `0da4086bce23be49ec8eeb934a15fb6829803abe`；性能收益不是批准依据，既有失败和评审历史不改。

批准的是 I1-I9，包括单代原子、后环失败保留有效前环、严格 CURRENT 及原 pinned 生命周期、链不可用时具名 FULL resync。T3的“不部分应用”指单代不得半应用，不撤销已验证前环；不得解释为整链原子。

32环和512MiB wire字节仅为结构限额，不批准“已具有墙钟中断能力”或“统一派生资源已过门”的声明。实际稳态传播1s/2s、故障恢复30s及统一资源门仍由验收检查；阻塞IO造成超门应如实FAIL，不伪造可中断保证，不另行扩大历史保留或预算。

资源归属由 `docs/plans/OCC_G4B_ACCEPTANCE_GEN3_20261028.md` §7规定：完整部署矩阵与统一资源补证由G4-B承担，不要求A重复既有性能/故障门。本次准入不代替B验收PASS，不改变生产默认模式。
