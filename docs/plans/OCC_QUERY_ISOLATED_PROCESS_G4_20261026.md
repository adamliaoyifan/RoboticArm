# OCC 独立只读查询进程：G4 设计与验收计划（冻结稿待审）

- parent: OCC-QUERY-INDEX-20261025
- generation: 4
- status: draft-for-review rev2（含评审必改 5 项冻结；未派实现）
- decision: 用户 2026-10-26 选定方案 (a) 独立只读查询进程
- predecessor: G3 blocked 收口（26339903/90f789ca；证据
  occ_query_index_g3_20261025/p0/ROI_CAUSE.md 含 D1 四分账）
- attribution_boundaries（继承用户裁定，不得在 G4 材料中越界）：
  1. G3 慢请求 668-695ms 墙钟−线程CPU 差**支持"线程未获执行时间主导"，
     不能单独证明全部由 GIL 导致**；
  2. 0.37Hz 写占空失败**已足以否决代表组**，不声称 2Hz 必然更差。

## 1. 目标与不变式

目标：把 v2 查询服务移入独立只读进程，使查询墙钟不再与写入进程的
CPU/GIL/调度耦合。**隔离不承诺写入吞吐自动达标**——若写入路径本身
在正式并发门下失败，另行立单，不得以查询隔离掩盖。

不变式（违反任一即设计缺陷，不是实现权衡）：

| 部分 | 冻结要求 |
|---|---|
| 写入权 | OccupancyMapStore 仍是**唯一** OCC 写入者；融合/射线/阈值/占用语义/提交/幂等逐字不动 |
| 查询进程 | 只持有**已提交的不可变派生快照**（QuerySnapshotData 及其块）；无 mapper 引用、无任何写路径、不加载 OCC 融合代码路径 |
| 数据传递 | 初始=完整快照一次传输；后续**只传变化块**；传输由后台发布者承担，**查询请求路径不得同步回调繁忙 mapper/写入者**；背压具名（不能无限积压也不能丢代不报） |

**rev2 冻结（评审必改项1）——变化集合与传输字段**：

- **增量基线禁用 digest 对比**。已核实 `CanonicalFactDigest.block_digest`
  仅覆盖 evidence leaves+active+box_members，**不含 frontier**：邻块 cell
  变化可翻转本块 frontier 而本块 digest 不变。变化集合必须复用写路径
  既有脏集：`block_ids(frontier_candidates) ∪ 变化 cell 所在块 ∪
  geometry_dirty`（frontier_candidates 本就含变化 cell 的 ±1 邻域，
  与 `_update_blocks` 同源），导出器不得自行重算或裁剪。
- **仅确认态变化**（settled/acquisition_stamp/map_source/cargo_observed/
  change_token/last_settle/ledger 等 provenance 腿变化、零 cell 变化）
  也必须推进代际：这些腿参与 `freeze_query` 的 root identity，导出=
  零块增量 + 新 identity/summary。
- **传输字段冻结**（导出单元=一代完整可重建集）：grid（shape/origin/
  resolution/geometry_hash）、每块 {evidence, active, frontier,
  flat_indices, state_counts, frontier_count, sensor_members, box_members,
  distance_layout 引用+distance_members, start/shape/block_id}、
  summary_values、map_snapshot provenance 腿、identity root 材料、
  sensor/box 块映射表。逐文件 sha256 校验；**frontier 字段为传输成员**，
  不靠 digest 推断。
- **接收端对照测试（新增门，必须过）**：(T1) 跨块邻居变化——块 A 内
  cell 状态变化翻转邻块 B 的 frontier 而 B 的 fact_digest 不变 → 副本
  必须服务更新后的 B.frontier/frontier_count（单进程基准逐字段全等）；
  (T2) 仅确认态变化——零 cell 变化的 settled/stamp 变化 → 代际推进、
  块字节全等、identity 推进、summary/provenance 全等。 |
| 版本一致性 | 代际**完整接收后原子发布**（部分代不可见）；请求身份不匹配→沿用现有具名拒绝（SNAPSHOT_EXPIRED / SNAPSHOT_IDENTITY_MISMATCH / RESOURCE_LIMIT 语义不变）；**绝不静默返回旧图**；两代保留/token 5s/512MiB 资源语义保持 |
| 故障处理 | 缺代、查询进程重启、传输中断=具名状态+**重新同步协议**（有界全量重传）；查询进程死亡不得影响写入主链路；重启后冷启动时行为=QUERY_INDEX_UNAVAILABLE 而非半新半旧 |
| 成本核算 | 快照构建、序列化、传输、接收重组、**双方额外内存/RSS**全部计入正式计时窗；不得把复制/传输挪到窗外；写入侧新增发布开销计入写入门（P95 比 b33×1.10 不变） |

## 2. 架构草案（实现前须逐项签核）

- **写入侧发布者**：store 提交路径在既有 freeze_query 之后追加
  "export 增量（changed blocks + summary + identity）"步骤；export
  本身有界、失败具名、绝不回滚已提交的地图事实。
- **传输（rev2 冻结，评审必改项3——不再保留候选清单）**：选定
  **代际目录 + 只读 mmap**：写端把每代导出为不可变代目录（逐块/逐腿
  文件+逐文件 sha256+manifest），在**有界代根**（tmpfs 或配置目录）下
  以**原子 rename 指针文件**发布；副本进程 mmap 只读加载，零拷贝、
  崩溃安全、字节级可校验、开销可直接计量（写出/读入字节数入账）。
  拒绝 DDS 载荷通道的理由入档：大帧 DDS 丢包前科（D555 实测）+
  全量快照可达数百 MB 超出话题语义；DDS 仅用于存活/触发信号话题。
  **冻结协议要素**：base/target identity=代 manifest 内完整 5 腿
  identity+逐件摘要；分块校验=逐文件 sha256+manifest 总校验；原子
  发布=指针 rename（部分代不可见）；队列上限=代根保留 ≤2 完整代+
  导出 staging ≤1（越限先删最旧已发布代，删不掉=具名
  EXPORT_BACKLOG_FULL 并阻塞新代发布而非丢代）；重同步=副本序号
  断档/校验不符/epoch 变 → RESYNC_REQUIRED → 全量重导（有界重试）。
  **服务名所有权**：v2 查询服务名**仅副本进程注册**；写端进程不再
  注册任何查询服务（杜绝同名双注册）。**死亡语义**：副本死亡时由
  客户端侧具名观测服务不可用（客户端 harness 记
  SERVICE_UNAVAILABLE，含发现超时上限），**不要求死亡进程返回错误**；
  写端不因副本死亡失败（隔离门覆盖）。
- **查询进程**：迁移现有 QuerySnapshotRegistry + MapQueryService 到
  只读进程；registry 语义（两代/会话/token/资源账）原样搬移；
  IDL/服务名/响应字段零变化（客户端无感）。
- **同步协议**：订阅发布者代流；序号连续性校验；缺号→RESYNC_REQUIRED
  （具名）→有界全量重传；身份锚（epoch/geometry_hash）不符→具名拒绝。
- **启动/监管**：launch 集成、启动顺序（mapper 先行、查询进程可后起、
  冷启动=UNAVAILABLE）、进程死亡重启策略与心跳。

## 3. 验收（全部重跑，不沿用任何旧通过结论）

原 G3 门**逐项不变**：主语义 180000 条对照 b33；原 316+回归清单；
ROI/frontier/instance 三分辨率×0/5/20×cuboid/chamfer（P95≤100ms、
max≤250ms）；summary/count 50/200ms；distance 1/256 5-20/50-200ms；
surface/snapshot 100/250ms；writes 72 组 P95≤b33×1.10、max≤b33max×1.20
+20ms、事实等价；resources init/reset≤30s、RSS≤2GiB、retained≤512MiB
（**双方进程合并核算**）；capture 198 组 P95 开关比≤1.10；placement
18 组 612 对全字段一致、P95≤5s/max≤20s；concurrency 1cm 0/5/20 箱
×120s×3、原 2Hz 写/10Hz summary/5Hz ROI/2Hz distance/1Hz surface
**不降频**、backlog≤1s、心跳双向 <1s、零 lease。

G4 新增门（隔离专属）：

| 门 | 通过标准 |
|---|---|
| 快照新鲜度（rev2 冻结，评审必改项2） | 稳态：提交→副本可见新代 **P95≤1s 且 max≤2.0s**（P95 不足以约束最坏陈旧；max 门并行冻结；全量重同步路径单列 ≤30s）。CURRENT/异步副本语义逐条冻结：① `use_current`=**副本本地最新完整代**（天然有界陈旧，由 max 门约束；文档化此语义，客户端不得假设写端即时一致）；② 请求 pin **更新（未收到）身份**→具名 `SNAPSHOT_IDENTITY_MISMATCH`（沿用现语义，不等待不阻塞——等待会重新引入查询路径耦合）；③ 旧 token 可读窗口=现有语义不变（token 5s 生命+两代保留内可读，逐出后具名 `SNAPSHOT_EXPIRED`）；④ 同步超时/断档→`RESYNC_REQUIRED` 具名+全量重同步（重同步期间服务上一完整代或 `QUERY_INDEX_UNAVAILABLE`，绝不半代混发） |
| 隔离正确性（rev2 可执行定义，评审必改项4） | **杀进程矩阵**：N=3 个阶段 × 各重复 3 次 = 9 次杀（阶段：a=READY 空闲；b=增量接收中途；c=查询突发中）；每次注入记录杀时刻/代序号/在途传输状态；聚合判定（AND）：写端主链路 9/9 零失败；停机窗内客户端 100% 具名 SERVICE_UNAVAILABLE（发现超时 ≤2s）；重启+重同步 9/9 ≤30s 恢复 READY；恢复后与单进程基准逐字段/identity/cursor 全等。**内存核算口径**：双方进程 RSS 以 1Hz 连续采样贯穿 构建→staging→传输→READY→杀/重同步 全程（不只 READY 后），**传输缓冲、在建快照、导出 staging、双方保留代全部入账**，合并峰值 ≤2GiB、retained ≤512MiB 不变；证据=采样时间序列+峰值表 |
| 传输故障注入 | 丢块/断流/乱序：具名 RESYNC_REQUIRED，恢复后与单进程基准逐字段全等 |
| 成本可见 | 快照构建+传输+接收的 P95/max 单列成表；写入 P95 增幅含 export 开销仍 ≤×1.10 |
| 进程隔离行为验收（rev2 改名降级，评审必改项5） | 验收=行为证据三件套：① 进程分离实证（副本与写端不同 PID，查询路径无 mapper/写端符号调用——代码审查+span 证据）；② 查询请求路径零同步 mapper 回调（span+设计审查）；③ **原并发延迟门全量通过**（120s×3×0/5/20 箱，原频率不降）。**"wall−CPU≤5ms"不再是验收门、不得称"GIL 解耦证明"**（独立进程仍共享系统调度）——保留为**诊断指标**（D1 harness 复测，对照 G3 668-695ms 作参考，不设过/不过线）。原并发门**未经用户裁定不得删除或放宽** |

## 4. 明确不做 / 保留项

- 不改融合算法、写语义、cursor/hash 配方、IDL、查询语义、GC 策略、
  执行器拓扑（除新增独立进程本身）。
- (c) 查询内批量化 numpy：保留为**可选局部优化**，非本代主线（纯查询
  仅 ~9ms，非主要矛盾；typed 构造 18ms 若在新进程下不再是瓶颈则不动）。
- 不以查询隔离声明写入 2Hz 吞吐达标（写入门独立判定）。

## 5. 派发结构（实现须另单）

建议分两单：G4-A（发布者+传输+只读进程+隔离正确性/故障门，离线+无运动
ROS）；G4-B（正式全矩阵+并发 120s×3+placement 对照）。均以本计划为
plan_revision，绑定实现前 HEAD。
