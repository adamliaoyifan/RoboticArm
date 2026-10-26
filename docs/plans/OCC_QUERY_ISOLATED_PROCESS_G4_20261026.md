# OCC 独立只读查询进程：G4 设计与验收计划（冻结稿待审）

- parent: OCC-QUERY-INDEX-20261025
- generation: 4
- status: draft-for-review（冻结待用户/审查签核；未派实现）
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
| 数据传递 | 初始=完整快照一次传输；后续**优先只传变化块**（块级 fact_digest 已具备不可变去重基础）；传输由后台发布者承担，**查询请求路径不得同步回调繁忙 mapper/写入者**；背压具名（不能无限积压也不能丢代不报） |
| 版本一致性 | 代际**完整接收后原子发布**（部分代不可见）；请求身份不匹配→沿用现有具名拒绝（SNAPSHOT_EXPIRED / SNAPSHOT_IDENTITY_MISMATCH / RESOURCE_LIMIT 语义不变）；**绝不静默返回旧图**；两代保留/token 5s/512MiB 资源语义保持 |
| 故障处理 | 缺代、查询进程重启、传输中断=具名状态+**重新同步协议**（有界全量重传）；查询进程死亡不得影响写入主链路；重启后冷启动时行为=QUERY_INDEX_UNAVAILABLE 而非半新半旧 |
| 成本核算 | 快照构建、序列化、传输、接收重组、**双方额外内存/RSS**全部计入正式计时窗；不得把复制/传输挪到窗外；写入侧新增发布开销计入写入门（P95 比 b33×1.10 不变） |

## 2. 架构草案（实现前须逐项签核）

- **写入侧发布者**：store 提交路径在既有 freeze_query 之后追加
  "export 增量（changed blocks + summary + identity）"步骤；export
  本身有界、失败具名、绝不回滚已提交的地图事实。
- **传输**：候选机制（评审时定一）：进程内共享内存段 / DDS 话题
  （分块消息）/ 文件+inotify。评审判据：200KB-6MB 级代际增量的
  延迟、丢包语义、重启恢复、测量可见性。
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
| 快照新鲜度 | 提交后 ≤1s 内查询进程可见新代（P95）；期间旧请求按现语义拒绝或用旧代，不得混代 |
| 隔离正确性 | 杀查询进程×N：写入主链路零失败；查询侧全程具名状态；重启+resync 后 ≤30s 恢复 READY 且语义/身份全等 |
| 传输故障注入 | 丢块/断流/乱序：具名 RESYNC_REQUIRED，恢复后与单进程基准逐字段全等 |
| 成本可见 | 快照构建+传输+接收的 P95/max 单列成表；写入 P95 增幅含 export 开销仍 ≤×1.10 |
| GIL 解耦证明 | 同 D1 harness 复测：写 in-flight 时查询 wall−CPU（未获执行）≤5ms（对照 G3 的 668-695ms） |

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
