# OctoMap 修复与 2 cm 验收方案——Cursor / Grok 4.7

- approval: 用户于本会话批准全文并要求执行、派给 Cursor Grok 4.7
- parent: OCC-QUERY-INDEX-20261025
- owner: eng / cursor / grok-4.7
- generation: 1
- product_base: 4c8f1862ec357cb01616f4b04be636227461cf44
- tool_base: 57d8c0bfe23fee54920c89c06a2b6bdcb97de976

## 1. 目标与调研结论

由 **eng / cursor / grok-4.7** 完成实现、测试、修复、提交和最终验收，不另交测试角色。本次采用已确认的范围：

- 允许兼容新增回执查询、资源统计和等待新快照接口。
- O5/O7 使用 2 cm；O6 使用 2/5 cm 共 96 组，不新增 1 cm 测试。
- 恢复采用显式等待新快照的新合同；旧立即查询合法的 `STALE` 和历史失败均保留。
- 不接 planning、不启动 Gazebo、不迁移生产、不自动解锁后端公平比较。

调研确认：

1. **Surface 存在可消除的锁依赖。** 已有快照的 `get()` 为检查 epoch 调用 `store_.diagnostics()`，会等待积分持有的 store 锁。它解释了阻塞机制，但不能解释所有历史 Surface 耗时。
2. **订阅入口有阻塞工作。** 回调同步等待 TF，并读取同一 store 诊断；订阅深度仅为 2。ROS 2 消息在被回调取走前保留于中间件，`KeepLast` 又受深度限制。因此应先缩短入口工作，而不是只扩大积分队列；不能据此把历史缺口直接定性为 DDS drop。[ROS 2 QoS 官方文档](https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst)、[执行器官方文档](https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Concepts/Intermediate/About-Executors.rst)
3. **ApplyBatch 已做帧级批处理。** 当前使用 lazy 叶节点写入，再执行父节点更新与剪枝；本轮不重复实施已有优化。OctoMap lazy 更新要求在需要一致树状态前补齐内部节点维护。[OctoMap 1.9.8 源码](https://raw.githubusercontent.com/OctoMap/octomap/v1.9.8/octomap/include/octomap/OccupancyOcTreeBase.hxx)

产品基线为 `4c8f1862ec357cb01616f4b04be636227461cf44`，工具基线为 `57d8c0bfe23fee54920c89c06a2b6bdcb97de976`。历史证据只读；产品变化后的成绩使用新目录、新版本。

## 2. 修复方案与接口

### A. 快照读取与 Surface

- 增加独立的 epoch 发布机制，已有快照读取不再调用 store 诊断或取得积分锁。
- 仓库锁仅保护句柄、TTL、容量和共享所有权；不得持有仓库锁等待 store 锁。明确 reset 的线性化边界，旧 epoch 句柄不可重新取得，已经开始的不可变读取按原合同完成。
- `acquire()` 获取最新快照后再进入仓库临界区，并复核 epoch，避免 reset 竞态。
- 保留不可变块和按内容版本复用的 Surface 缓存；预分配响应数组，消除逐列扩容。不得缩减列数、改变分页或用缓存命中代替首次构建。
- 分段记录仓库等待、store 等待、Surface 构建/命中、响应填充和客户端总耗时。不能直接观测的传输、调度残差保留为残差。

**行为门：**人为暂停积分、令其持有 store 锁时，已有句柄的 ROI/Surface 读取必须能够先完成；reset、TTL、release、并发读和缓存内容均与独立 oracle 一致。

### B. 入口与回执闭合

- 订阅回调只提取身份、登记入口事件并投递有界原始消息队列；移出 TF 等待、点云转换和 store 诊断。
- 增加一个串行适配线程处理同时间戳 TF 和转换，再送入原单线程积分 worker；保持顺序及原有去重规则。
- 原始消息等待队列容量设为 2，积分等待队列维持 2；任一队列满都返回带完整 ID 的明确拒绝。
- 为入口 callback group 配置独立执行器线程；查询与 health 保留独立 callback group。健康发布不得同步等待积分锁。
- 点云发布/订阅使用显式 `Reliable + KeepLast(32)`；回执深度设为 1024。容量只是本次运行配置，不作为交付可靠性的证明。

新增 `GetObservationStatus`：

- 以 epoch、sensor ID、observation ID 查询，返回 `NOT_SEEN / QUEUED / PROCESSING / COMPLETED / REJECTED / EXPIRED` 及原始终态回执。
- 终态账容量 4096，保留至少 120 秒；不能淘汰在途记录。容量不足时明确拒绝新接纳。
- 终态必须先落账，再发布回执；查询不重做积分。
- 兼容入口沿用原 ID 构造规则，重复消息不得重复更新地图。

**关键限制：**`NOT_SEEN` 只能证明服务器未记录该 ID，不能算拒绝或成功。正式 topic 评测不自动重发；回执订阅缺口与通过查询找回的终态分别报告。未进入回调的未知 ID 仍使 O7 失败。

### C. 恢复等待

保留 `GetSnapshot` 立即查询语义，新增 `WaitSnapshot`：

- 输入：expected epoch、目标 observation ID、`max_age_ms`、`timeout_ms`。
- 目标输入完成后，返回包含该输入或更晚成功输入的快照；依据完成记录和 observed revision，不能只比较可能不变的 map revision。
- 使用 Humble 已支持的 deferred service response，等待期间不占用服务回调线程；完成事件及 5 ms 定时检查负责唤醒。
- 最大 8 个待处理等待请求；满额返回明确压力状态。reset 返回 epoch mismatch，目标拒绝返回对应拒绝，无新帧返回 timeout。
- 恢复计分使用 `max_age_ms=2000`、`timeout_ms=2000`。客户端从发起请求开始计时，包含全部等待。

新恢复门：第一条查询与第一条恢复输入同时开始，指定该输入；必须在 **2 秒内**返回 `OK`，快照年龄≤2秒。随后 29 条查询使用原立即查询接口及原延迟门。

旧接口“新帧未完成时立即首查”的结果另列，不能用新接口结果覆盖旧失败。

### D. 写入与资源

- 保持单线程积分、全部有效射线、概率模型和帧末维护，不引入 OpenMP。
- 本轮不为已退出范围的 1 cm 失败重写树算法；先验证新目标。若 2/5 cm 写入仍超门，保存失败并定位 `search / probability / setNodeValue / allocation / inner-update / prune`，不得在计分过程中修改产品。
- 新增 `GetResourceUsage`，输出当前值和峰值：共享块、视图目录、Surface 缓存、在途响应、入口队列、回执账。
- 读视图/缓存统计按实际所有权去重，同时计入仅被旧视图保留的块、目录及缓存；记录已分配容量，不能只统计有效元素。统计未覆盖的分配必须明确标记，不能宣布资源门通过。
- 测试多句柄共享、两版本共存、8 句柄上限、TTL、release、reset；释放最后一个所有者后，相关归属字节应归零或回到明确的当前视图基线。

## 3. 评测流程与指标

### 阶段顺序

1. **离线整理：**读取现有补证结果，形成事实/机制/假设清单；补证缺项不阻止明确代码缺陷的行为测试。
2. **实现与行为测试：**完成 A–D、接口兼容和判官正负例；失败注入必须真实触发对应路径。
3. **非计分预演：**验证启动、TF、独立发送、就绪、终态账和失败重放；允许修复后重复，不消耗计分样本。
4. **冻结：**提交产品、工具及计划，生成输入 manifest、运行命令和产物 hash。冻结后才进入正式评分。
5. **评分：**正确性与捕获资格 → O5 → O6/O7 → O8；O5 失败停止后续 O6/O7，保留安全可执行的正确性/资源测试。
6. **收口：**独立复算每组、每窗，分别报告产品结果与工程交付结果。

### 环境与输入

- 当前 x86 主机，Release、ROS 2 Humble、OctoMap 1.9.8、`discretize=false`。
- 调速器固定为历史 `powersave`；不绑核。客户端执行器显式设为 4 线程，发送调度独立于响应处理。
- 记录实际 RMW、DDS 配置、CPU 拓扑、线程数、逐线程 CPU、上下文切换、RSS、外部负载和频率；不把逻辑核数当实际利用核数。
- 持有 `/tmp/elfin-occ-backend-perf.lock` 串行测量；等待其他重负载结束，不终止他人进程。
- 保留原生成器的几何、点数、outside/cross 定义及 seed 序列；冻结每个最终输入 hash。
- 性能 ROI 固定 `{0,0,0}→{16,16,16}`，4096 cells。另测原 1 cm 对应的固定物理 ROI `[0,0.16 m)^3`，在 2 cm 下为 `8³` cells，用于语义对照，不混入 4096-cell 性能统计。

### 正式门限

| 项目 | 工作量与重复 | 通过条件 |
|---|---|---|
| O5 | 2 cm、倒角、0/20箱×30k/100k；每组3预热+30评分，另30首次构建样本 | 30k积分 P95≤500 ms、max≤1000 ms；100k≤2000/5000 ms；acquire、ROI、首次/更新后/重复 Surface 分别≤100/250 ms |
| O6 | 2/5 cm×0/5/20箱×倒角/长方体×30k/100k×outside .5/.9×cross 0/.5，共96组；各3+30，另30个新进程冷启动样本 | 每组独立满足上述门；初始化≤30秒；RSS≤2 GiB；读视图/缓存峰值≤512 MiB |
| O7 正常 | 2 cm倒角、30k、outside .5/cross 0；0/5/20箱×3次×120秒 | 每窗240输入、120查询链；全部完成、零拒绝/未知/重复处理；积分与查询沿用原门；queue age max≤1秒；health最大间隔<1秒 |
| O7 过载 | 20箱、10 Hz×60秒=600输入，1 Hz查询；固定30秒排空阶段 | 600 ID终态闭合；允许明确拒绝，已接纳输入全部完成；无未知、崩溃、死锁；排空后在途为0 |
| O7 恢复 | 随后2 Hz×30秒=60输入、30查询 | 60输入完成；首条 WaitSnapshot≤2秒且快照年龄≤2秒；其余29条原接口查询全部成功并满足原延迟门 |
| O8 稳定性 | 固定射线、递增时间戳，180秒×3；首60秒预热；另30帧饱和检查 | 饱和后概率变化键和几何变化块均为0；无新键时节点数稳定；后两个60秒 RSS中位增长≤5%；资源有界 |
| O9 捕获扰动 | ROI、30k融合、首次Surface各30对ON/OFF，AB/BA交替 | P95比≤1.10；无缺失/错配，所有注入失败可重放；不合格操作不得认证性能 |

补充规则：

- O8 再做无输入、遮挡各10秒×3：旧概率保持，未接纳新数据时新鲜度不得前移。
- O7 过载必须至少观察到一次实际排队或拒绝，否则标 `not_evaluated`，不得自动升压后择优报告。
- 新产品影响公共查询、资源与入口，本次 **96组重新评分**；旧同版本数据保留作基线，不能给新版本代签。
- P95统一取排序后第 `ceil(0.95n)` 个；n=30时为第29个。缺样不通过，超时和错误不得从分母删除。
- 正常窗要求本次有限集合内每个输入/请求100%成功；九窗为 AND，不宣称长期100%可靠。延迟门要求每组实测P95及max满足阈值，不跨组平均。
- 判官必须能拒绝：单窗失败、少一帧、重复终态、全拒绝、回调前缺口、恢复timeout、错误快照、资源缺项。用健康与这些故障对照验证判别能力。

## 4. 证据与执行约束

正式计分前，Cursor 将下述程序固化为可运行 `COMMANDS.sh`：构建→行为测试→预演→O9→O5→O6→O7→O8→独立判官→失败重放。每一步固定实际参数、输入和环境，不保留待填写占位符。

| 边界 | 常开证据 | 失败时保存与检查 |
|---|---|---|
| 发布→回调→入队 | 唯一ID、计划/实际时刻、TF身份、队列状态、拒绝原因 | 输入生成manifest/无损输入、TF、逐ID时间线；重放入口与拒绝 |
| 积分→终态→回执 | 输入ID、完成identity、分段时间、账本及发布/接收事件 | 可重建累计地图的完整输入前缀、终态原文；复算守恒 |
| acquire→ROI→Surface→release | 请求ID、快照身份、年龄、状态、内容hash、分段耗时 | 请求/响应、快照重建输入、失败前后窗口；校验内容与释放 |
| 资源与恢复 | 归属字节、RSS、health、等待目标与截止时间 | 所有权清单、epoch变更、完成事件；检查泄漏、过期和超时 |

- T0/T1全程保留；T2默认失败前后各5秒，另保存必要完整地图输入前缀。若预演测得边界延迟超过窗口，评分前扩大并冻结。
- 每例详细转储≤512 MiB、每run≤2 GiB、总计≤8 GiB；使用压缩和可校验生成manifest，不能截断重放关键数据。
- 写明 `capture_complete`、`replay_possible`、缺失项和SHA；失败证据持久化后才能拆卸或重跑。
- 预演限2小时，正式测量限8小时，均不含等锁。启动失败不消耗评分样本，单独报告次数/率；预算耗尽的项目为 `not_evaluated`。
- 产品、工具及配置在评分期间冻结。若需修复，结束该候选版本评分、保留失败，提交新候选并重跑受影响门，不能拼接多个版本凑通过。
- 不开展 Orin 推算或跨环境速度比较。

## 5. Cursor 派工与交付

在独立工作树基于工具基线实施，避免与现有离线补证任务争写；所有协调记录仍写主工作区的 `docs/agents/`。

| 子任务 | Owner | 依赖 | 交付与关闭条件 |
|---|---|---|---|
| S0 合同、基线与证据工具 | cursor / grok-4.7 | 无 | 冻结manifest、完整门表、分段事件及判官正负例；重放验证通过 |
| S1 产品修复与兼容接口 | cursor / grok-4.7 | S0 | A–D实现；锁竞争、入口、回执、恢复、reset/TTL、资源归属行为测试全部通过 |
| S2 预演与捕获资格 | cursor / grok-4.7 | S1 | 启动及接线预演通过，O9合格；冻结产品/工具SHA和精确命令 |
| S3 最终集成验收 | cursor / grok-4.7 | S2 | 执行全部适用门、修复流程及独立复算；逐项给出pass/fail/not_evaluated和剩余阻塞 |

派发时使用父任务 `OCC-QUERY-INDEX-20261025` 下的新修复/验收子任务，不重写已领取的历史要求。计划提交后，把精确 `plan_revision`、各阶段基线、依赖和验收条件写入线程；仅就绪项设 `dispatch_ready=yes`。Cursor 通过 `agent_start.sh` 领取、记录角色笔记与提交证据，通过 `agent_complete.sh` 按实际结果关闭。

最终报告必须分开：

1. 工程修复与工具交付结果；
2. **2 cm代表目标及2/5 cm矩阵验收**；
3. 新恢复等待合同与旧立即查询结果；
4. 历史1 cm失败、未覆盖项及后续阻塞。

## 6. 派工绑定与执行入口

本节将已批准的 S0–S3 绑定到唯一任务ID，不改变上述范围和门限。

| 阶段 | subtask | kind | depends_on | 初始基线 | 初始dispatch_ready |
|---|---|---|---|---|---|
| S0 | OCTOMAP-2CM-REPAIR-S0 | subtask | none | 57d8c0bfe23fee54920c89c06a2b6bdcb97de976 | yes |
| S1 | OCTOMAP-2CM-REPAIR-S1 | subtask | OCTOMAP-2CM-REPAIR-S0 | 57d8c0bfe23fee54920c89c06a2b6bdcb97de976 | no |
| S2 | OCTOMAP-2CM-REPAIR-S2 | subtask | OCTOMAP-2CM-REPAIR-S1 | 57d8c0bfe23fee54920c89c06a2b6bdcb97de976 | no |
| S3 | OCTOMAP-2CM-REPAIR-S3 | integration | OCTOMAP-2CM-REPAIR-S2 | 57d8c0bfe23fee54920c89c06a2b6bdcb97de976 | no |

- 每个线程均绑定本文件首次提交的精确 SHA，generation=1。下游初始基线是谱系起点，不是允许忽略依赖的产品版本；前置Result必须提供完整提交SHA、dirty状态、构建产物身份与证据路径。
- 后续阶段在前置done/pass后，由reviews绑定前置实际提交再开放领取；若改变受保护revision，按代次规则发布同一subtask的更高generation，不原地重写已发布要求。尚未满足的依赖不允许提前领取。
- S0 owner领取后从工具基线创建独立工作树；推荐 `/tmp/elfin-occ-octomap-repair-2cm-g1`，分支 `agent/eng/octomap-repair-2cm-g1`。路径已有内容时先查归属，不覆盖。S1–S3沿用该谱系并核实依赖提交。
- 主工作区计划与证据通过绝对路径读取；不得把其他任务的未提交改动或整个master混入候选产品。阶段修改与最近的模块合同/测试一起提交。
- 证据根：主工作区 `docs/status/evidence/occ_octomap_repair_2cm_20261031/`；阶段使用独立 `s0/`、`s1/`、`s2/`、`s3/runNN/`，失败不覆盖。每阶段有 `FINAL_REPORT.md`、机器可读结果及实际命令；评分命令必须在S2冻结。
- S0只开展离线工具与行为验证；S1开展产品行为验证；S2开展非计分预演及O9资格；S3才开展正式O5–O8。S0的故障注入工具在S1/S2与真实产品边界集成验证后才允许认证，不能以模拟成功替代真实捕获资格。
- 旧离线补证任务 `OCTOMAP-INTERFACE-DATA-COLLECTION` gen2保留自己的范围；新授权仅适用于本谱系，不改变其“不得修产品/启动负载”边界。

## Pointers

- `docs/plans/OCC_RESOLUTION_TARGET_2CM_20261030.md`
- `docs/plans/OCC_O6_O7_EVALUATION_G1_20261029.md`
- `docs/plans/OCC_INTERFACE_CLOSEOUT_OFFLINE_G2_20261030.md`
- `docs/status/evidence/occ_o6_o7_20261029/closeout_g2/FINAL_REPORT.md`
- `docs/status/evidence/occ_o5_o7_interface_collection_20261030/run01/FINAL_REPORT.md`
- `docs/architecture/perception_architecture.md`
- `.cursor/rules/debug-evidence.mdc`
