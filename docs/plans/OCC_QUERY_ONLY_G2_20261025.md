# OCC 查询优化 G2：Claude 集成基座、只读派生索引与完整验收

- parent: OCC-QUERY-INDEX-20261025
- generation: 2
- status: approved-for-dispatch
- product_base_revision: 352191b3e73f400e17c2682f0ff296c47a1757d3
- product_base_branch: agent/eng/integration-prior-g1
- predecessor_plan_revision: c15d002d
- owner: eng/codex/gpt-6
- dispatch_ready: yes
- scope: 查询、派生索引及等价投影；不优化或扩展现有 OCC 观测/融合/占用能力。
- authorization: 实现、离线测试、独立构建、无运动 ROS 查询及 placement 服务验收；不授权 Gazebo、MoveIt 栈、运动、n=3 或容量实验。

## 1. 基座、继承与优先级

本代从 c15d002d 建立，后者直接继承 Claude 集成产品 352191b3。产品链包含 4b51f9da、777924a1、d1ba2e87、7a285eb4、352191b3；不从 master 或性能失败的 COST-CORE 分支重新集成。

本文件与同目录 `OCC_INCREMENTAL_QUERY_INDEX_G1_20261025.md` 共同构成完整实施规格。G1 的六项查询算法、DTO/API、分页/快照身份、测试矩阵、命令、证据矩阵全部继承；冲突以本 G2 为准。G1 的“仅文档/未派发”由本代授权取代，G1 本身不改写。实施者必须先阅读两份文件。

实施者领取后从本计划的实际提交创建 `agent/eng/occ-query-index-g2` 和 `/tmp/elfin-occ-query-index-g2`。不在 Claude 工作区编辑，不推进其分支或 master，不重建共享 install。任务使用独立 build/install/log。Claude 工作区未提交的 `scripts/g3/admission_lease_probe.py` 不读取为产品基准、不覆盖、不纳入提交。

## 2. 查询优化的严格定义

同一初态、同一更新序列和同一读取身份下，新旧 OCC 事实与现有公开读取结果必须一致。允许改变取得答案的代价，不允许改变答案、观测能力或准入决定。新查询只是已有事实的索引化读取，不产生新事实。

允许：

1. mapper 现有写操作边界收集 old/final 净变化，不改任何融合公式、运算顺序、采样顺序或接受条件。
2. store 同事务维护 stats/frontier/instance/block/dirty-column 派生索引，原子发布只读查询快照。
3. stats/frontier/高程旧读路径替换为等价索引/缓存；保留全量实现作测试 oracle，不作在线热路径。
4. 新增 G1 定义的只读 Python/typed ROS 查询服务、分页、身份校验、错误码、资源上限。
5. 节点薄适配查询回调；查询不在持有写事务长锁时执行全图运算。沿用现有 heartbeat 行为。
6. 几何/传感器实例反向索引仅引用已有身份；没有真实绑定就返回具名缺失，不能修改旧 ledger 或写入协议来创造新身份。

禁止：

- 改 OCC 分辨率、active/hull/crop、射线算法、点云预处理、hit/miss/log-odds、置信度、阈值、融合、状态转换、来源/标签、shadow、几何锁、撤销恢复、pending/confirmed、revision/幂等/重置语义。
- 迁移主存储：撤销 G1 §4 的主 logodds float64/lock 紧凑化要求。现有主数组/list 保持；仅新派生索引使用紧凑数组/bitset/COW。
- 新建观测源、重启 PARKED depth 通道、改变写接口或 Add/Remove/Reset 接受条件；索引失败不能丢失真实释放事实。
- 更改 Attachment、候选生成/排序/预算、atlas、支撑/间距/碰撞门、规划输出；禁止把距离查询替代碰撞认证。
- 改 heartbeat/lease 参数、readiness 重入策略、driver 超时、执行器或模型资产。现有 30s stats workaround 本代不改；测试使用原 5s deadline且达到更严格查询门。
- 重写写入 worker、执行器调度/回调组拓扑、全局线程策略。撤销 G1 §7 “门失败则必须实现 worker 隔离”的扩项要求；若确有该瓶颈，保存调度证据、标 blocked 并请求另单。

## 3. 修改白名单与类接口

可改目录/文件：`luggage_perception` 下 G1 所列新纯模块与测试；现有 `cargo_volume_mapper.py`（变更通知及读缓存）、`occupancy_map_store.py`（派生索引/快照）、`cargo_grid_projection.py`（等价增量读投影）、mapper node（只读服务适配）。允许接口包新增查询 srv/msg 及对应 CMake/package 登记，不改旧线合同。允许 `scripts/g3/occ_query_eval.py`、测试夹具、证据/文档与本任务独立构建脚本。

`luggage_packing`/`luggage_planning` 产品源只读；通过真实既有 placement 服务验证回归，不趁机改其算法。若接线必须改白名单外产品源，先提交具体 diff 提案，等待授权，不自行扩名单。

公有 API 沿 G1 §6：`acquire_query_snapshot/summary/get_cell/query_cells/query_frontiers/count_frontiers/query_instance/nearest_obstacles/surface/surface_changes`。输入身份、坐标、region、filter、page；输出冻结 DTO、状态、同版本身份、完整性、游标、计数及耗时。没有公有可写变量或 setter。所有内部修改仅由 `QueryIndexOwner.initialize/apply_delta/reset`、`IncrementalSurfaceProjection.update_columns` 等受 store 调用的方法完成；消费者只能读取不可变快照或独立副本。

实例查询定义 live occupied 的现有 sensor instance，或已有可靠 box identity 的几何成员；不得用账本序号冒充永久箱 ID。缺失身份测试必须返回 `INSTANCE_BINDING_MISSING`，不以额外观测补齐。

## 4. 执行顺序与收口责任

同一 owner 负责实现、测试、修复、独立集成和提交，不另等待 test agent：

| 阶段 | 交付 | 不能越过的门 |
|---|---|---|
| S0 | 冻结 352191b3 oracle、写路径清单、基准输入和耗时 | 基准失败 ID/dirty/环境记录完整 |
| S1 | stats/frontier 增量缓存及旧 get_stats 适配 | 语义全等、查询/heartbeat 门；独立提交先回报 |
| S2 | block/实例/分页及不可变查询快照 | 删除/恢复/旧游标/并发完整性 |
| S3 | dirty-column 高程和有界 surface patch | 与原全量投影逐字段全等 |
| S4 | 精确体素 AABB 最近距离查询 | 独立暴力 oracle、unknown/radius/删除测试 |
| S5 | typed 服务、独立安装、无运动压力和 placement 集成 | 下述全部门；各项缺失不得报总 PASS |

公共 DTO 在 S1/S2 冻结后不得原位改义；必要变更显式升 schema。S1 可单独通过并交付，但不能代替六项总验收。不部署回 Claude 分支；交付集成建议与各阶段 hash，由用户决定合入。

## 5. 正确性与不变性验收

沿用 G1 §9 全矩阵：seed 0..19 × 每 seed 500 事务，三分辨率 0.01/0.02/0.05、倒角/cuboid、0/5/20 实际箱；小图每事务、大图边界和结束全量对照。输入生成器及实际箱数必须落盘，不静默截断。

1. 主 OCC state/logodds/source/labels/instance/lock/shadow、revision、旧 content_digest、接受/拒绝码逐事务完全一致；数值 NaN 按同位置同语义比较。主计算没有授权数值误差。
2. stats/frontier 集合/顺序/计数完全一致；高程数值误差最多 1e-12，状态、标签、tie break、门判定及旧序列化/digest 完全一致，否则 FAIL。
3. 全更新路径覆盖：重复操作、拒绝、重叠提交、标签变但 state 不变、sensor shadow 更新后 remove、reset/preserve、pending/confirmed。索引不改变事实提交结果；索引异常必须具名，不能发布混版。
4. 每图 1000 固定查询点对独立体素 AABB 暴力距离，误差 <=1e-9m；删除最近障碍、unknown、空图、边界、截断、同距 tie 全覆盖。非 exact 结果不可声明安全。
5. 两读者一写者 1000 事务 ×3：结果各属一个快照；导出篡改不影响内部；旧游标失效具名；分页拼接等于全量 oracle，无漏/重。
6. 工作量门：单格 state 变更 frontier 重评 <=7 格；热 stats 扫描 0 个体素；仅重算实际 dirty 列；实例查询不扫描不含该实例的块。初始化/reset 不套增量门，单独计时。
7. 352191b3 原有 mapper/store/projection/commit/shadow/revision 及 placement structural/Attachment/session 测试全部回归。失败在同环境基准复验，新增失败 ID 必须为 0；新合同测试不得 skip。既有 skip 必须逐项列原因及未覆盖能力。
8. 同冻结 placement 请求，新旧合法候选、顺序、几何/地图身份、termination_reason 一致；仅时长诊断可变。不得因缓存改变预算起点或重置续取预算。

## 6. 性能、负载和可靠性验收

继承 G1 §10 完整矩阵及 §11 精确命令。环境：同工作站、ROS Humble、headless，无 Gazebo/MoveIt；记录 CPU/RAM/线程/依赖/affinity。每次计分产品源码 clean，固定代码/输入/配置/独立安装 hash；基准与新版顺序交替配对，不混用共享 install。先 cold 筛查，超 max 即 FAIL并停止该组；其余每组3预热+30次，nearest-rank P95=第29/30；不足30为 not_evaluated，不外推。

| 每组门 | P95 | max |
|---|---:|---:|
| ROS stats/summary（含序列化） | 50ms | 200ms |
| frontier 总数纯查询 | 1ms | 5ms |
| ROI/frontier/instance 页，最多4096，ROS往返 | 100ms | 250ms |
| 最近障碍单点 / 256点，radius=0.30m | 5/50ms | 20/200ms |
| 完整 surface 导出含序列化 | 100ms | 250ms |
| snapshot 获取 | 100ms | 250ms |
| ComputePlacement 全请求，结构命中/强制回退分别 | 5s | 20s |

初始化/reset <=30s；mapper peak RSS <=2GiB，派生索引/保留快照 <=512MiB。旧快照最多2代、token最多5s；超限具名拒绝，不能靠丢结果满足时限。G1 查询负载中的无匹配/密集匹配都测。

写入**仅作回归**：同 G1 的 30k/100k 点、外部比例0.5/0.9、跨边界射线比例0/0.5、三分辨率与0/5/20箱矩阵。统计更新+索引+投影+hash+发布准备整体；新版 P95/基准 P95 <=1.10，max <=基准 max×1.20+20ms，额外索引成本单列。本条替代 G1 的 integrate 1s/2s及 add/remove 500ms/1s强制绝对门，避免为达到写性能而越界改 OCC 算法；原绝对值仍作为信息性目标报告，不拿未达标写入能力冒称节拍已满足。

并发门保持严格：每0/5/20箱、1cm、120s墙钟×3，2Hz integrate(30k)、10Hz summary、5Hz ROI、2Hz 256点距离、1Hz surface，原0.25s heartbeat和2s lease不改。接收间隔 max<1s，零 lease 过期，accepted请求全部完成，无静默丢弃，积压<=1s；延迟仍满足上表。同一负载基准也测，保存 callback/锁等待时间线。若现有写入能力无法承载，本门 FAIL/blocked，不降低频率、不修改融合或 worker。

本门要求固定输入下每单位查询结果正确率100%、超 max 次数0；不是统计意义上的长期可靠性保证。30样本只足以计算登记的经验P95，不声称99.9%成功率；所有负载组分别通过才总PASS。120s×3是3个独立复位压力窗全通过（AND），只证明所测窗口内零心跳失效；任一失败即FAIL，不通过补跑删除失败。修复须新hash新一代测量目录。

## 7. 命令、证据和停止规则

使用 G1 §11 的 pytest及 `occ_query_eval.py oracle/benchmark/concurrency/planning/replay` 命令；实现 CLI 后必须 `--help` 与参数矩阵行为测试。concurrency实际覆盖三箱数组，planning实际覆盖三分辨率×三箱数组×命中/回退。新增脚本可提供 `--baseline-revision 352191b3` 和独立 baseline worker，不能在同进程混装不同版本模块。

EVIDENCE=`docs/status/evidence/occ_owner_b1/occ_query_index_g2_20261025/`。沿用 G1 §12 的 T0/T1/T2 边界矩阵、失败重放、10s前/5s后心跳窗口、每轮8GiB/总32GiB。T0必须含基准/实现hash、输入hash、dirty列表、加载路径、采样数、各组结果；测试/计时原始 stdout、逐请求JSONL、oracle首差异、资源/队列指标齐全。T1开关配对30次P95比<=1.10；超限先修取证，不用扰动数据认证性能。

测试前验证：只创建本任务ROS节点、无goto/FJT/vacuum实体副作用客户端。允许无运动ROS服务，不允许为了查询测试启动 Gazebo、MoveIt或完整装箱。启动预检可在30min墙钟内修复本任务脚手架；超限具名停止，产品失败不可归为预检以抹掉。

遇到白名单外修复需求、主 OCC oracle差异、未知写入者、资源超限或证据不完整，保存失败再停止对应阶段；未受影响的离线工作可继续。不得修改验收阈值、删失败、退回粗分辨率或只报告纯函数最快数字。

## 8. 交付与派发合同

任务名 `QUERY-ONLY-INTEGRATION`，owner `eng/codex/gpt-6`，一个实施 owner负责S0-S5完整验收，不再自动派生多条并行修改同一 mapper/node 的任务。先 `agent_start.sh` 领取主工作区discuss线程；每阶段独立提交，最终 `agent_complete.sh` 带真实 pass/blocked、实现hash、命令和证据路径。

最终报告逐项列六类查询完成情况、旧语义不变审计、每组P95/max/RSS、写入回归、并发heartbeat和placement全请求结果；任何强制门未评估不能总PASS。查询计划不承诺每箱时间或n=3成功，后者另行授权。只交付分支，不自行合入生产或改共享安装。
