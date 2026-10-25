# OCC 增量存储与查询索引 G1：六类查询及规划节拍验收

- parent: OCC-QUERY-INDEX-20261025
- generation: 1
- status: planned
- authorization: 用户要求详细执行计划；本轮仅文档，不派发实现、不启动ROS/Gazebo/运动。
- base_revision: 352191b3e73f400e17c2682f0ff296c47a1757d3
- workspace: /tmp/elfin-master-pack-baseline-g3
- coordination_root: /home/adamliao/work/elfin_humble_ws/docs/agents
- implementation_owner: eng/codex/gpt-6
- integration_owner: eng/codex/gpt-6
- dispatch_ready: no

## 1. 目标、已知事实与非目标

目标：现有1cm稠密OCC保持占用/证据/快照语义，提供无需请求时全图扫描的frontier、实例、局部区域、最近障碍、统计与高程查询；负载下不饿死0.25s心跳；最终证明整个ComputePlacement仍满足p95<=5s、max<=20s，而非仅证明单个缓存读取快。

已读基座源码：`CargoVolumeMapper`的一维x-fastest状态/source/active为bytearray，labels/instance为array('q')，sensor shadow为独立数组，LogOddsGrid主values/geometry_locked仍为Python list。`stats()`会生成全量frontier列表后取len；surface/counts有NumPy全量投影；store在事务末构建不可变2.5D快照。`_placed_boxes`与体素instance ID不是同一种身份，不能未经绑定就视作等同。

当前get_stats慢路径已在源码定位，但“为何心跳没执行”需同时保留executor/callback/锁/接收时序证据，不写成连续独占GIL8秒的定论。工作区已有他人 `admission_lease_probe.py` 未提交，禁止覆盖或纳入本任务。

非目标：迁移OctoMap、重写感知/Attachment、改变1cm分辨率或阈值、放松lease/碰撞/支撑门、接实机SDK、在线EMS/F/P、重复整场装箱诊断。已有30s stats等待仅为过渡，不作为性能修复；验收必须在原5s调用deadline内完成。

## 2. 总体结构：一个所有者，多种只读索引

```text
OccupancyMapStore.apply_update(update)   唯一外部写入口
  -> mapper批量变更记录（old/final cell evidence）
  -> 增量计数、实例索引、frontier、block摘要、dirty-column投影
  -> 构建同版本QuerySnapshot + 既有MapSnapshot
  -> 校验完整性并原子发布 SnapshotBundle

MapQueryService -> store.acquire_query_snapshot(expected_identity)
                -> QuerySnapshot只读查询 / 有界分页
```

不新建第二个地图manager或可写事实库。map和query投影来自同一次事务，planner不能向索引写“预测箱子”。索引不能成为新观测来源。异步诊断可以读取旧的已钉住快照；要求当前地图的规划调用不允许拿旧索引搭新surface。

所有派生索引有明确生命周期：初始化全构建一次；普通更新只改相关范围；reset/new epoch失效旧token；相同operation_id精确重试不重复更改统计或索引。拒绝输入不发布部分结果。真实释放pending事件保留既有迟事实语义，不能因索引构建失败撤销已发生的物理事实；索引失败时标QUERY_INDEX_UNAVAILABLE并闭锁依赖查询，原事实和证据保留。

## 3. 类边界、写权限与私有变量

新增纯模块建议如下，保持node薄层：

| 模块/Class | 私有可变状态 | 唯一更新方法 | 公开只读能力 |
|---|---|---|---|
| `cargo_voxel_delta.py / VoxelDeltaBuilder` | 本事务第一次old/最终new、dirty列/块 | `record_before/record_after/finalize`，仅mapper内部调用 | 冻结DeltaBatch |
| `cargo_query_indexes.py / QueryIndexOwner` | 计数、frontier bitset、实例分块成员、block摘要 | `initialize/apply_delta/reset`，仅store调用 | `freeze()`产生查询快照 |
| `cargo_surface_projection.py / IncrementalSurfaceProjection` | 各列统计、当前二维tile | `initialize/update_columns`，仅索引owner调用 | 不可变SurfaceProjection |
| `cargo_map_query.py / MapQuerySnapshot` | 无公有可写状态 | 仅构造；不提供setter | 各查询函数、独立导出副本 |
| `cargo_distance_query.py / NearestObstacleQuery` | 单次调用的优先队列 | 私有局部操作 | 精确有界最近障碍查询 |
| `cargo_map_query_adapter.py` | 有界session/token表 | `open/next/close/expire` | ROS typed请求响应适配 |

DTO公开字段只读、深度冻结；不暴露可重新开启writeable的内部数组，不把mutable mapper引用交给消费者。构造器验证形状、有限值、坐标系、身份。诊断计时/队列计数不进入地图内容digest。所有消费者只经store/query方法，不直接访问 `_grid/_source/_labels`。

## 4. 变更收集与紧凑存储

审计并覆盖所有写路径：sensor hit/miss、integrate_points、depth通道（若仍PARKED则不能借此重启）、mark_placed_box、unmark及remaining重栅格、sensor shadow恢复、reset/preserve_placed、显式fill free、来源/标签改变、确认/pending转换。

- Delta记录每个触及体素在本事务前与事务结束时的state/source/label/instance/logodds/lock及shadow变化。净变化去重；不能只记state改变，否则标签/顶部来源/shadow撤销会漏。
- 允许不同索引用字段变化掩码选择更新；frontier只关心state/active，surface还关心top source/labels等，实例关心owner变化。确认态变化可以没有dirty cells，但bundle身份照常推进。
- 先保持数值精度：主logodds迁为float64连续数组、lock/seen为uint8或bitset；不得顺手降float32以换内存。主/影子证据不得合并或删除。
- 全量reset/大量变化显式走批量初始化/重建，不伪装O(1)。需要分配/构建新代期间状态为INITIALIZING；启动准入不把半地图当ready。
- 用块内紧凑数组/bitset表示changes，不为420万格各建Python对象/tuple。block edge冻结16 cells；边缘块按真实尺寸裁切，无active格的块可跳过。
- 若变更处理异常，旧快照仍不可变；系统必须明确拒绝新查询或返回旧epoch钉住的只读结果，不能将部分新数组冒称当前提交。

## 5. 六项操作的精确方案

### Q1 frontier数量与位置

保持原谓词：active且UNKNOWN，六邻域中至少一个FREE或OCCUPIED。基座对neighbor active没有额外检查；等价测试先钉住这点，若要收紧另提语义修订，不能在优化里暗改。

维护frontier位图、总数、逐块数量。对state/active变化集合D，只重评D及六邻域，去重后以成员旧/新差更新计数。初始化/reset才全构建。位置查询按区域block筛选后读取成员，默认stable flat-index升序分页，不先生成全地图点列表。`frontier_count`总量O(1)，区域计数用完整块摘要+边缘块扫描。

### Q2 实例反向索引

显式区分 `instance_kind=sensor_instance` 与 `committed_box`。sensor实例查询对应当前live occupied且instance_id匹配的格；shadow另有证据域，默认不返回。已提交箱体按实际几何栅格成员，独立记录box_id，不覆盖sensor_instance字段。

索引结构 instance_key -> block_id -> 位图/有序局部索引；返回O(结果规模)，不全图过滤。unknown instance0不是“所有无主格”的快捷查询。传感器ID须带producer/session命名空间，或标legacy未绑定，不能跨epoch混同。

旧ledger缺少稳定ID：store提交时绑定已有真实box_id/验证记录；历史记录只能生成显式epoch-local legacy handle并声明无跨运行稳定性，不从列表序号伪造永久ID。不改旧提交接受条件；缺身份时按legacy语义可存，但按新box_id查询返回INSTANCE_BINDING_MISSING。

多物体栅格重叠允许多成员，不用单个instance整数覆盖全部归属。remove一箱只撤它的成员，保留其他锁/最新sensor shadow；完全复现既有重栅格结果。

### Q3 最近障碍与距离：先索引查询，不强制全图ESDF

实现精确block branch-and-bound：按点到候选block AABB的距离下界入优先队列；跳过对应状态计数为0的块；块内批量计算点到实际1cm体素立方体AABB距离。找到best后，只有所有未访问块下界>=best才宣称exact；相同距离以flat index稳定择一。

这是点到占用体素体积的距离，不是点到体素中心。分别返回 occupied最近距离、unknown最近距离（若请求）；不把unknown当free。outside hull/outside map具名返回，不用空索引解释成安全。该距离不包含工具/箱体尺寸，不替代完整包络、支撑或容器hull检查。

`max_radius_m`默认0.30，范围0..地图对角线；点批量<=256。半径内无匹配返回RADIUS_CLEAR和已证明下界，不返回伪造精确距离或NaN；全域无匹配返回NO_MATCH。达到work/deadline上限返回QUERY_BUDGET_EXHAUSTED、exact=false，不作为碰撞放行依据。

障碍删除不靠“只改六邻域距离”：查询总读当前不可变block索引，因此不保留错误的远处nearest parent。只有实际高频密集查询证明需要时才另开增量EDT/ESDF后端，保持同一接口和未知策略；本代不强制承担全距离场维护成本。

### Q4 分块区域查询与跳过无关空间

每block保存active/unknown/free/occupied/frontier计数、出现的label/instance键或精确索引引用。查询先做region与block交集，状态/instance过滤，最后在边界块精确裁切；不能用block摘要直接把整个块判为匹配。

region为container坐标轴对齐AABB，半开区间；返回与region有正体积交集的active cell，边界规则与几何栅格化一致。点查询单独按containment索引，不混用交集。未知/不存在block必须明确保持unknown，不能因稀疏索引未存就当free。

### Q5 高频统计

维护total/active/inactive/unknown/free/occupied计数、semantic occupied直方图、placed_count及frontier总数。事务按最终净delta加减；total及inactive仅初始化/几何实例变化改变。volume=count*r³，ratio分母沿现有stats合同原样保留，零体积定义不改。

`get_stats`读取已发布summary，不调用投影、frontier、np.unique或全数组扫描。legacy响应仍输出相同字段；新query接口补身份/confirmed/index_status。readiness调用不附带重型拓扑数据；不能在summary读取时“顺便补齐索引”。

### Q6 高频高程

每事务收集dirty XY列，仅扫描这些列的nz体素计算原规则：最高occupied、unknown/known比例、top source、semantic label及底向上tie break。单列O(nz)可先接受，不必维护复杂高度树；最坏全图变化仍需全投影且显式计时。

每个二维tile按copy-on-write更新，未变tile复用不可变数据；完整发布29k格仍有O(nx*ny)序列化/旧digest计算，不能宣称总导出O(changes)。提供surface patch读取仅用于明确增量消费者；既有全量snapshot输出保持字节/数值语义。只减少3D重复扫描，不改变height量化或unknown规则。

## 6. 公共Python查询接口

```python
store.acquire_query_snapshot(expected_identity) -> MapQuerySnapshot
snapshot.summary() -> MapSummary
snapshot.get_cell(CellKey) -> CellRecord
snapshot.query_cells(region, states, instance_filter, page) -> CellPage
snapshot.query_frontiers(region, page) -> CellPage
snapshot.count_frontiers(region=None) -> CountResult
snapshot.query_instance(instance_key, region=None, page=...) -> CellPage
snapshot.nearest_obstacles(points, max_radius_m, unknown_policy) -> DistanceBatch
snapshot.surface(region=None) -> SurfaceProjection
snapshot.surface_changes(since_identity) -> SurfacePatchResult
```

`CellKey=(map_epoch, ix, iy, iz)`；resolution/grid shape/frame/geometry绑定快照，reset后旧key拒绝。所有API默认container_link；world到container转换由边界adapter使用同版本TF，核心不查询latest TF。

共同响应：status/reason、snapshot_identity、voxel_identity、index_version、frame_id、grid origin/resolution/shape、complete、next_cursor、returned_count、examined_cells/blocks、elapsed_ns。CellRecord按请求字段返回state/source/label/instance/geometry_membership，禁止默认导出所有shadow。`limit`默认1024、最大4096；无limit全量导出不是在线接口。

游标绑定query参数hash和snapshot身份，重试相同cursor返回相同页，地图推进不会偷偷换数据；新请求指定CURRENT则取得当前已发布bundle。最多保留2代query快照且总索引/快照预算<=512MiB（不含唯一live voxel存储）；不能淘汰仍被单次查询使用的对象，超额拒绝新session，旧token过期返回SNAPSHOT_EXPIRED。最长期限5s墙钟，不自动续期。长时间全量离线导出另走显式durable snapshot，不用规划服务分页装成无限buffer。

新ROS只读服务以typed字段实现，不使用任意JSON命令总线：`GetCargoMapSummaryV2`、`QueryCargoCellsV2`（状态/实例/frontier枚举筛选）、`QueryCargoDistanceV2`、`GetCargoSurfacePatchV2`。共同字段包含request_id/expected_identity/region/limit/cursor；数组定义与上面DTO同构。旧get_stats适配summary，不增加写接口。source/instance枚举必须显式，不允许自由字符串绕开字段校验。

错误码至少：SNAPSHOT_IDENTITY_MISMATCH、SNAPSHOT_EXPIRED、QUERY_INDEX_UNAVAILABLE、INVALID_REGION、INVALID_FILTER、INVALID_CURSOR、INSTANCE_BINDING_MISSING、OUTSIDE_MAP、OUTSIDE_HULL、QUERY_BUDGET_EXHAUSTED、RESOURCE_LIMIT。分页complete=false与索引未就绪不同；不能通过空列表混淆未知/失败/确实无匹配。

## 7. 身份、原子性与并发

现有content_digest主要描述2.5D投影/ledger/确认态，不能假设它完整覆盖每个3D格。保留旧digest不改协议；新增 `voxel_digest` 覆盖grid布局、active、状态/source/label/instance/锁/有效证据与实例绑定。采用按block的版本化canonical hash，再组合block树root，排序规则固定，不能Python hash；传感器shadow若作为公开证据域读取也纳入对应digest。

QuerySnapshot绑定 `(epoch,revision,content_digest,voxel_digest,index_schema_version)`。各索引来自同一bundle；索引字段不反向改变原地图语义。旧日志digest保持原义，新字段不能冒称旧接口已有覆盖。snapshot发布前dirty block/column摘要必须完成；未完成返回未就绪，不把旧统计贴新revision。

copy-on-write chunk使读者在短锁内取得bundle引用，随后无锁读旧不可变版本；writer串行更新并原子交换发布引用。不能为了快而把mutable mapper数组给snapshot只读视图。事务/索引复制及哈希成本列入更新总时长和RSS。

查询与heartbeat不共享会执行重计算的互斥回调组；普通状态query不得等待整笔map计算锁。对长Python循环改批量native/NumPy，不以“换MultiThreadedExecutor”作为已解决证明。无运动集成若heartbeat门仍失败，必须在此阶段实现计算worker隔离或分段调度，并验证同身份原子发布；不修改lease 2s或心跳0.25s。具体worker实现由实测调度证据选择，但未满足心跳门不得收口或运动。

## 8. 优先序与可独立交付阶段

| 阶段 | 内容 | 依赖/通过条件 |
|---|---|---|
| S0 BASELINE | 固定真实更新/查询输入、原stats慢路径、线程/回调时间线、mutation路径清单 | 零运动；保存基准与oracle，不启动整场装箱 |
| S1 FAST-STATUS | delta写入、增量stats/frontier、summary读取、心跳不阻塞 | 先解决当前lease触发候选；不等距离/实例全部完成才交付 |
| S2 REGION-INSTANCE | block索引、实例域绑定与分页、快照原子性 | S1；所有撤销/恢复语义等价 |
| S3 SURFACE | dirty列投影、COW tiles、现有全量发布/新patch接口 | S2；full vs incremental oracle逐字段一致 |
| S4 DISTANCE | block精确最近障碍/unknown距离与有界状态 | S2；独立暴力oracle通过，不需要ESDF |
| S5 QUERY-INTEGRATION | typed服务、客户端使用summary/region、安装身份、无运动并发性能与planning预算 | S1-S4；所有性能/心跳门与取证健康通过 |
| S6 PRODUCT | 既有n=3闭环和后续容量门 | S5后另行授权，当前不启动 |

S3/S4可以独立worktree并行，但公共DTO由S2 owner一次提交冻结；node/srv/集成文件归S5单一owner。每阶段实际派发时填具体agent/model、exact plan_revision、base与依赖commit；不预填未产生hash。

## 9. 正确性验收：全量oracle而不是源代码字符串检查

固定随机seed0..19，每seed500笔事务，按序覆盖hit/miss、重复op、标签/实例切换、确认/pending、重叠箱提交、按不同次序remove、shadow更新后remove、reset/preserve、非法payload、stale identity。每笔更新后小图对全量reference比较state/logodds/source/labels/instance/shadow、stats、frontier集合、surface各字段、digest确定性。大图在各边界事件及结尾做全量对照。

- grid分辨率0.01/0.02/0.05，部署倒角和cuboid；block边界15/16/17格、地图边界、inactive洞、0/5/20真实箱且actual==requested。
- 小图统计/集合/整数完全相等；float64保持原运算序可比处逐值一致，projection最多1e-12数值误差且所有阈值判定一致。semantic tie break必须原样。
- 重复update不重复增量；状态不变但来源/实例改变可见；撤销几何后与无几何的sensor控制链等价；拒绝未改可见bundle。reset旧token明确失效。
- distance用所有目标体素AABB暴力最小值oracle，seed固定、每图1000点；距差<=1e-9m，tie稳定；测试唯一最近障碍删除、远处次近障碍、unknown策略、边界、空图、radius截断与预算耗尽。
- region/instance/frontier分页拼接与全量oracle集合及stable顺序一致，重复页相同；不把partial当complete。游标篡改/错identity具名拒。
- 并发两个读者+一个writer、1000事务×3重复：每个结果可对应且仅对应一个bundle，不混版；导出篡改不可达内部；snapshot读时不扫描3D图。
- 使用工作计数验增量性：修改单格后frontier评估<=7个唯一位置；surface重算列数==受相关字段影响的唯一dirty列；热summary读取visited_voxels=0；实例query不得访问不含该实例的block。初始化不套此增量门。

## 10. 工作负载与性能门

这是拟定工程门槛，不是测得能力。环境固定同一工作站/ROS Humble、headless、无Gazebo；记录CPU/RAM/依赖/线程/affinity，产品源先冻结commit，测试时dirty产品文件=0。主部署1cm必须通过；2/5cm作为回归，不拿较粗图替代。

**更新负载矩阵**：0/5/20真实箱；30k/100k传感器点，外部比例0.5/0.9，跨边界射线比例0/0.5；部署分辨率三值。保留原始射线端点、原采样预算和顺序；几何提交/删除另测0.35×0.53×0.22m及真实Attachment相关档位。单次cold筛查超max则停止该组重复，max=FAIL、p95=not_evaluated。通过筛查的每组3预热+30独立重置到同起态测量；只测warming不改变负载。

**查询负载**：10k固定seed点读；完整stats；frontier首4096个；0.3m立方ROI cells；0/5/20账本各有效实例首4096个；距离单点与256点批（radius0.30m）；完整29k高程导出与100列patch。无匹配和大量匹配分别测，输出规模不可隐减。

| 指标（各组独立，nearest-rank p95=第29/30） | p95 | max |
|---|---:|---:|
| ROS get_stats/summary，含序列化 | 50ms | 200ms |
| frontier count纯读取 | 1ms | 5ms |
| ROI/frontier/instance一页<=4096，ROS往返 | 100ms | 250ms |
| 最近障碍单点 / 256点批，纯查询 | 5ms / 50ms | 20ms / 200ms |
| 热完整surface导出，含序列化 | 100ms | 250ms |
| 100k点 integrate + 索引 + snapshot/hash + 发布准备 | 1s | 2s |
| 单箱add/remove + 索引/高程/摘要 | 500ms | 1s |
| 全请求ComputePlacement（结构命中/强制回退首批分别） | 5s | 20s |

构建/reset完整索引<=30s；mapper总peak RSS<=2GiB，派生索引/历史快照合计<=512MiB。大体积export不是轻量query，不能占住stats executor。实际near query若大量同距/半径内候选导致预算耗尽，负载门计失败而非以具名拒绝算性能通过。

**并发节拍门**：每个0/5/20箱场景跑120s墙钟，3次：2Hz integrate（固定30k点），10Hz summary，5Hz region，2Hz distance256，1Hz完整surface，持续0.25s runtime heartbeat。2Hz更新如处理不过来，bounded queue拒绝/背压必须记录；本验收负载要求accepted全部完成、无静默丢弃、不积压超过1s。heartbeat接收间隔max<1s、零lease过期；真实运行身份/接口一致。不是只测timer发送频率。

**planning端到端门**：服务真实调用使用同冻结snapshot 0/5/20×三分辨率、结构命中与强制回退各3预热+30次，含snapshot读取/身份验证/atlas/序列化，不含运动；snapshot获取p95<=100ms/max250ms，全请求仍<=5s/20s。预算续取不重置，pending/stale明确拒绝。不因query变快就推断轨迹规划、每箱周期或容量已通过。

所有门为固定负载确定性验收：要求每组通过，不统计人群成功率。样本不足为not_evaluated；失败必须保留输入，不自动降频/延长lease/减少候选。capture开关配对30次p95比<=1.10；超限先修capture再验收。

## 11. 测试与脚本交付入口

新增 `scripts/g3/occ_query_eval.py`（子命令为本计划交付要求，当前未实现），focused tests如下；旧mapper/store/projection/commit/shadow/revision tests全回归，完整实际命令清单入报告。

```bash
set -o pipefail
python3 -m pytest -q src/luggage_perception/test/test_cargo_voxel_delta.py src/luggage_perception/test/test_cargo_query_indexes.py src/luggage_perception/test/test_cargo_surface_projection.py src/luggage_perception/test/test_cargo_distance_query.py src/luggage_perception/test/test_cargo_map_query_contracts.py
python3 scripts/g3/occ_query_eval.py oracle --seed-start 0 --seeds 20 --transactions 500 --out "$EVIDENCE/oracle"
python3 scripts/g3/occ_query_eval.py benchmark --resolutions 0.01,0.02,0.05 --boxes 0,5,20 --warmup 3 --repeat 30 --screen-first --out "$EVIDENCE/benchmark"
python3 scripts/g3/occ_query_eval.py concurrency --duration-wall 120 --repeat 3 --out "$EVIDENCE/concurrency"
python3 scripts/g3/occ_query_eval.py planning --warmup 3 --repeat 30 --out "$EVIDENCE/planning"
python3 scripts/g3/occ_query_eval.py replay --bundle "$FAILED_CASE"
```

服务/并发/planning子命令会启无运动ROS节点，须在对应实施任务明确授权后执行；当前文档轮不启动。任何goto/trajectory/vacuum副作用客户端不在该runner内，行为测试钉死零运动请求。接口包改动用独立build/install完整重建依赖并记录运行时模块与.so路径/hash，不混用别人的overlay。

## 12. 证据矩阵与停止规则

EVIDENCE=`docs/status/evidence/occ_owner_b1/occ_query_index_g1_20261025/`；每组目录新建，原始失败不覆盖。

| 边界 | T1 | 失败触发与T2 | 窗口/预算 | 重放 |
|---|---|---|---|---|
| update→索引 | op/request/identity、dirty cells/columns/blocks、分段wall/CPU | oracle不等、超时：事务前durable checkpoint+原update+delta摘要+输出 | 当前失败事务及之前重建所需日志；2GiB/例 | oracle/replay |
| query→响应 | query字段、identity/token、examined/returned、latency、complete | stale混版/错误结果/超时：snapshot引用、完整query/response | 单次query，128MiB/例 | replay |
| runtime heartbeat | 源/接收单调时间、sequence、incarnation、首拒绝reason | 间隔>=1s或lease拒：callback入/出、锁等待、线程采样及原输入 | 前10s后5s有界环，256MiB | inspect/replay |
| snapshot→placement | 图/包络/策略身份、分段耗时、候选计数 | 超门/假合法：同快照、请求响应、候选/门结果 | 单请求，512MiB | planning replay |

checkpoint内容寻址去重，单轮8GiB/总保留32GiB，达到容量在下一组前停止并记录resource_censored，不删失败以凑门。采集健康门实际写入/读回/身份join通过才计分。数据持久化后仅清理本任务进程，不碰并行仿真。量测阶段不修改被认证产品，修复后新hash新目录重新登记，不盲补跑。

## 13. 交付与派发

完成交付包含API/类型说明、唯一写入口审计、oracle报告、每负载完整耗时与工作计数、RSS、实际心跳接收间隔、安装身份与planning整请求门。任何六项尚未通过单列，不能以“已实现全部接口”收口pass。S1可先独立交付解除轻量状态阻断，S2-S5继续不得阻塞其可用修复。

派发前用git rev-parse解析本计划产品基座与计划提交hash，不使用文档文本猜测hash；如集成head已前移，新增派发补充说明实际diff，不把另一agent的lease诊断或修复覆盖。当前所有stage dispatch_ready=no；具体任务必须绑定真实依赖结果后再创建邮箱项。
