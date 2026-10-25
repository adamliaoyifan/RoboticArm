# OCC 查询 G3 接手：并发 ROI 归因、局部修复及剩余验收

- parent: OCC-QUERY-INDEX-20261025
- subtask: QUERY-ONLY-INTEGRATION
- generation: 3
- owner: eng/claude/glm-5.3
- status: approved-for-dispatch
- dispatch_ready: yes
- predecessor_plan: b9f4b2dc / docs/plans/OCC_QUERY_ONLY_G2_20261025.md
- predecessor_thread: docs/agents/discuss/2026-10-25_1157_occ-query-only-g2.md
- implementation_base: 3621419c4e1f1e13cfa19e05980193df0813a6d9
- product_revision: afadb52591a713af0ca26cdce9a5ec995d701be0
- paired_baseline: b33dda919521ef043fa9e30b07dcc79c87f76fce
- authorization: 用户授权转交 Claude，实施查询侧修复、离线及无运动 ROS 验收；不授权 Gazebo/MoveIt栈/运动/生产合入。

## 1. 接手与范围冻结

本代因 owner 变更替代 G2，而非改写其 Claim 或失败历史。G2 的正确性与性能门、G1 六类接口及证据要求继续有效；本文补充实施细节和诊断次序，不降低任何门。原产品正确性180000条、placement18组612对、316回归通过只适用于原冻结版本，新版按影响范围重验。

Claude 领取新线程后从本计划提交建立独立 `agent/eng/occ-query-index-g3-claude`、`/tmp/elfin-occ-query-index-g3-claude`，独立 build/install/log。不在原 Codex 或生产工作区改文件、不推进 master/integration-prior-g1、不覆盖共享 install。

必须先完成交接静默屏障：原 owner 确认停止新增实验、记录后台 write_matrix_03 当前组及进程状态、在组边界停调度或有界终止并保存 interrupted；Claude 核验相关旧进程已退出后才开始任何计时/ROS工作。未获确认可读源码、整理计划、静态审查，不并行跑重型测试。原任务队列项在原 owner 确认前保留，最高代次规则禁止其再以G2完成通过。不得按泛进程名杀其他任务。

旧工作区三个未跟踪文件仅是未验证原型：`scripts/g3/occ_query_region_diagnostic.py`、`src/luggage_perception/test/occ_query_region_probe.py`、`test_occ_query_region_probe.py`。不自动复制进产品。接手时记录内容SHA/dirty清单、原作者确认后可作为参考；从冻结产品独立实现或经审查显式导入，不把原型当已测试成果。

允许优化：ROI/frontier/instance块选择、分页和DTO构造、查询注册表及派生索引维护、距离批查询、有界取证。禁止修改主OCC存储、融合/射线/阈值/分辨率/hull/裁剪/标签/shadow/占用/提交/幂等/身份含义；禁止规划算法、Attachment、支撑/碰撞门、lease、GC全局策略、线程池/执行器/回调组拓扑/写入worker重构。不得改变schema2 hash配方、查询预算、分页大小、采样率或丢弃负载求快。G2白名单仍适用，白名单外提案先上报。

## 2. 已知耗时与未证结论

| 位置 | 已知事实 | 必须继续分离 |
|---|---|---|
| ROI纯查询 `cargo_map_query.py:_query` | 无instance时range全部块，全部排序后才region过滤；按块累加后反复排序selected | 全块选择/排序、单元mask、prefix合并、CellRecord构造各占多少CPU/wall |
| 区域frontier `count_frontiers` | 同样遍历所有块，再判相交 | 能否复用同一精确block选择器而保持计数全等 |
| 注册表 `cargo_map_query_adapter.py:publish` | 与open/resolve共用_lock；锁内遍历最多两代全部块/layout/tile去重和资源计费 | 锁持有与ROI等锁的实际重合、对象析构/计费成本；不能引用“短锁”注释作为证明 |
| 服务 `cargo_map_query_service.py:_execute` | open→operation→meta→close，operation还包含typed响应构造 | acquisition、纯查询、DTO→ROS、close、callback前后残余分别多少 |
| 并发06 | ROI往返445.096ms，_execute416.260ms，写入在窗口内开始；后续ROImax577.810ms | 重叠不是因果证明；wall减threadCPU是离CPU时间，不等于全是GIL或锁 |
| 新索引写入 | 历史profile04有apply_delta63ms、freeze23ms、rehash16ms、通知14ms等诊断记录 | 旧版本数据不可当最新互斥分段账；要在afadb525同输入重新核平 |
| 距离尾延迟 | 1cm/256点五组P95 50.63--58.17ms；部分旧诊断确见约29ms gen2暂停 | 当前失败是否计算/构造/GC/排队，不能推广为全由GC引起 |
| 取证 | 198比较42超1.10；历史A/A也有误拒信号 | 区分取证成本、顺序/系统噪声；不挑on/off有利结果，不自行换指标 |

首份交付必须回答“消耗在哪”：按同请求列互斥阶段wall、thread CPU、锁wait/hold、访问块/单元、结果数、GC重叠、写入阶段重叠和未归属。不能仅再报总耗时或一句‘线程竞争’。

## 3. P0 证据复核与轻量插桩

读取主证据 `docs/status/evidence/occ_owner_b1/occ_query_index_g2_20261025/s5/REPORT.md` 的最新段、`concurrency_rehearsal_06`、`query_matrix_02`、`write_matrix_03`、`preflight/REGION_BLOCK_PRUNING_DIAG_PLAN.md`。复核原始monotonic时刻、source/install hash、病例ID和完整请求；不重放整场仿真。

新增诊断模式仅记录每请求/事务的定长边界，不逐体素日志：

1. 客户端scheduled、actual_send、response_received；服务callback_entry、execute_enter、open begin/end、operation begin/end、meta/close、callback_exit。无DDS接收时刻时把send→callback标‘transport+executor待分离’，不得冒称纯排队。
2. registry锁：attempt/acquired/release、thread_id、operation与事务ID；区分publish/open/resolve/close。采集包装必须保持原锁获取次数/顺序，异常路径也记录，不人为sleep。
3. query：候选块范围计算/排序、精确mask、prefix合并、CellRecord、typed构造，记录thread_time_ns与perf_counter_ns；工作计数包括**候选块数/实际考虑块数/过滤后块数**，不能仅报当前examined_blocks（其未包含被过滤掉的块）。
4. writer：原OCC计算与新增delta、frontier、blockfreeze、hash、surface、registry计费/发布分开。嵌套span只统计exclusive桶；先保存inclusive时间便于复查。
5. 可选GC begin/end及generation，native GC不改；采样OS线程调度只能诊断使用，cProfile不能认证性能。

跨进程仅同一主机monotonic且启动时记录时钟域；所有span按request/事务/identity连接。每请求互斥阶段和与被分解父span残差<=max(1ms,5%)；未满足只报残余，不硬分配给‘其他’作归因。锁等待span嵌于open时不重复加总；GC与阶段重叠单列，不能双计或从正式耗时扣除。

capture健康要求所有已完成请求唯一join、记录无丢弃、源版本清晰。用预分配buffer和后台有界写入；锁内不格式化/压缩/写磁盘。关闭诊断不增加逐体素开销。T3资料不用于正式过门。

## 4. P1 首要修复：精确ROI块裁剪

在 `cargo_map_query.py` 或聚焦的纯 helper 中实现 `intersecting_block_ids(grid, region)`，输入冻结grid/region，输出稳定块ID序列；region=None保持原全域路径。从规则block坐标直接定位候选范围，不遍历全部块再过滤，不新增另一可写地图。

具体要求：

- 用当前 block_bounds 同一坐标运算构造轴边界，按严格相交 `block_high>roi_low && block_low<roi_high` 二分上下界或数学范围加精确复核；不用随意epsilon修边界。范围夹到grid；零厚/非法region保留原拒绝语义，完全外部合法region保留原空结果语义。
- 数据布局、blockID计算、部分末块从当前实现核对，不能假设blockID等于最小flat顺序。可在不可变共享grid-layout缓存轴边界，必须资源计费，不放跨epoch全局cache。
- 与instance块集合取交，再按frontier/state摘要过滤；保留现有最终cell相交检查、active过滤、instance绑定、距离/未知规则。
- 保持全局flat-index顺序和原cursor配方。各block的flat索引跨行交错，不能简单按块拼接前limit条；优先先保留原selected逻辑，若测得为热点再做有证明的limit+1有序merge/heap；complete必须由第limit+1个证据判定。
- `count_frontiers(region)`复用相同selector；完整块摘要+边界格精查，region=None仍O(1)。

离线验收：独立全block AABB暴力oracle，shape 1/15/16/17边界与真实部署shape，三分辨率、非零origin、所有轴面及nextafter两侧、薄ROI/外部ROI/倒角inactive、1000 seeded随机ROI。全部选块集合相同；三箱数组每个cells/frontiers/instance入口，limit=1/17/1024/4096，多页拼接/错cursor/reset/同token更新并发全等，结果/identity/cursor逐字段一致（elapsed/work诊断除外）。

性能工作门：有界ROI不检查范围外block；考虑块数<=相交轴范围笛卡尔积（再减instance过滤），新增单次visited指标必须实际覆盖selector开销；固定ROI、增大地图远端只允许轴二分/固定维度成本，不出现全块O(B)扫描。对保存失败ROI做3预热+30交替配对纯查询，报告工作量及P95/max；这不是ROS过门，修复采用前还须后述并发对照。

## 5. P2 并发归因实验与条件修复

先读原证据、离线复现，再只运行以下**固定诊断矩阵**：1cm、倒角、原失败ROI输入（保存payload/首次phase），0箱和20箱，3预热+30次；每组原afadb525与改版交替，独立同输入初态，原线程/参数不变。诊断允许控制请求序列，不把控制序列说成原压力门。

| profile | 条件 | 分离的问题 |
|---|---|---|
| D0 | 只读pinned快照，单查询，含纯函数和ROS各一组 | 查询自身与序列化基线 |
| D1 | 独立reader/writer，原30k同序更新；ROI分布于write前/中/后 | 写入重叠是否增加wall、锁wait或CPU工作量 |
| D2 | 原完整频率诊断窗30s，固定各请求phase，含距离/surface | 加入其他读请求后的排队/GC/服务时间变化 |

D1不得在产品锁内加barrier改变调度；由fixture发起/observe阶段实现时序，并记录实际落点，未实际重叠的样本不能算重叠组。每类至少30有效样本；最多60尝试，不足标not_evaluated，不无限撞时序。D2短窗只诊断，不能代替120s正式门。总计时不得与旧write矩阵或其他本任务重算并跑，主机其他负载/affinity要记录。

按测量选择修复：

- 若selector/排序主导，先验P1收益，不顺手改registry。
- 若registry发布计费的锁wait证据明确，在唯一publisher纪律下将**纯派生资源计费**移到锁外，锁内校验generation版本后原子swap；如支持并行publish需CAS/retry/串行证明。旧代仍由reader持有，禁止原位变更；512MiB/两代/5s/token错误及资源失败语义不变。大对象回收可推迟到锁外但不得无限保留。不改事务锁或引入worker。用受控线程事件证明长计费不阻挡读已发布快照、并发publish不丢代、reset/expire/资源拒绝不混版。
- 若typed构造主导，减少重复对象构造/转换并证明跨响应无共享别名、CDR逐字段等价。不得复用可被下一请求覆盖的响应缓存。
- 若wall大而thread CPU小，结合锁/GC/OS调度证据细化；不能用wall-CPU独自锁定GIL。若主OCC计算或executor拓扑才是瓶颈，本代只提交证据和最小另单建议，保持blocked，禁止扩权。

成功归因必须有同输入、同负载、单项修改前后对照及工作量证据；只有同时发生记correlation。即使原因未唯一分离，性能失败也不能判PASS。

## 6. P3 剩余距离、写入和取证门

ROI优先，但本任务接手G2全部未过门，不以‘ROI修好’宣称总完成：

- 距离：重放5个失败组，拆pinned取得/筛块/堆/叶计算/CellRecord/typed/CDR/GC。允许减少同快照batch重复筛选、减少不必要临时对象；保留体素AABB精确距离、stable tie、未知、radius、预算、256点输出及资源计费。1000点/图独立oracle及删除最近障碍回归必须过。不修改全局GC策略或推迟GC到评分窗外。
- 写入：只优化新增派生维护，最新同输入核平delta/frontier/freeze/hash/surface/publish开销，记录dirty数量和bytes；不得以最后一次空refresh掩盖前次维护。允许按净变化去重、复用不可变未变块、精确增量计费；新旧hash/事实/旧接口必须逐字等价。原OCC融合/射线不优化。
- 取证：先以原无取证版本做A/A配对，固定顺序seed，5个独立owner，每个3预热+30样本；然后同法A/B。A/A若误拒，保留全部结果，不能断言42失败全为噪声。优先减少同步压缩/分配，用有界后台journal、保持完整lossless数据和health。每组原P95比<=1.10门不变；若门持续无法区分无观察者与有观察者，单列measurement_blocked向用户申请修订，不挑模式、不静默改阈值。既有绝对延迟失败独立保留。

## 7. 正式验收与运行次序

先P0归因报告、P1离线正确性、代表性失败组修复，再无运动D1/D2。代表组未过不自动耗时跑全矩阵。一个已冻结修复版本通过预检后才串行完整验收；中途不得修改产品，任何修复另立hash/目录。

| 门 | 输入/次数 | 通过阈值 |
|---|---|---|
| 主语义 | G2 seed20×500×18组=180000条；旧baseline b33 | 事实/旧身份/接受拒绝/旧读取全部一致；无新增假合法 |
| 原回归 | G2原316清单+所有新增query/registry/IDL测试 | 新增失败0；新合同无skip，存量逐项复验 |
| ROI/frontier/instance | 三分辨率×0/5/20×cuboid/chamfer；稀疏/密集页 | 3warm+30/mode，ROS P95<=100ms、max<=250ms；无预算拒绝冒充成功 |
| summary/frontier count | 同完整查询矩阵 | ROS50/200ms；纯count1/5ms（P95/max） |
| distance1/256 | 18组、radius.30m、原on/off模式 | P95<=5/50ms、max<=20/200ms；所有点完整正确 |
| surface/snapshot | 同矩阵 | P95<=100ms、max<=250ms |
| writes | 原72组，30k/100k、outside.5/.9、cross0/.5 | P95比b33<=1.10；max<=b33max×1.20+20ms；全部事实等价 |
| resources | 全矩阵 | init/reset<=30s，RSS<=2GiB，retained<=512MiB；不漏计共享/独有内存 |
| capture | 原198个逐组比较口径 | P95 on/off<=1.10，零丢/错join，不能用低样本替代 |
| placement | 原18组，结构命中/强制回退，612对完整响应 | 全字段一致，P95<=5s、max<=20s；预算和候选数不变 |
| concurrency | 1cm，0/5/20箱×120s墙钟×3 | 原2Hz30k写/10Hzsummary/5HzROI/2Hz256distance/1Hzsurface；原延迟门、backlog<=1s、全部accepted完成、零静默丢、heartbeat双向max<1s、零lease；独立语义replay全等 |

全部P95采用nearest-rank第29/30，cold max先筛；不足30 not_evaluated，不只统计成功样本。并发每个请求类型至少30完成样本、满120s，超限当次失败并保留后续有界诊断窗，不拿短窗过门。每单位正确率100%、超max/丢失/错误身份0；各组AND通过，只证明固定负载窗口，不宣称长期可靠率。baseline压力若失败照实保留，不要求本任务修baseline；candidate必须满足门，主写入承载不足需另单而非降低频率。

取证门未过时受扰动数据只作诊断。整体验收不过，保留原b33部署基准；不以placement快或正确性全等抵消性能失败。

## 8. 运行入口与证据

继承现有CLI，不造另一个基准框架。以下命令使用新worktree，先source ROS和独立候选IDL；`EVIDENCE`指主工作区G3新目录，`INSTALL`与`BASE_INSTALL`均为独立构建，`DOMAIN`选择未占用值。执行前记录实际值和命令。

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q src/luggage_perception/test/test_cargo_map_query_contracts.py
python3 scripts/g3/occ_query_eval.py oracle --seed-start 0 --seeds 20 --transactions 500 --baseline-revision b33dda91 --out "$EVIDENCE/oracle"
python3 scripts/g3/occ_query_eval.py benchmark --mode queries --install "$INSTALL" --domain "$DOMAIN" --samples 30 --warmup 3 --out "$EVIDENCE/queries"
python3 scripts/g3/occ_query_eval.py benchmark --mode writes --samples 30 --warmup 3 --baseline-revision b33dda91 --out "$EVIDENCE/writes"
python3 scripts/g3/occ_query_eval.py planning --install "$INSTALL" --baseline-install "$BASE_INSTALL" --domain "$DOMAIN" --samples 30 --warmup 3 --out "$EVIDENCE/planning"
python3 scripts/g3/occ_query_eval.py concurrency --install "$INSTALL" --baseline-install "$BASE_INSTALL" --domain "$DOMAIN" --duration-wall 120 --repeat 3 --boxes 0,5,20 --out "$EVIDENCE/concurrency"
python3 scripts/g3/occ_query_eval.py replay --bundle "$FAILED_BUNDLE" --out "$EVIDENCE/replay"
```

第一条不是全回归；S0必须从原316报告取真实完整文件清单并记录命令。P0新增诊断入口需在首次运行前落盘具体args、seeds、输入SHA、D0/D1/D2矩阵及停止条件，不临场改参数或脚本吞失败。诊断全矩阵不并行；低成本源码阅读不算计时竞争，任何测试重算重叠必须标记而非隐瞒。

新证据根：`docs/status/evidence/occ_owner_b1/occ_query_index_g3_20261025/`；旧G2证据只读。T0版本/输入/dirty/加载路径/环境，T1阶段统计，T2失败前10s后5s、原snapshot/请求响应/并发update序列、原始CDR、锁和callback时间线，沿G2每轮8GiB/总32GiB限额。健康核验及独立replay后再清理本任务进程。原型未验证、取证缺失、诊断扰动、部分窗口都显式标注。

交付顺序：HANDOFF.md（旧进程收口/版本）、ROI_CAUSE.md（分段账及证据强弱）、修复提交+测试、S5逐门表。首个归因报告在开始正式全矩阵前回帖，防止再次仅堆叠反复测量。任何需要修改禁区或验收规则的结论先停该路径上报，其他独立离线工作可继续。
