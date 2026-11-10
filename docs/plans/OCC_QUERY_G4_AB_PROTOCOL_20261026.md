# OCC G4-A / G4-B：独立只读查询协议与派发验收合同

- parent: OCC-QUERY-INDEX-20261025
- generation: 4
- status: approved-for-dispatch
- owner: eng/claude/glm-5.3
- code_base: 7863a840fada102af5b42e5c2f71eccb6c5a0f09
- paired_baseline: b33dda919521ef043fa9e30b07dcc79c87f76fce
- prior_plan: docs/plans/OCC_QUERY_ISOLATED_PROCESS_G4_20261026.md @ 7863a840
- prior_acceptance: 26339903 G3 / b9f4b2dc G2
- authorization: 用户要求补齐协议并继续实现；允许隔离查询进程、发布者、对应launch/无运动ROS与验收；禁止Gazebo/MoveIt栈/机器人运动/生产合入。

## 1. 优先级与边界

本补充覆盖rev2的未决/冲突条款，其他G2/G3强制门继续有效。G4-A可立即领取；G4-B仅登记依赖，A通过后绑定确切实现hash才能启用，绝不使用浮动HEAD开测。

OccupancyMapStore唯一事实写者；不改融合/射线/主数组/分辨率/裁剪/几何/占用/阈值/影子账/提交准入/幂等/规划/Attachment/lease或原heartbeat。授权新增后台导出与接收线程、独立ROS只读进程及生命周期接线；不重构OCC计算worker、不全局调GC或控制器执行器。

进程分离只移除同一解释器内的竞争渠道，不承诺OS调度/CPU资源完全解耦。wall-threadCPU只作诊断，5ms不再是硬门或GIL证明；正式并发绝对时限保留。这是按本次用户要求落实的合同修订，不回写G3失败结果。

`use_current`在隔离模式明确为“具有有效发布者存活证明、未落后于已获知head的本地最新完整代”，不是写端线性一致读。正常传播窗口允许有界滞后，响应仍回显精确身份；用于规划的消费者必须pin所依据的身份并保留执行准入复核。不得宣传客户端语义完全无变化。原进程内模式保留用于对照/回滚，单次launch二选一，默认不替生产启用隔离。

## 2. 部件、接口与权限

| 部件 | 输入/输出 | 可变状态与唯一修改入口 |
|---|---|---|
| QueryExportPublisher（writer进程，纯模块） | committed QuerySnapshotData+完整dirty集合 -> offer结果、immutable manifest/objects、head/control | 私有有界pending/staging；`offer_committed/start/stop`，store提交后调用；不得改mapper |
| QueryWireCodec | DTO/数组 -> 明确schema二进制块+manifest；反向校验解析 | 无跨请求可变状态；`encode/decode/validate`，禁pickle/任意代码反序列化 |
| QueryReplica（独立进程纯模块） | head/manifest/objects -> immutable published registry | 私有状态机/接收代/ACK；`poll/apply_complete/invalidate/close` |
| QueryMapReadFacade | expected identity/token -> snapshot/具名错误 | 实现MapQueryService所需只读方法；无mapper导入/写入口 |
| cargo_map_query_replica_node | 4个既有typed服务 -> 既有IDL响应 | 仅ROS薄适配和启动关闭；无store事实写入 |

公开字段为深度不可变DTO或独立副本，不暴露可写mmap/数组；私有状态只能经以上函数更新。publisher与replica的内部同步协议不对planner开放。现有 `/cargo_map/get_summary_v2`、`query_cells_v2`、`query_distance_v2`、`get_surface_patch_v2` 在隔离模式只由replica注册。mapper仍保留旧get_stats/观测/写服务及原runtime peer，不能误删所有旧读服务。readiness不把replica冒充mapper peer；replica另发可观测查询健康状态，不更改既有lease peer集合。

## 3. 传输：不可变文件对象与只读mmap

冻结选择同机私有代根+不可变内容寻址对象+只读mmap，默认独立tmpfs目录；可配置本地目录但正式成绩必须同一介质。不是任意远程FS协议，不使用DDS承载快照或把D555历史故障当本方案因果依据。mmap不保证端到端零拷贝：编码、校验、Python重建和页缺失成本都实测入账。

根布局：`owner.json`、原子替换的`head.json`（最新提交head+最后成功control时间+错误）、`published.json`（最新完整manifest引用）、`objects/<sha256>`、`generations/<incarnation>/<seq>/manifest.json`、受控`staging/`、`control/ack`与`control/resync`。根0700；路径仅受控相对名称，拒绝逃逸/符号链接/非法shape/dtype/长度，不允许载入object数组。仅发布者可写objects/generations/head；replica仅写自身ACK/resync，不修改payload。

wire_schema_version=1（独立于旧index schema2），每代manifest包含：producer_incarnation随机UUID、map_epoch、seq、kind(FULL/DELTA)、base完整5腿身份及base manifest hash、target完整身份、geometry_hash、grid、commit_monotonic_ns、每项类型/字节数/shape/dtype/sha256、删除项、layout引用、manifest自身规范化checksum。规范化JSON UTF-8、键排序、固定separators；浮点/NaN等主体数值用明确dtype二进制，不依赖JSON猜精度。payload单对象<=4MiB，较大对象分块；manifest<=8MiB；所有长度先验证上限再分配。

每份已发布manifest是**自包含的完整引用表**，DELTA只新写变化对象，未变对象引用已有内容地址；不复制未变块到每代目录。接收增量必须验证base和target，不因seq连续就接受错base；冷启/缺代可取带FULL标记的完整引用表，不需要遗失的旧目录链。FULL在正常导出通道有界生成，不从查询请求同步调用mapper。

导出覆盖QuerySnapshotData的全部公开语义：grid、blocks的evidence/active/frontier/flat/state_counts/frontier_count/membership/fact_digest及distance派生材料、sensor/box反向表、summary、完整map_snapshot、surface tiles及patch追溯所需数据、distance_index或确定性重建材料、身份元数据。若选择接收端重建派生字段，记录重建配方版本及耗时，并逐字段oracle验证。不得遗漏仅provenance变化。会话token不跨进程搬运；cursor配方不变，密钥在replica启动时新建、同一接收器会话内稳定。

变化集合来自既有dirty/frontier邻域/geometry变化，包含跨块frontier-only更新；`fact_digest`只保留原身份用途，**不得作为传输payload等价性的唯一依据**。另设wire payload checksum覆盖frontier等传输成员，不修改原content/voxel digest。零块但身份/确认态/stamp变化仍发布新代。所有FULL/DELTA必须与冻结源QuerySnapshotData全字段对照。

写完对象和manifest、校验长度/摘要后才原子rename `published.json`。本协议保证进程崩溃时不暴露半代，不声称tmpfs掉电持久化；重新启动必须从唯一owner重同步。普通文件模式的fsync策略固定并计时，不通过关闭持久化操作挑成绩。

## 4. 背压、文件生命周期与资源

store提交后只offer不可变引用和dirty元数据，成功/拒绝都不回滚已提交事实；export异常记录具名状态，绝不吞成成功发布。写端后台串行编码；待处理引用<=2代，staging<=1代，已发布可读代<=2；这些引用不等于允许额外无限内存，全部受统一预算。

队列满返回EXPORT_BACKLOG_FULL并设置查询导出不健康；不阻塞事实主链、不默默覆盖队列。记录未发布seq区间；恢复时显式RESYNC到最新完整代，旧缺失提交在新鲜度统计中是未送达/FAIL，不能因合并重同步而从母集消失。无负载下不应触发背压；正式负载下任何缺代/过期/预算拒绝计失败。

清理不得采用rev2的“越限先删最旧已发布代”：只可回收不被两代保留、staging、在途接收或活动query引用的对象。replica ACK包含已安全映射的代及引用状态；receiver在打开对象期间与publisher采用有界控制锁/协议保护，不能先列目录后被删导致TOCTOU。只读已映射文件不能被truncate或覆盖；unlinked仍驻留页面/引用仍计资源，不能靠unlink假装释放。活跃调用完成后释放其代引用，不逐出正在执行的query；旧token按两代/5s规则到期。ACK丢失安全保留、有界背压；receiver失联超过2s后进入不可用，重启使用新会话并清理已证明无读者的残留，不根据PID数字单独删除。

合并RSS<=2GiB，派生唯一物理资源<=512MiB：writer新增派生缓存、receiver解码/索引、共享文件/映射、两端队列、staging、重同步副本、在途临时buffer均计入，主OCC存储仍只属于RSS。共享文件按inode/对象去重计物理预算，同时报告双进程RSS之和和PSS/共享页，不能从RSS门中任意扣除。保留G2既有owner_evidence计费规则，若实际构建期超512MiB则RESOURCE_LIMIT，不擅自扩大。1Hz RSS历史外，增加分配/映射/staging峰值事件和各PID high-water，避免漏掉瞬时峰值。

## 5. 状态、新鲜度与重同步

receiver状态：STARTING -> SYNCING -> READY；任何坏hash/缺块/错base/未知schema进入RESYNC_REQUIRED，resync失败进入UNAVAILABLE。不提供部分代。读取验证完成后一次原子切换registry，带新epoch/incarnation时先失效全部旧tokens，再暴露新世界。仅同epoch同producer的保留旧代允许旧token读取。

后台control/head发布周期0.25s，receiver轮询25ms；不新增服务路径对mapper RPC。head从owner当前已提交bundle读取，不用上次已导出代冒充head；包含producer与monotonic心跳时间。提交→head传播延迟也入账。多个事件不得复用过时健康戳续命。writer忙导致control无法发布时同样触发不可用，不靠陈旧健康状态继续服务。

判定：
- 新`use_current`：仅在control有效（age<=2s）、无已知落后head、无错误状态时服务本地最新完整代；否则QUERY_INDEX_UNAVAILABLE，reason=REPLICA_LAGGING/RESYNC_REQUIRED/EXPORT_BACKLOG_FULL。传播中尚未获知新head的窗口只能承诺有界副本读，不能宣称强一致。
- expected identity未收到：SNAPSHOT_IDENTITY_MISMATCH，不等待、不退回旧代。已保留准确身份可按原规则读；token不自动续期。
- 已获知head超前：CURRENT立即拒绝；旧pinned代仅在同producer/epoch、control健康、token合法、落后未超过2s时可读。超过2s全部查询fail-closed，包含旧token。静止地图control持续健康且无新head时可继续读，不按快照“年龄”拒绝。
- producer死亡/重启或geometry/epoch变：旧token具名失效；在新完整代前UNAVAILABLE。replica重启所有旧token失效；新token/cursor密钥导致字节不同是允许且必须测试的会话变化，不要求重启前后token逐字一致。

稳态每笔accepted提交从owner commit_ns到receiver READY(target)测量：每组>=30个提交，P95<=1s、max<=2s。未可见直到2s记失败，不删除/不记作零延迟。已登记故障注入恢复单列<=30s，不混入稳态P95；正常压力不能自行归类为故障以豁免。

resync只允许后台控制消息触发；每个故障episode最多3次，间隔0.25/0.5/1s，累计30s墙钟（含重启/构建/校验）。超限保持UNAVAILABLE并输出原因，不无限重试。相同seq/manifest重复幂等，seq相同内容不同拒绝；乱序旧代不得倒退；坏checksum/缺块/错base/旧incarnation/unknown schema均不进入READY。小块乱序可等完整manifest验收，终态内容必须相同。

## 6. 启动、服务和故障边界

提供 `cargo_map_query_mode=in_process|isolated`，现有默认in_process不动；隔离验收显式isolated。launch在isolated关闭mapper的4个新typed读服务，启动唯一replica；不启动第二份OCC owner。replica可先起，服务返回UNAVAILABLE直到完整同步；mapper先起后replica也能同步。root实例独占锁阻止两个publisher；服务发现检查重复注册即预检失败。

正常请求deadline2s（性能门仍更严格）；服务进程死亡由客户端具名SERVICE_UNAVAILABLE或REQUEST_TIMEOUT，已在途且已完成请求允许合法成功，不要求杀死边界每条都不可用。不得给生产所有客户端无限重试。监管最多每episode重启3次/30s，用同一协议确认新receiver incarnation与READY，超过后保持故障。新健康状态与旧frame runtime readiness区别记录，不放宽原peer门。

授权修改范围：新增codec/export/replica纯模块和测试、replica ROS node、mapper/store的导出hook及4服务注册分岔、现有launch新增选择参数、CMake/package安装登记、无运动eval脚本及文档；旧事实字段只读。查询只读进程不得import/实例化CargoVolumeMapper或运行融合。共享schema拆出必要纯DTO模块时须有原import兼容测试，不借机重构神模块。

## 7. G4-A 实施及验收

实施顺序：A0冻结wire DTO/生命周期和源码基准；A1离线FULL/DELTA导出重建；A2独立进程+启动/健康；A3故障注入与代表组。owner完成测试/修复/证据/提交，不把测试留给另一角色。

| 门 | 工作负载/次数 | 必须观察到的通过结果 |
|---|---|---|
| A1 codec语义 | 三分辨率×两hull×0/5/20箱，seed0..9各100更新；独立只读重建 | 所有读字段/身份等于源快照，源主事实不变；无pickle，无可写别名 |
| A2增量边界 | 跨块frontier-only、标签/instance-only、零cell确认态、shadow/remove、overlap、partial block、reset，各3重复 | dirty全覆盖；checksum覆盖frontier；FULL与DELTA终态逐字段等价；未变对象不重写 |
| A3会话/发布 | 初次同步、查询中发布、过期/两代逐出、cursor tamper、并发读写1000更新×3 | 单响应单代；旧token具名失效，读中引用不被回收；无双服务 |
| A4杀receiver | READY空闲/DELTA中途/查询突发，3阶段×3次=9，固定seed0..2 | 写入accepted及结果对照全等、无因receiver死亡导致写失败；客户端在2s内完成/具名失败；每次30s内恢复正确READY |
| A5传输故障 | 丢对象/截断/损坏/重复/乱序/错base/旧producer/未知schema/断流/ACK丢失；initial FULL和DELTA两个边界各3次=60 | 无混版/倒退；重复合法幂等，恶意/损坏拒绝；可恢复场景30s内正确，持久故障30s内稳定UNAVAILABLE且队列有界 |
| A6写端重启/reset | producer重启、epoch reset、geometry改变，各3次 | 旧token不跨世界；无旧文件误认新数据；冷启完整同步后恢复 |
| A7代表时延 | 1cm倒角0/20箱，D0只读和D1真实30k写重叠，3warm+30查询；每组>=30accepted提交 | ROI P95<=100ms/max250ms；稳态可见1s/2s；无等待mapper调用；span闭合<=max(1ms,5%) |
| A8成本代表 | 原write失败的1cm/0箱/30k/.5/0及1cm/20箱/30k/.9/.5，3warm+30配对b33 | 整写P95<=1.10×基准、max<=1.20×基准max+20ms；export/队列/构建接收CPU/字节/时刻单列；后台工作不漏账 |
| A9资源/取证 | 上述全程和故障峰值；代表ROI/distance/surface取证on/off各30 | 2GiB/512MiB门；零丢/错join，T1 P95比<=1.10；否则不认证受扰动时延 |

A7仍受操作系统资源竞争，wall-threadCPU不设5ms门，保留CPU/锁/GC诊断；不再称GIL证明。A7/A8失败即代表组FAIL，不自动进入B全矩阵。A4/A5每个单位AND正确，无样本/未触发注入not_evaluated，最多每case5次触发尝试且保留无效记录；不是长期成功率声明。fault retry门3次指产品恢复，不是追加实验择优。

## 8. G4-B 完整验收（依赖A PASS）

仅在A全部强制门通过、确切实现SHA落Result后可启用。B先核对A SHA/源码及独立install逐文件hash、完整输入矩阵与资源，再冻结测试；期间不改被测产品。需要修复则停止B、保留失败，回A新代修复再绑定新B基座，不在评分中途改代码。

| 门 | 矩阵/聚合 | 阈值 |
|---|---|---|
| 主事实/旧接口 | 原180000条，固定b33基准和输入 | 全等，无丢失身份/提交语义 |
| 回归 | 原316清单+G2/G3/G4新测试、真实IDL | 零新增失败；新合同无skip |
| 查询18组 | .01/.02/.05×0/5/20×cuboid/chamfer，cold+3warm+30每mode | ROI/frontier/instance100/250ms；summary50/200ms；纯count1/5ms；distance1 5/20ms，256点50/200ms；surface/snapshot100/250ms（P95/max） |
| 写入72组 | 原30k/100k、outside.5/.9、cross0/.5，3warm+30配对 | P95比<=1.10，max<=基准max×1.20+20ms，完整写输出全等；串行无后台余活漏账 |
| capture | 原198组+新增IPC健康join | P95 on/off<=1.10；完整原始记录、零丢/身份错配 |
| placement18组 | 原hit/fallback、三分辨率三箱数612对 | 全字段相同；5s/20s；快照身份正确、预算不重置 |
| concurrency | 1cm 0/5/20各120s×3；2Hz30k写/10Hzsummary/5HzROI/2Hzdistance256/1Hzsurface | 原查询门、backlog<=1s、双向heartbeat max<1s/零lease；accepted全完成、零静默丢或拒绝冒充成功；新鲜度P95<=1s/max2s |
| resources | 所有组含启动/reset/staging/接收/清理 | init/reset<=30s，合并RSS<=2GiB，派生物理资源<=512MiB |

每组独立nearest-rank P95=第29/30；并发各类型至少30完成且满120s，失败/未同步/超时不从分母删掉。缺样not_evaluated；所有组AND才总PASS，只证明固定负载，非长期可靠率。cold超max保存后该组失败，不盲补跑。baseline原压力失败保留，不为比较成功改其逻辑；candidate仍须独立满足门。任何writer吞吐失败不得靠隔离结论豁免。

## 9. 运行与证据入口

复用 `scripts/g3/occ_query_eval.py`，新增 `isolation` 子命令及mode支持，不新造第二套指标计算。A必须在首次运行前通过CLI行为测试，固定输出和退出码0/3/4=pass/fail/not_evaluated，失败不覆盖原目录。

```bash
python3 scripts/g3/occ_query_eval.py isolation --stage codec --seed-start 0 --seeds 10 --transactions 100 --out "$EVIDENCE/a_codec"
python3 scripts/g3/occ_query_eval.py isolation --stage faults --repeat 3 --install "$INSTALL" --out "$EVIDENCE/a_faults"
python3 scripts/g3/occ_query_eval.py isolation --stage representative --warmup 3 --repeat 30 --install "$INSTALL" --baseline-install "$BASE_INSTALL" --out "$EVIDENCE/a_rep"
python3 scripts/g3/occ_query_eval.py oracle --seed-start 0 --seeds 20 --transactions 500 --baseline-revision b33dda91 --out "$EVIDENCE/b_oracle"
python3 scripts/g3/occ_query_eval.py benchmark --mode queries --query-process isolated --install "$INSTALL" --samples 30 --warmup 3 --out "$EVIDENCE/b_queries"
python3 scripts/g3/occ_query_eval.py benchmark --mode writes --query-process isolated --install "$INSTALL" --baseline-revision b33dda91 --samples 30 --warmup 3 --out "$EVIDENCE/b_writes"
python3 scripts/g3/occ_query_eval.py planning --query-process isolated --install "$INSTALL" --baseline-install "$BASE_INSTALL" --samples 30 --warmup 3 --out "$EVIDENCE/b_planning"
python3 scripts/g3/occ_query_eval.py concurrency --query-process isolated --install "$INSTALL" --baseline-install "$BASE_INSTALL" --duration-wall 120 --repeat 3 --boxes 0,5,20 --out "$EVIDENCE/b_concurrency"
```

新增参数是本代交付要求，不能拿现有未接线命令冒充隔离。测试前记录实际--help、独立ROS domain、CPU/RAM/affinity/OS/Python/NumPy/ROS/FS介质、source/install/schema hash、dirty=0；基准和候选不得混overlay。无其他本任务后台测量，记录外部负载。正式评分前无运动启动预检可在30min内修复本任务脚手架，产品失败不能变成脚手架以抹掉。

证据根 `docs/status/evidence/occ_owner_b1/occ_query_isolation_g4_20261026/`，A/B独立新目录。T0每版本输入hash/资源/退出；T1提交→head→encode→published→received→validated→READY及请求身份join、队列字节/工作计数；T2前10s后5s，失败manifest/对象哈希与重建所需对象、原query/response、更新序列、故障脚本和seed；T3仅诊断。每run8GiB/总32GiB，内容对象去重不删失败，超限停止。离线replay先验证对象存在/完整再重建/比对，使用b33原事实oracle而非候选自证。

## 10. 派发与收口

G4-A：subtask `G4-A-ISOLATED-CORE`，owner eng/claude/glm-5.3，generation1（新子项，父计划G4）；base为7863a840，实际从本计划提交建 `/tmp/elfin-occ-query-isolated-g4-a`、`agent/eng/occ-query-isolated-g4-a`，先agent_start领取。先协议DTO提交，再代码，再全部A门，回帖hash与矩阵。

G4-B：subtask `G4-B-FULL-ACCEPTANCE`，同owner，depends_on G4-A-ISOLATED-CORE；初始dispatch_ready=no。A通过后由reviews在B线程冻结准确base及安装身份、标ready；未Claim的草案元数据可据真实依赖更新并追加Binding事件，已Claim绝不原位改基座。本次用户授权涵盖后续B测量，不需要再请求运动授权，因为本任务始终零运动。B只负责全矩阵及报告，不自行产品修复/合入。

总任务未PASS前不替换b33生产基准。G3 blocked是历史终态，不重写为成功；G4-A通过也不是G4-B或n=3通过。任何禁区需求或未能满足门据实blocked，禁止扩大内存/放宽频率/吞掉陈旧或拒绝数据。
