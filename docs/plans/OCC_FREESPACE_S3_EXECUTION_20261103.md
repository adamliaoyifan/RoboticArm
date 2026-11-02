# S3 一次性执行与验收清单（2026-11-03）

本文把已批准合同及计分前复核要求汇总为执行清单，不新增产品门限或验收行。责任人 eng/claude/glm-5.3；任务 Q-20261102-2 / OCTOMAP-2CM-REPAIR-S3 gen3；父任务 OCC-QUERY-INDEX-20261025。原 plan_revision 仍为 8caa0d44a3c6ce3b8adec02c4bb7bf1dd612e451；计分前补充依据为 reviews 提交14eda04a。不得改写已领取合同。

## 1. 起点、结果与责任边界

- 产品/工具起点：3bb26935f091055d77b9defa87511489cbfb32e6；交付证据头75ec78fae792910be1de24ef4c1809dbf17d6199。
- worktree：/tmp/elfin-occ-freespace-impl-g4；历史证据：docs/status/evidence/occ_freespace_final_20261102/implementation_g4/。主工作区docs/agents唯一协调，不在卫星树建第二邮箱。
- 已复核：33/33冻结字节、七反例拒绝/两对照绿、53测试通过/1跳过、冻结C++行为通过、四操作O9原始配对复算。不是正式P/R/L成绩。
- S3可以领取；原COMMANDS score不能原样启动，先完成第3节有限集成修正。修正、验证、冻结通过后，owner自行继续正式评分，不另等一次泛化reviews、不再派implementation gen5。
- 最终必须产生真实pass/fail/not_evaluated；工程执行完成不等于产品通过。测不过可以收口，不要求改到全绿。
- 只测几何可放置候选与现有placement消费链；不含机器人可达性、运动规划、Gazebo、生产部署、Orin推算或后端公平比较。

## 2. 环境、输入和语义冻结

| 项 | 固定配置 |
|---|---|
| 平台 | 当前x86；Release；ROS2 Humble；OctoMap1.9.8；discretize=false；powersave；不绑核 |
| 线程 | 单积分worker；发送、查询独立绝对时钟调度；客户端executor显式4线程；记录实际节点线程/CPU，不把nproc当利用核数 |
| 主负载 | 2cm、chamfer、30k、outside=.5/cross=0；P为0/5/20箱；R/L为20箱 |
| 几何/config | docs/status/evidence/occ_o6_o7_20261029/run01/fixtures/geometry_chamfer.yaml 与 config_chamfer_r0p02.yaml，逐字节SHA |
| 箱体查询 | 0.30×0.20×0.20m与0.50×0.30×0.30m交替；每次一种尺寸、yaw=0/90两朝向、全部候选，不取top-N替代完整输出 |
| policy | support_tol=.05m、min_support_ratio=.6、top_clearance=.03m、same_layer_clearance=.05m；ULD hull限制 |
| 性能ROI | 索引起点(0,0,0)、排他终点(16,16,16)，4096格；另固定物理[0,.16m)^3在2cm为512格，只作语义对照 |
| 入口 | 沿已资格化topic/service配置；raw等待队列2、integrate等待队列2；点云Reliable/KeepLast32、回执深度1024；实际运行值入证据，不临时调大队列择优 |

unknown不等于free；空树NO_DATA；已有观测但无可行known-free体积为NO_KNOWN_FREE。floor_prior只表示地板存在，不能证明整个箱空。支持面、横向/顶部净空、完整体积、hull均验；surface高度摘要不替代体积验证。正例必须返回oracle要求的可行集合，不能全部返回无解作弊。

正式manifest逐输入列seed、几何/箱数、点数、outside/cross、epoch/sensor/observation ID、时间戳、计划序号、输入SHA或可校验无损生成规则。不得用重复ID降低实际积分量，不能只以六帧预演清单声称全部正式输入冻结。生成器/参数/seed序列也有SHA；每帧TF同时间戳可重放。L周期输入保持原生成器规则。

## 3. 计分前有限集成检查（≤2h，不含等锁）

只补以下评测接线，不扩展产品功能：

1. **正式汇总**：O8/O5/O6接各自真实调度产物和原始判官，检查固定组/窗/样本集合及内容，禁止只比较passed==total。review证据中的0/0、true/true、30/30但无原始样本九案全部拒绝；足额健康对照通过。启动未运行与已开始但缺样分开，不能把失败降成未评估。
2. **报告转换**：保留qualification verdict；正式run使用独立run_id及冻结SHA生成formal verdict。合法not_evaluated→pass/fail是新运行结果，不当篡改；同run删样本/改结果/谎报摘要仍拒绝。20行原样保留。judge在实际失败时非零，未评项目不冒充通过。
3. **预算、锁与依赖**：统一8小时正式预算覆盖P/R/L/O8/O5/O6及正式复算/失败收集；等perf lock不计。清楚指定唯一锁持有层，避免嵌套flock。P失败停止R/L；O5失败停止O6；其他安全可执行阶段按预算继续。保留退出码，不能靠shell set-e误跳过安全阶段或用||true掩盖失败。
4. **完整流程负例**：走score调度→adapter→磁盘artifact→judge→20行报告，至少覆盖健康、缺样、超门、错误状态、非法数值、伪汇总、输入/内容损坏、已有资格到正式报告的正常转换。旧七反例和缓存受控反例回归。用有界同源预演验证接线，不启动长窗验证启动顺序。
5. **冻结**：提交工具修正，记录产品/二进制与gen4的差异。纯离线汇总变更可保留原O9，必须证明产品、采集、时钟、负载及受测二进制SHA均不变；若改变上述路径，重测受影响F/C/O9。不得重新构建出不同二进制却沿用旧资格。

预演失败保留证据再修；2h结束仍不成立则输出具体阻塞与未评行，不能假启动或无限延长。禁止因发现非阻断的优化建议拖延评分。

## 4. 锁与资源行为资格 F/C

F：9类（空树、观测空箱、满箱、部分unknown、遮挡、悬空/夹层、边界、两版本、reset）×10seed×2尺寸×2yaw=360。真实核心与独立逐体素/hull oracle候选集合、计数完全一致；假free/碰撞/越界为0；已知有解正例返回至少1个。

C：9类路径各30次：同revision共享、不同revision、外部buffer最后持有者、8/9句柄、TTL、reset、异常、缓存并发、写锁屏障。通过条件：

- 实际store写锁保持1s时，已有租约ROI/Surface/free-space在200ms内完成且结束时锁仍持有；不是模拟延迟。正式计时hook关闭。
- 新租约/Wait等待有界；8个在途名额真实计入，第9个WAIT_PRESSURE≤500ms；不持仓库锁等store，不持数据锁做ROS序列化/网络等待。
- reset后旧句柄不可重新取得；已经开始的不可变读取可完成；TTL/release遵守原合同；异常finally释放。
- 同内容缓存共享不重复记账；只有外部数组持有时继续计账；最后一个所有者释放归位；顺序重复释放不以累计flow抬peak，并发峰如实计入。
- 受控持租约产生残留时独立检查器点名对应类，释放后回基线；盲化、不恢复、错误分摊负例拒绝。

资格复用须绑定原始证据与SHA；只在相关变更后重跑受影响项，不把资格重复运行混入正式样本。

## 5. O9采集资格

四操作：ROI、30k integrate、首次Surface、完整free-space查询（含acquire、全部页/结果物化、release）。每操作30对ON/OFF，AB/BA交替，同输入、同版本。首次Surface必须miss。

每操作P95(ON)/P95(OFF)≤1.10；30/30配对，全部状态合法、请求/观察身份对应、完整trace无丢事件；内容摘要匹配；实际失败可重新执行产品重放。O9只证明采集扰动，不证明绝对性能。纯离线工具修正是否复用见第3节。

## 6. P/H正常负载（第一正式性能门）

0/5/20箱各3窗，每窗120s，九窗AND。每窗独立t0：输入i=0..299在t0+i/2.5；查询j=0..239在t0+j/2。先完成服务/TF/首积分/已知快照就绪，记录预热，随后t0之后任何错误都在分母中。

| 指标 | 每窗门限 |
|---|---|
| 更新 | 恰300输入；完整身份每ID恰一成功终态；拒绝/unknown/重复处理=0 |
| 输入端到端 | 实际publish调用开始→完成终态可观测；P95≤350ms、max≤500ms；同时保留publish返回与receipt/status边界 |
| 完整query | 恰240；实际调用起点→全部候选物化并release完成；P95≤250ms、max≤500ms |
| 调度迟到 | publish/start减各自计划时刻；输入和查询分别P95≤20ms、max≤50ms；不混入query服务耗时 |
| 新鲜度 | 每条正常查询快照年龄≤1s，clock域明确；不是仅map revision推进 |
| health | 原始单调时间戳重算最大间隔<1s，覆盖窗边界；缺数据不通过 |
| 闭合 | t0+120.5s前该窗所有任务闭合，无积压增长 |
| 完整性 | 页身份/游标/数量/唯一性/候选域/最终done/摘要/释放均一致；错误不伪装空结果 |

300×9=2700输入、240×9=2160完整查询。单窗失败就是P/H失败，不跨窗平均；完成九窗的独立结果，若进程崩溃/不安全则剩余未评，停止依赖的R/L。查询NO_KNOWN_FREE可为合法成功；正常评分中NO_DATA/STALE/压力/timeout不是成功。

## 7. R过载、排空、恢复

20箱，同observer PID、epoch、地图连续，不reset、不重启、不丢历史队列。

| 阶段 | 输入 | 查询 | 门 |
|---|---:|---:|---|
| 过载60s | 10Hz，600 | 2Hz，120 | 所有600 ID有明确终态；允许明确拒绝；至少发生一次实际排队或拒绝，否则压力覆盖not_evaluated；已接纳者最终完成 |
| 固定排空30s | 不再发过载帧 | 2Hz，60 | 以固定截止前到达的真实回执/事件判在途=0；截止后查询成功不能补签排空 |
| 恢复30s | 2.5Hz，75 | 2Hz，60 | 75输入全成功；首Wait与首恢复输入同t0并绑定其完整ID；Wait≤2s、年龄≤2s；使用该租约完成查询并finally释放；其余59条沿P/H |

过载/排空的明确压力、STALE或有界timeout按阶段合法集记录，不能泛化允许所有ACQUIRE_*。每条请求有结果，不把NOT_SEEN当拒绝或成功；订阅缺口与状态查询找回的终态分别列，不合并掩盖边界缺失。首Wait完整链耗时单列，不能只给wait RPC时间。三线程共享计划起点，发送不得等待查询join。R失败保留，停止依赖的L计分；安全后续门按预算执行。

## 8. L两小时持续与无泄漏

20箱，连续120min；18000输入、14400完整query。前10min预热，所有错误仍记录；后110min计分16500输入、13200query，分55个独立120s窗逐窗沿P/H门并AND，不用全程平均掩盖坏窗。

- 同时完成1000轮真实acquire→读→显式release；失败尝试单列，未取得租约不计数，不通过无限重试洗掉主评分失败。100轮真实跨reset/异常生命周期放在预热/专用已记录子阶段，不能在110min评分窗随意reset造成NO_DATA豁免。
- 每轮保留按所有者类别的原始采样、单调时间、最后释放事件。lease/client-held块与目录、外部surface buffer、inflight/scratch等请求生命周期类回归本轮明确基线；store retained当前地图允许合法变化，必须解释所有权，不强求不同地图字节相等。
- 并发瞬态可有界等待消退；逐类min只能辅助判断，全部采样和同时存活峰保留。受控持续缓存/块残留必须被拒；目录归位不能代替所有字节归位。
- RSS最大≤2GiB（2097152KiB）；同时存活read_view+cache组合峰≤512MiB（536870912B），不是单类峰相加或累计分配flow。覆盖缺项不能认证通过。
- RSS至少1Hz、每个计划10min桶≥600样本，时间单调、桶完整、无NaN/Inf/bool；当前0.5s采样可保留。第110–120min RSS中位 / 第10–20min中位≤1.05；中位口径沿冻结判官nearest-rank下中位，不静默换成中均。
- 无崩溃/死锁；实际线程/CPU/上下文切换/频率/外部负载入证据。RSS不立即下降不自动判泄漏，分配器保留与活对象字节区分。

结论只允许“在120min与规定生命周期中未发现泄漏”，不宣称数学上永不泄漏。

## 9. O8概率与地图稳定性

固定射线、递增时间戳，180s×3；每轮首60s预热；后两个60s RSS中位增长≤5%。另30帧饱和核验：恰30个完整匹配终态，饱和后changed_keys=0、几何变化块=0，无新键时节点数稳定。

no-input、真实几何遮挡各10s×3，重复ID另列3次对照；持续时间实测≥10s，不四舍五入放过9.x秒。固定旧区域由fixture推导483516格，保留每格state字节与float32 log_odds无损字节、区域、页身份、SHA和release；判官从磁盘解码复算长度/域/有限性/内容，不信自报“未变”。旧区域概率保持；没有接纳新数据时新鲜度不前移。真实遮挡允许新障碍区域合法更新，不能用重复ID冒充遮挡。全部hold同租约完整分页，不裁剪probe-content。

## 10. 原O5/O6能力矩阵（独立报告）

O5：2cm chamfer，0/20箱×30k/100k=4组；每组3预热+30评分，另30首次Surface构建样本。O6：2/5cm×0/5/20箱×chamfer/cuboid×30k/100k×outside .5/.9×cross 0/.5=96组；每组3预热+30评分；沿已冻结执行器每组30个新进程冷启动样本，预算据此预估，不把冷启动并入热样本、不缩样凑完整矩阵。

| 指标 | 每组门限 |
|---|---|
| 30k integrate | P95≤500ms、max≤1000ms |
| 100k integrate | P95≤2000ms、max≤5000ms |
| acquire、ROI、Surface首次/更新后/重复各自 | P95≤100ms、max≤250ms |
| 初始化 | 每个冷启动≤30s |
| 资源 | RSS≤2GiB；完整归属read_view+cache组合峰≤512MiB |

正式样本索引为预热后30条（行3..32），不混预热、不漏末尾；首次/重复/更新后三种Surface必须真实发生。O5失败不跑O6。1cm不新增测试，历史1cm失败保留。本矩阵不代签2.5Hz/2Hz；预算不足O6可未评，不影响已完成的新目标结论，但不能称全部矩阵通过。

## 11. 判官统一规则与固定20行

- P95为升序第ceil(.95n)个，n=30取第29；max保留所有样本。错误、超时、缺字段、NaN/Inf/bool/负值不能删出分母。
- 频率以计划/实际起止和逐请求截止衡量，不以总吞吐或线程数代替。100%仅指本次有限样本集合；九窗/55窗为AND，不宣称长期100%可靠。
- 缺样/错误已发生是fail；预算/上游依赖使阶段未启动是not_evaluated。资源未覆盖not_evaluated。绝不将0/0、全拒绝、全无解、全未运行判pass。
- 页脚-only、丢事件、合法SHA但内容/长度错、未重跑产品的伪重放均拒绝。每个门有健康对照和故障对照。
- 资格14行：F.matrix_360、C.behavior_9x30、C.owner_class_split、G4.seven_review_badcases、G4.counterexample_controlled、P.record_contract、H.scheduling、R.overload_drain_recovery、L.read_release_lifecycle_real_1000、L.reset_lifecycle_100、L.ownership_per_class_reconciliation、L.rss_stability、O9.four_op_pairs、Evidence.validity。
- 正式6行：Formal.P、Formal.R、Formal.L、Formal.O8、Formal.O5、Formal.O6。资格行名称中的1000/100不是短预演已完成该数量；正式数量由Formal.L核验。
- 主目标通过需要相关资格成立且Formal.P/R/L/O8全pass；Formal.O5/O6单列。主目标存在fail则fail；无fail但缺必要项则not_evaluated。工程任务按实际执行和证据闭合情况收口。

## 12. 执行顺序、时间预算和停止规则

顺序：领取→计分前有限工具修正→同源有界预演→冻结/资格核对→P/H→R→L→O8→O5→O6→独立复算→失败重放/最终报告。启动失败在首正式样本前不消耗样本，但记录次数、原因和耗时，受2h预演预算约束；计分开始后不得借“启动问题”删除失败。

正式8h（不含等锁）为统一预算：P最低18min、R约2min、L固定120min、O8至少9min稳定窗另加hold/探针时间；余量优先满足复算和失败保存，O5/O6基于预演实测估时后排入。阶段启动前检查完整阶段预计时长及收尾余量，不能确定能完成则记未评，不削减样本或L时长。不得执行他人进程终止来获取独占。

| 事件 | 后续动作 |
|---|---|
| F/C/O9资格不成立、冻结不符、接线未通过 | 不计分；有限修正预算内处理，否则记录阻塞收口 |
| P失败 | 保留所有已评分窗；不跑R/L；安全O8/O5可继续 |
| R失败 | 保留；不跑依赖恢复状态的L；安全门继续 |
| L/O8失败 | 保留；安全旧矩阵按剩余预算继续 |
| O5失败 | O6未评，说明依赖失败 |
| 崩溃/死锁/捕获损坏 | 先落失败前缀和资源/线程信息，再清理自有进程；不可盲重跑 |
| 预算耗尽 | 未启动项not_evaluated；已失败仍fail；输出已完成结果 |
| 正式阶段需要改产品 | 结束该候选、保留失败，另列后续修复；不在本run拼多个版本 |

## 13. 命令与产物交付

已有入口（精确路径）：`/tmp/elfin-occ-freespace-impl-g4/docs/status/evidence/occ_freespace_final_20261102/implementation_g4/COMMANDS.sh`。它是待有限修正的起点，不是当前可直接执行的最终score命令。owner在独立S3工作树提交最终COMMANDS.sh，所有路径/参数实填并冻结，提供以下独立入口：build/tests/fmatrix/rehearsal/counterexample/freeze/o9/score/judge/replay/verify/budget。

领取：主工作区 `scripts/agent_start.sh` 按Q-20261102-2真实路由领取；禁止伪造领取事件。评分必须显式OCC_FS_SCORE_OPEN=1且OCC_FS_SCORE_REHEARSAL不为1，由最终脚本检查冻结后进入；非计分明确rehearsal标记。正式路径可直接完整执行，也可按原顺序逐阶段恢复调度，但不得重复已有计分单位。最终ACTUAL_COMMANDS.log记实际命令、环境、开始结束、退出码；不能只提供示例命令当执行证据。

新证据使用docs/status/evidence/occ_freespace_final_20261102/scoring_<runid>/（runid由owner在首次执行前确定写入冻结manifest），历史implementation_g4不覆盖。保存：

- FREEZE.json、input_manifest、环境/二进制/工具/库/typesupport/fixture/脚本SHA、clean状态及允许的仅证据提交血缘。
- T0/T1全程边界记录；T2失败前后各5s（预演若不够则评分前扩大并冻结），完整地图输入前缀或可校验无损生成manifest、同stamp TF。
- 逐ID发布/回调/入队/开始积分/完成/终态落账/回执发布接收/状态查询；每页request ID/租约身份/cursor/结果/释放；queue、lock wait、enumerate、fill、客户端完整耗时。
- health/RSS原始时间序列、每类live/同刻peak/累计flow、所有者及释放事件、线程CPU/上下文切换/频率/外部负载。
- O8无损probe-content、F oracle、O9全部pair及trace、失败原文和重放输出。只无法直接观测的传输/调度耗时保留residual，不编造归因。
- 每例≤512MiB、每run≤2GiB、总≤8GiB；压缩/共享可校验前缀，不截断重放必需数据。容量预算评分前核算；超限风险不得用删关键样本处理。
- capture_complete、replay_possible、缺项、逐文件SHA。重放必须在干净隔离进程执行真实产品并比较身份/状态/内容；不以JSON读回或时间性能复现作为失败原因已复现的替代。

## 14. 一次收尾的交付物

1. 固定20行最终verdict：每行pass/fail/not_evaluated、样本数、阈值、实测P95/max/峰值、证据路径、未评原因。
2. 主目标报告：是否在本次有限集合实现2.5Hz更新+2Hz完整查询、120min内资源有界；合法无解率与查询失败率分别列。
3. 每个失败的边界时间线和瓶颈定位：store/仓库锁等待、TF/入口/worker排队、积分、枚举/首次缓存、填充/传输残差；只有实测支持才定因。
4. 独立旧矩阵O5/O6结果、旧1cm历史失败、新Wait恢复与旧立即查询语义分列。
5. 工程交付结果、实际命令/版本/输入清单、重放可行性、后续事项。报告可以以产品fail结束；不继续无限修复，也不把not_evaluated包装成pass。
6. owner通过agent_complete按实际工程结果关闭任务，Result明确产品结果。无需为了取得“工程pass”把产品结果改绿。

## 15. 起点绑定哈希

FREEZE SHA256=8d1dbd6beb46dd24718eae883501633bdfc74dc3991bdd4a895ab55e7568cb83。
COMMANDS SHA256=7e79edad40d7d13fdc0adf90b8e0917a0b07e8b20035a1fbd3373835d401af8b。
FREEZE.inputs规范JSON SHA256=f1f0a061660339e174d61e093aa3d6f015eb0b4254e6ffb027fb0f451f55ad02。
这些绑定gen4已复核起点；S3新工具和正式输入追加自己的SHA，不覆盖或冒用旧哈希。

参考：OCC_SURFACE_FREESPACE_FINAL_DELIVERY_20261101.md；2026-11-03_0715_occ-freespace-g4-review.md；implementation_g4/COVERAGE.md与implementation_g4_review证据。
