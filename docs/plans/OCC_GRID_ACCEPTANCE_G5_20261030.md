# 自研 OCC 分块网格完整验收计划 gen5

- date: 2026-10-30
- parent: OCC-QUERY-INDEX-20261025
- subtask: G4-B-FULL-ACCEPTANCE
- generation: 5
- owner: eng/cursor/grok-4.7
- candidate_revision: 6f75902202081ee59077faac62289400adfb627d
- baseline_revision: b33dda919521ef043fa9e30b07dcc79c87f76fce
- supersedes: 暂停中的gen4，仅恢复自研网格完整验收；不恢复OctoMap性能对比
- inherited_contracts: 5e287b6a 的业务/压力分判；19df2eed 的原矩阵、接口、合批、资源合同；未明确覆盖项保持

## 1. 当前状态与执行边界

自研完整B验收未通过。历史fastpath代表组b0/b20距离成功P95约65.7/69.7ms超过50ms，漏发3/0；传播观测间隔约1041/1003ms，但不能当精确commit延迟；b20 surface成功17不足30，CURRENT只4次而非正式20次，资源派生预算not_evaluated。叶子批量化/lexsort负结果不属于产品6f759022，不纳入候选。

先完成Q-20261030-3的OctoMap收口并确认无遗留重负载进程，再开始本代计时；前项产品FAIL不阻止本项启动。Cursor可先做非重负载准备。保留所有历史FAIL，不用本次结果改写旧判定。仅评测脚本、专用测试及诊断适配器可改，生产src、接口、默认配置和算法不改；不做Gazebo/运动/n=3，不实施性能修复，不输出与OctoMap的速度比。

使用干净树 /tmp/elfin-occ-e2e-eval-cursor-g4（若已被占用则另建同产品基座专用树并登记）。脚本可从eefb4822及后续已审修订定向移植，禁止整支合并改变产品。工具提交独立，产品差异断言必须为零。

## 2. 机械准入、命令和仪表

冻结 candidate/baseline源码、dirty、IDL、独立install逐文件清单、输入/配置/工具SHA；启动子进程核对实际加载路径/hash，不能仅核源码。重建install视为新工件，不沿用历史摘要。共享接口做b33对照，candidate独有V2走真实服务和独立oracle，不把facade与typed时延作比。

复用occ_query_eval及现有capacity runner/core，先提交行为tests，再输出实际CLI --help和展开的COMMANDS.sh（准备、oracle、查询、write、capture、placement、业务窗、压力、replay、judge所有命令含真实路径/参数）。不存在的子命令先实现测试，不写虚构命令宣布可运行。每条命令映射下节case_id、期望样本数、判官和证据目录；冻结前不评分。

行为判别力至少覆盖：错身份/空身份、重复或缺计划ID、全拒绝、迟到成功、CURRENT重复start及提前重试、合批缺覆盖证明、缺完成、快慢写和有界队列、n=29不足/n=30第29条、记录器全关/人为慢记录器、资源超额/共享去重/短命缓冲、故障重放。每项正负对照至少各一例且符合预期。

事件包括input_ready/send/receive/write_return、融合/派生/快照、offer/排队/编码/hash/文件写/发布、接收/验证/READY、query scheduled/send/callback/result、planner采纳surface与账本及求解起止；全带run/input/request/完整地图身份、进程线程和单调时钟。可观测阶段记录墙钟/CPU/工作量；无法观测的段具名unknown，不用中位数相加代替端到端。

commit无精确事件时保存区间；READY观察减write_return有双向偏差，不称精确时延或单向上界。若有端点区间，逐提交算传播上下界：全满足上界门才能PASS，下界已超才能FAIL，中间无法裁定则not_evaluated。不可将head心跳时间充当publish时间。提交exact/coalesced/timeout/export_failed按真实覆盖身份join，所有未送达留分母。

## 3. 矩阵与验收阈值

| 单元 | 工作量与样本 | 判定 |
|---|---|---|
| 事实/操作/旧读取 | 原180000条固定oracle；原316及相关新增回归 | 逐字段/约定浮点位模式全等，零新增失败；原矩阵逐ID展开，新增不静默skip |
| 查询 | 3分辨率1/2/5cm x 0/5/20箱 x cuboid/chamfer=18组，各类型3预热+30评分 | ROI/frontier/instance P95/max 100/250ms；summary50/200；count1/5；distance单点5/20、256点50/200；surface/snapshot100/250 |
| 写入回归 | 原72组（含30k/100k、outside/crossing），保留原输入/种子；每组3预热+30配对，AB/BA各15 | P95候选<=b33x1.10；max<=b33max x1.20+20ms；事实一致 |
| 取证 | 原198组同路径ON/OFF，每组30对，AB/BA各15 | P95比<=1.10，身份join完整，dropped/failed均0；不以低耗时为由换门 |
| placement兼容 | 原18组/612对响应身份及字段对照 | 固定同输入/配置/Attachment/atlas，无假合法或错误箱满 |
| 真实placement服务 | 18组，每组3预热+30评分 | RTT P95<=5s/max<=20s；目标地图有界获取<=2s；surface/snapshot100/250ms |
| 增量 | 重复operation、同观测新stamp、概率饱和、稀疏、metadata-only、reset六类，各30事务 | 事实与分类变化分账；dirty传播全覆盖，未变块复用；无伪造zero-work |
| 生命周期/完整性 | 原故障矩阵清单逐case登记；fastpath新增边界每个至少1次确定性复演 | 原合同具名拒绝/有界恢复、原子发布、固定句柄不漂移，无静默错读 |

原72/198/180000/612清单先从历史manifest恢复并哈希，不能仅凑总数；缺定义的case为not_evaluated，列清单而不自造替代。旧结果只有同产品、同输入、同工具判据且可追溯才可复用；其它SHA标历史支持，不算本代完成。

生命周期必须覆盖pointer producer/path/digest变化、摘要不符、restart/resync重新校验、payload变化、读写并发持有旧句柄、关闭/杀进程/重同步。稳定pointer下正文原位损坏须报告实际检出时机，不继承“每poll立即检出”。与原故障门不一致则列FAIL/待裁定，不能由脚本豁免。无重复全故障战役：能复用同版本可追溯覆盖即登记复用，其余仅补缺case。

真实业务链使用真实ROS ComputePlacement，显式证明planner读取目标surface和同版本账本，求解内固定身份；覆盖结构命中、route拒绝续取、惰性网格回退和预算截断既有fixtures。输入就绪到选位返回总耗时另测，不把地图等待藏入服务耗时。本轮不设相机Hz或整体观察链时间硬门。

## 4. 持续业务与旧压力分判

业务窗：1cm倒角、0/5/20箱各120s x3、seed0/1/2。3写及各查询3次预热并完成初始同步，评分共同barrier；writer完成驱动单在途，上一笔返回即续投、不等READY、不休眠。1Hz acquire->ROI->surface->release，每种请求至少30成功、覆盖全120s、计划120条链全留档、零missed_schedule，各操作沿用上表门。

业务窗独立客户端在t=3+6k执行20次目标CURRENT获取和真实placement（k0..19）；每次获取2s绝对期限，合法传播等待50ms重试，固定快照持有遵守2s合同，求解用固定输入副本。20/20获取成功，无预算重置；placement使用上表5/20s门，单窗20不足P95样本时P95标not_evaluated，不能拼3窗代替；18组正式30样本服务门负责P95。重叠请求不能丢弃，若有限并发无法发出则具名miss，不能顺延成按时。

旧压力两档分别保留：完成驱动写+C1原summary10Hz/ROI5Hz/distance256 2Hz/surface1Hz；固定2Hz写+C2同查询负载。各0/5/20箱120s一次、seed0；C2单发送+单等待，240计划到达，not_admitted_capacity保留身份。每类型单在途、不补偿突发；账本闭合，30s排空。旧distance50ms、零miss等仍按原门判，不以新业务频率洗绿旧失败。

业务和压力均记录完整120s、至少30完成写、每类成功>=30，否则not_evaluated；严格CURRENT拒绝不冒充成功，不要求每个瞬时读都成功。传播P95<=1s/max<=2s按第2节计量资格判。双向health max<1s、零lease（真实frame-runtime路径，不以假心跳代替），无法接线为not_evaluated。容量不足不等于产品错误；报告完成率、每10秒完成数、队列年龄和峰值，不用倒数中位数当吞吐。

## 5. 资源、统计与停止纪律

合计部署RSS<=2GiB；派生唯一物理资源<=512MiB；init/reset<=30s。主OCC只计RSS；writer缓存、receiver解码/索引、共享对象、队列、staging、resync、临时buffer均入派生账。同一物理对象去重，独立拷贝分别计，不把磁盘+RSS+PSS简单相加。至少1Hz RSS/PSS、VmHWM与分配/释放事件；观测0不等于峰值0，缺归属=unknown/not_evaluated。

资源代表组1cm倒角0/20箱各3预热+30提交及一次受控FULL resync，先通过仪表fixture再计量；资源超限或不明导致无法安全计费时停止依赖组，保留低风险离线对照。部署与init/reset峰值全程计入，不把后台成本移到窗外。

P95=ceil(0.95*n)-1，30条取第29条；所有终态/成功子集分别出表，>=30成功才能签时延PASS；不删首尾、不择优重跑、不跨组池化。三次业务窗逐次AND通过，只表示有限样本全通过，不推断长期100%。任何错身份/假合法/事实错误零容忍。

本代使用此前授权16小时实际测量额度的剩余额度，先核对旧gen4已消耗量；锁等待不计，不额外叠加16小时。到期未测记not_evaluated。统一perf锁，与OctoMap串行；每run记录外部负载/governor/affinity/线程/CPU/软件/文件系统。每操作30s超时、子进程30分钟、窗口排空30s，超时存证不自动补跑。

顺序：版本和门清单审计 -> 工具行为tests与冻结 -> 正确性/资源代表组 -> 查询和真实业务链 -> 写入/取证/其它回归 -> 持续业务 -> 旧压力 -> 汇总。性能失败完成有界组后继续独立组；正确性/崩溃/资源失控停止依赖测试。旧压力FAIL不得吞掉业务验收结论，但完整旧合同不签PASS。不得为某一距离瓶颈再次自行优化产品。

## 6. 交付与派发

证据根 docs/status/evidence/occ_owner_b1/occ_grid_acceptance_g5_20261030/。交付GATE_INDEX（每门阈值/工作量/版本/原始来源/判官/状态/缺口）、MANIFEST、COMMANDS、逐请求raw、资源账、失败可重放输入、独立judge、FINAL_REPORT。每run<=8GiB、总<=32GiB；失败前10秒后5秒边界和必要输入前缀保留，配额满具名停止不删除关键证据。

最终四项分判：事实/功能完整性、实际业务链、旧压力合同、资源/恢复；再列完整产品验收结论。任何强制项FAIL或not_evaluated都不能称完整验收通过，允许“业务通过、压力失败”。工程任务交付完成与产品PASS分开。产品修复只提具名缺陷和最小后继范围，不实施。

派发Cursor/Grok-4.7，gen5替代暂停gen4的自研部分；OctoMap收口为时序前置而非产品PASS依赖。后端比较gen2继续dispatch_ready=no，完成本任务不自动解锁比较或生产迁移。
