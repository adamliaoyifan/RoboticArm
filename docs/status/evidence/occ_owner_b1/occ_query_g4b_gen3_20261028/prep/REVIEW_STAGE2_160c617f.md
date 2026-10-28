# Stage2 160c617f复核与补齐顺序

- revision: 160c617f
- verdict: wiring_not_accepted_for_scoring
- scope: 离线源码/已有raw复核，未运行ROS/仿真/性能

## 已验证与边界

runner脚本SHA256=4da0d20f5c9405d5f48bfa075ecb5127116fdc72c82974622ed63f8c5e373442，与清单相符。独立pytest两个文件39 passed in0.03s。但resource_meter专用test目前为worktree未跟踪文件，160c617f树不包含该文件，不能声称提交可复现39测；冻结前补提交，不改他人文件。

接受双子进程可运行、install路径可记录、核心/资源fixture本地通过；不接受完整接线完成、真实并发评分已可用或原始事件已包含全部判门要素。以下无需额外全量实验，均由已有源码/raw定位。

## F1 P1：summary真实失败被wiring_ok掩盖，含产品边界缺陷

prep/harness_smoke_8/events_query.jsonl中summary 61次AttributeError("'_MapSnapshotView' object has no attribute 'occupancy_confirmed'")，另2次REPLICA_LAGGING；summary零成功。run.json却wiring_ok=true。该run script_revision=2222487a且dirty=true，不能签160c617f同SHA结果，但160c617f相同调用路径仍在。

产品core cargo_map_query.py:358-369的summary读取occupancy_confirmed/pending_changes/change_event；wire cargo_query_snapshot_wire.py:339-367的_MapSnapshotView未提供这三项（第一项已有实际异常证据，其余为源码合同缺口）。这是绑定产品副本summary路径问题，不是正常CURRENT传播等待。

先保留raw、构造最小wire→真实summary离线反例并提交A后继修复范围，不许B脚本捕获AttributeError后填默认值、从mapper旁路读取或吞错。产品修复必须另行批准/绑定，不改现有0da4086b安装。脚手架wiring_ok须每类有合法成功且错误具名，任何未预期异常否决接线，不能只要求总query非空。

## F2 P1：C2没有独立墙钟到达，且结果被硬编码

runner:253-281把write_once同步放在到达循环内；写慢时期间到达未送入C2Admission，写返回后先清发送槽再补处理旧到达，无法产生真实单在途+单等待压力。int(window/rate)+1还生成t=window的第241帧（6秒smoke的13/13恰是同一off-by-one），合同要求120秒240帧。

on_send_result无论apply返回accepted真假均填committed，propagation直接填exact_visible；closed/drained=True也不依据真实发送/READY终态。这不是仅“缺评分判官”，而是输入事件已失真。

修为调度器与单写worker分离，墙钟到达不被apply阻塞，真实accept/result回传驱动状态机、FIFO晋升；t<window，窗后仅drain。不得在写返回时猜传播，须由完整身份链关联实际READY后填分类。用20ms/1.2s受控写替身复演240帧与积压/拒绝即可，不跑正式120s矩阵。

## F3 P1：没有共同评分起点、预热混入汇总

两worker预热后各自重置clock0，没有READY/预热屏障及共同start_at。smoke8实际query warmup_done t_ns=3800033484075898，writer=3800033766526817，相差约282ms，查询已评分而写仍预热。主进程截止却从spawn后计时，部署初始化长时会提前终止评分窗。

解析写/查询时收集全部event，未按warmup_done/phase过滤；smoke8 manifest的120 queries=6s×18Hz+12预热，预热并未排除。需共同barrier+start_mono、phase字段和两端评分区间校验；启动/预热有独立有界期限，评分120s从共同起点算。--drain-s必须传入worker，不沿用内部硬编码30；超时terminate/kill要wait并留具名终态，不能仍wiring_ok。

## F4 P1：提交/合批/READY时刻与身份不足以判正式传播门

runner:549把submit_t_mono当commit_t；可另报submit→READY上界，但不能替代冻结commit→READY门（会把融合耗时计入传播并误判失败）。offer链是接纳导出请求，不是成功发布或实际合批终态。每offer修订区间不足以证明跳过的中间代被下一实际READY覆盖；仅rev:N还丢epoch/producer/digest。query循环在manifest seq变化时即emit replica_ready，没有要求本次READY状态。

应在不修改产品行为的边界适配中记录真实提交完成身份/时刻、offer状态、实际published目标/覆盖记录和有效READY；同世界完整身份join，明确区分exact/coalesced/timeout/export_failed。无精确commit事件则该指标not_evaluated，不猜测；可同时保留上界。添加屏障驱动offer多代合批→只见末代READY、错误epoch、publish失败、非READY但manifest存在等离线接线反例。

## F5 P1/P2：查询测量仍有遗漏/变负载

- attempt_current用调用前now记录获取结果，且now是本轮replica.poll及其他查询之前缓存值；真实获取超过deadline仍可能登记期限内成功。必须记录真实send/result时刻，50ms重试以真实返回为锚，绝对deadline不改。
- POINTS*85实际255点，不是256；把输入长度断言写入运行器。ROI/云生成器须与原冻结输入核对，不以便利fixture替代负载。
- run_query串行阻塞查询循环，scheduled/callback/result目前缺完整逐请求join；scheduler_stats只存最后20条issue_log，不能审计全窗。handle异常路径未finally close会影响会话与资源，应统一context/finally。
- 清单声称rng(20261023+1000*seed+index)，实际make_cloud(index)忽略index、复用rng(20261023+seed)流；原计划要求继承输入hash，不能此时自行换生成器。逐写sha16只说明不同输入，不能证明原矩阵身份一致。
- dir_sha12另用glob且跳隐藏文件/忽略symlink，不是已绑定v2；只记录模块路径并非校验。准备/评分前校验v2完整manifest、实际加载文件hash；不重建install。

## 资源仪表状态

8个fixtures仅证明给定条目上的记账算法，不证明事件源完整。当前high_water是1Hz采样最大值，不是/proc VmHWM或分配事件峰值；未接入派生事件时不签512MiB。空entries亦不能因peak0/complete真而认完整，必须由预期事件源覆盖证据控制not_evaluated。共享key须按实际物理对象而非内容相等就去重，schema与丢事件检测在事件接线时冻结。

## 决定执行顺序（不增加范围/批次）

1. **保存并暴露失败**：F1最小产品反例与A后继修复提案；脚本判失败/退出完整性先修。B评分停，A不全重跑。
2. **修真实事件生产**：F2/F3/F4和F5时间/负载/身份/完整raw；同时落实run_id新目录/manifest及文件配额。运行器不是产品，仍scripts+专用tests范围。
3. **接采集与故障窗**：真实双向heartbeat/lease事件、资源分配/释放/staging/队列/resync事件及high-water，T2失败前10秒后5秒可重放；同一事件源复用，先受控fixture，不盲跑完整栈。
4. **最后实现离线judge**：输入为已验证raw；覆盖错误响应、错覆盖、过窗、缺源/缺样/丢记录的负对照，不以纯计数代替合法成功。资源缺源即not_evaluated。
5. 补齐未跟踪测试、纠正冻结清单、脚本SHA与完整继承矩阵，再在产品绑定解决后进入代表组。已通过的核心具体反例不循环重审。

所谓五项残差顺序因此不是先写judge，而是先真实事件/捕获，再judge。保持不评分、不优化产品、不放宽lease；产品缺陷需要新批准/绑定，脚手架准备可在既有授权内继续。
