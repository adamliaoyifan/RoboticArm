# OCC G4-B gen3 验收合同：容量实测与节拍解耦

- date: 2026-10-28
- status: approved_pending_install_binding
- parent: OCC-QUERY-INDEX-20261025
- subtask: G4-B-FULL-ACCEPTANCE
- generation: 3
- owner: eng/claude/glm-5.3
- candidate_revision: 0da4086bce23be49ec8eeb934a15fb6829803abe（源码已核实干净；独立install待绑定）
- baseline_revision: b33dda919521ef043fa9e30b07dcc79c87f76fce
- supersedes: G4-B gen2 的节拍、并发聚合及派发规则；不追改历史结果
- inherited_plans: 1c0049d4 的 OCC_QUERY_G4_AB_PROTOCOL_20261026.md；bcaa882165ac72eb94b3491ba7d6fd25b1908fa2 的 OCC_BLOCK_INCREMENTAL_G4A_G2_20261026.md
- dispatch_ready: no
- authorization: 用户要求修改原 B 吞吐门、放宽节拍、按新现状修改合同

## 1. 本次裁定与不变边界

目标是保证 OCC 事实正确、查询及时、增量维护有效，并报告当前实现尽可能连续工作的实际容量。取消“生产必须持续处理 2Hz 点云”的预设，不把固定 2Hz 输入下的容量不足直接判为产品不可用。

本合同只改变验收，不修改生产输入、融合、查询、lease、队列或调度。30k/100k 是测试负载，不是 depth camera 点数上限；禁止为过门裁点、缩减射线或换稀疏场景。只读查询进程必须是真独立进程，写入必须与查询实际重叠，不能用同进程 facade 或写完再查冒充并发。

原合同 §8 concurrency 及“任何 writer 吞吐失败不得豁免”的概括句由本文 §3-5 替代。原 §10 禁止擅自降低频率的条款仍约束执行者；本文是用户明确授权的版本变更，不授权运行中临时降载。旧失败保留，不按新门重标历史 PASS。

## 2. 门的继承与变化

| 项目 | gen3 裁定 |
|---|---|
| 固定 2Hz 写入 | 从生产硬门改为独立压力画像；不承诺生产频率 |
| 固定输入 backlog<=1s | 不再决定总 PASS；保留超过 1s 的比例、最大值及增长率 |
| 写入运行方式 | 主验收最多一笔写入在途，完成即提交下一笔，不等待副本 READY、不插入人为休眠 |
| 写入回归 | 保留 72 组同输入配对：P95<=b33×1.10；max<=b33 max×1.20+20ms |
| 查询/心跳/身份/故障 | 原门保持；不得以容量不足豁免错误结果、lease 失效或实际请求超时 |
| 传播 | commit→首次覆盖该提交的 READY，P95<=1s、max<=2s；逐提交分账，不测 head 心跳年龄 |
| 资源 | 保留原 2GiB RSS、512MiB 派生唯一物理资源及 init/reset<=30s；覆盖/归属仍待单独裁定 |
| A 已关闭证据 | A7@64f0bdf0、A8@ed44accb、A9@0da4086b 保留，不以本修订要求重跑 |
| B 完整验收 | 仍为独立矩阵，不用 A 代表组替代 |

以下原 B 门逐项继承：180000 条事实/旧接口全等；原316项及新增回归（新合同不得 skip）；18 查询组；72 写入组；198 取证组及 IPC 身份 join；18 placement 组/612 对；增量工作计数；资源/init/reset。组内输入、种子与基准不变，运行前从原 manifest 固定完整清单。

查询 P95/max（ms）：ROI/frontier/instance 100/250；summary 50/200；纯 count 1/5；distance 单点 5/20、256点 50/200；surface/snapshot 100/250。Placement 保留 P95<=5s、max<=20s、响应全等、一次请求固定快照、预算不重置。取证开关 P95 比<=1.10、零丢失/错身份。

## 3. C1 主验收：完成驱动的并发容量

### 3.1 固定输入与程序

- 部署 1cm、倒角 hull、真实 0/5/20 箱，沿用原并发 30k 点云序列及输入 hash；每箱数组独立运行 3 次，seed=0/1/2，不择优。
- 独立 writer 与 replica 进程、独立 install、同源 monotonic 时钟。预热 3 笔写入及各查询类型 3 次，排空后从已同步身份开始计时。
- 每次评分 120 秒。写生成器只保持一笔在途，上一笔返回后立即投递下一笔，不等待导出、接收或查询成功。评分截止不新增写入，已接纳写入及其传播进入最多 30 秒收尾窗。
- 查询按墙钟独立调度：summary 10Hz、ROI 5Hz、distance256 2Hz、surface 1Hz。查询调度不能随写入完成而暂停；记录 scheduled/send/callback/result 四类时刻，避免 coordinated omission。
- 请求不无限并发堆积：每种查询最多一笔在途；到期仍忙的计划请求记 missed_schedule，不补成突发。missed_schedule 必须具名并计入调度完整性，不能删掉或冒充成功。
- 写入压力与响应能力分开：报告每笔端到端写耗时、融合/派生/快照/hook、后台编码与接收成本。不得只计算主线程耗时或把后台排空移出资源账。

### 3.2 判门与容量输出

主验收不规定写入 Hz 下限；仍要求每个 run >=30 完成提交、每类查询 >=30 合法成功响应并覆盖完整 120 秒，否则 not_evaluated，不自动延长/补跑。30 提交是统计充分性要求，不宣称低于该数量就是产品错误；如确实低容量导致缺样，先保存结果再另行冻结采样窗。

硬门：全部已接纳写入结果与基准语义全等；无未解释丢失；收尾 30 秒内终结；各查询时延满足 §2；双向 heartbeat max<1s、零 lease 失效；传播 P95<=1s/max<=2s；资源门成立。调度没有主动漏发（missed_schedule=0），不能通过降低查询频率得到 PASS。

容量报告至少包含：评分窗提交完成数/120s、含收尾完成数及分母、服务时间 median/P95/max、每 10 秒完成数、写入占用率、输入/实际融合点数、pending 导出代数/字节及峰值、CPU 秒/提交、双进程 RSS/PSS。报告“该负载实测完成率”，不把 1/median 宣称为可持续生产吞吐或稳定性证明。

## 4. C2 固定 2Hz 压力画像：容量不足不否决

保留固定输入压力对照，但不再用其重复 9 个窗口证明同一容量不足。1cm、0/5/20 箱各一次 120 秒，seed=0，共 3 个窗口；输入与查询频率同 C1，仅写入变为每 0.5 秒一个计划到达。与 C1 分开目录、分开结论，不比较为完全同日程配对。

测试生成器最多一笔发送在途、等待队列最多一笔；队列已满的新到达记 `not_admitted_capacity`，保留帧 ID/输入 hash/到达时刻，不覆盖等待帧、不冒充已提交。此为评测端有界施压策略，不是新增生产丢帧机制。accepted 的定义为服务端实际接纳；生成器调度/发送/服务端接纳分别计数，禁止混用。

逐计划到达满足账本闭合：到达数=未接纳数+发送后具名拒绝/失败数+接纳数（各项互斥）；每个 accepted 最终有结果及传播分类。记录计划到达→发送、发送→提交、提交→READY 三段，不把生成器等待计为导出传播。

| 压力结果 | 对总验收的影响 |
|---|---|
| 完成率<2Hz、生成器有界未接纳、排队年龄>1s | `capacity_limited`，报告数值，不单独否决总 PASS |
| 查询原门/心跳/资源/传播失败、错结果、静默丢、收尾未终结 | `safety_or_responsiveness_fail`，仍否决，不以过载为由豁免 |
| 正常 CURRENT 传播等待 | 单独记录具名拒绝，不算合法成功，不删分母；按 §5 判读 |
| 样本不足或记录不完整 | `not_evaluated`，不签压力画像完成 |

没有等待队列无界增长的许可。C2 不寻找极限频率，不追加自适应扫描或多次择优；若需要实际生产 admission/coalescing 策略，另单设计。

## 5. CURRENT、合批和统计判官

- 保留 strict CURRENT：已知新 head 则不能返回旧身份。合法 `REPLICA_LAGGING` 等拒绝与实现错误分开；逐类报告 attempted/success/named_rejection/error/timeout，全部原始请求留档。
- 查询延迟同时报告所有已发请求终态及成功子集，两者均检查原 P95/max；失败不得从总样本删掉。成功样本数不足30为 not_evaluated。正常具名传播拒绝不强制零，但不允许通过“全部迅速拒绝”过门。
- 每个 C1 run 固定20次有界 CURRENT 获取检查，墙钟 t=3+6k 秒（k=0..19）；通过独立客户端，不阻塞持续查询调度。每次总期限2s，具名传播等待按50ms间隔重试，其他错误立即失败；20/20须获得合法身份，固定句柄跨代稳定（持有不超过合同2s）。计总获取耗时和尝试数，不能只计最后成功请求。无预算重置。
- 每笔已提交事实必须分类 `exact_visible/coalesced_into/timeout/export_failed`，覆盖证明保留实际 commit identity、导出覆盖成员和首次 READY 时刻；合批后中间身份不要求曾可单独请求。未送达/超2s不可从新鲜度分母消失，重复 READY 样本不能增加提交数。
- 每组独立 nearest-rank P95=index ceil(0.95*n)-1（n=30 时 index28），不先平均三次 P95。C1三次必须逐次达硬门；C2容量分类不参与 AND，正确性/响应/资源仍参与。
- 总 PASS 表示所有冻结有限样本硬门通过，不表示长期100%可靠。协议错误/错身份/静默丢失的样本容忍为0；不从3次成功推断长期故障率。判官须以错身份、全拒绝、漏提交、限容量但响应健康、无限积压、记录器慢等负/正对照证明能区分合同内外。

## 6. 执行准备、证据及修复纪律

沿用 `scripts/g3/occ_query_eval.py` 和已接入的 isolation 适配器，不新建一套判官。上述 C1/C2 是精确执行程序要求，不宣称 CLI 参数已经存在。实现者先在非评分阶段完成参数/接线行为测试并保存真实 `--help` 与完整命令，再冻结脚本 SHA 进入评分；产品源码不因验收改动。

运行器应能显式选择 completion_driven / fixed_2hz，固定箱数、seed、duration=120、drain_deadline=30、pending_limit=1 和查询频率。用受控快/慢写替身验证：C1无额外休眠且单在途；C2总计划240帧（t=0..119.5）及账本闭合；错误路径不会被容量分类掩盖；20次获取期限不会被重试重置。

Binding 前固定原矩阵输入 manifest/hash、确切产品与脚本 commit、dirty=0、双 install hash、真实 IDL、CPU/RAM/OS/Python/NumPy/ROS/FS/affinity及外部负载；锁 `/tmp/elfin-occ-backend-perf.lock` 与 OctoMap 线互斥。冷启动/JIT 单列，不能混入预热样本或免掉 cold max 门。基准与候选不混 overlay。

证据新目录：`docs/status/evidence/occ_owner_b1/occ_query_g4b_gen3_20261028/<run_id>/`。保留 T0 manifest、T1逐请求/提交/导出事件与计数、T2失败前10秒后5秒的重放输入/对象/hash、原始时延、资源峰值事件。每run8GiB/总32GiB继承；超限停止，不删失败。总判官输出0/3/4=pass/fail/not_evaluated，并单列 pressure_capacity_status。

| 边界 | T1 / 失败触发 | T2 内容及检查 |
|---|---|---|
| 调度→写入→commit | 到达/发送/接纳/完成ID和时刻；账不闭合、错误或超时触发 | 前10秒后5秒原点云输入、更新序列、参数、基准/候选输出；离线逐提交事实对照 |
| commit→export→READY | commit与覆盖代身份、encode/publish/receive/READY、队列；新鲜度/身份失败触发 | 同窗 manifest、引用对象及hash、head/control；先验文件完整性后重建验证 |
| CURRENT/pinned→query | 每请求身份、状态、时延、记录join；错身份/超时/全拒绝触发 | 同窗请求/响应、token生命周期、对应不可变快照；原请求按基准oracle逐字段回放 |
| 两进程资源/心跳 | PID、RSS/PSS、派生计费、分配/释放峰值事件、heartbeat；超门触发 | 同窗资源账、对象/映射/引用清单、进程状态，核对去重及峰值，不从采样0推断分配0 |

各行使用 JSON/JSONL 加现有二进制对象，合计受上述8GiB/32GiB限制；沿用原重放入口，实际可运行命令须在 Binding 前写入 manifest 并用一份故障 fixture 验证。未建好的入口不能标 capture_complete；取证健康须零 dropped/failed、join完整，T1开销沿用<=1.10门。不增加相机RGBD/TF全链采集。保留数据完成hash与flush核验后才允许清理进程；不以压缩或配额删除重放必需对象。

代表组失败先保存并停全矩阵；不在评分中修改产品或改阈值。既有 A 门不因这次节拍调整重跑；若 B 暴露产品缺陷，回 A 后继修复，再绑定受影响 B 门。

## 7. 已批准的协议、资源归属与交接

用户于2026-10-28 16:57明确批准 catch-up 协议及资源门归属裁定。catch-up规格 `OCC_SEQ_GAP_CATCHUP_SPEC_20261028.md` 正式准入，单代原子/失败resync/严格CURRENT保持。不存在继续等待同一项策略批准的前置；下列资源分工是本次授权的落地定义，不是宣称资源已PASS。

| 责任 | 判定与交付 |
|---|---|
| G4-A | 已完成产品实现、已关闭门及现有分项资源账作为交付依据；不为资源全矩阵重开A，也不追签原ledger为统一512MiB PASS |
| G4-B（eng/claude/glm-5.3） | 承担完整部署资源补证、统一预算核算和全部B矩阵；先验收仪表、再资源代表组、最后完整矩阵 |
| reviews | 核对输入/判官/资源归因及最终结果；不以批准协议代替事实验收，不将采样0认定为分配峰值0 |

资源口径保持原协议§4：合计RSS<=2GiB，派生唯一物理资源<=512MiB，init/reset<=30s。512MiB覆盖writer新增派生缓存、receiver解码/索引、共享文件/映射（按对象或inode去重）、双方队列、staging、resync副本及在途临时buffer；主OCC本体只计RSS。必须分项给出计费依据/生命周期/去重键，不能把逻辑registry预算直接当物理账，也不能简单将磁盘/RSS/PSS相加。不能准确覆盖某项时标unknown/not_evaluated，不默认0。

B准备阶段用已知大小的分配/共享/释放fixture验证资源账：同一对象共享不双计，独立副本分别计，短命staging/resync计入峰值；超512MiB负对照必须拒绝PASS。实际运行保留双PID RSS/PSS及high-water、至少1Hz历史、分配/映射/staging事件峰值。先复用B首批1cm倒角0/20箱代表输入、3预热+30提交的计时窗核算（包含启动、变更导出、接收和排空），并补一次受控FULL resync资源窗口；不能仅用旧0.02m/5箱账外推。首批即属B证据，不追加独立优化战役；代表组FAIL或缺必需字段则停止全矩阵。通过后资源监测覆盖B全部组及原故障/清理边界，原2GiB/512MiB门不变。RGBD束/TF完整链采集仍属任务外，OCC提交/传播/查询边界不可缺。

当前可以准备B绑定：锁定源码 `0da4086bce23be49ec8eeb934a15fb6829803abe`、本批准计划精确SHA，并登记独立install哈希、真实IDL、完整矩阵与环境清单。准备/仪表测试不是评分，必要的脚手架修改须独立提交且不得改受验产品。reviews核对这些机械准入项后追加Binding、置dispatch_ready=yes；不需要再次请用户批准同一协议或资源分工。当前未取得install证据，故不虚写已Binding。G4-B gen2线程追加generation3事件，原历史元数据/Result不改。生产b33不替换，不启动Gazebo/MoveIt/装箱，不宣称n=3通过。
