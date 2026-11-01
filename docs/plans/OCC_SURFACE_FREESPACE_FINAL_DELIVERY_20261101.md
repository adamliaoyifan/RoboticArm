# OctoMap Surface / free-space 最终交付合同

状态：用户2026-11-02已选择先落实统一目标再最终验收，本修订正式绑定该路线。产品基线2ec2393122481d4ad8467c85e9971e13bc0f820c；S2 gen10已关闭且独立复核通过，不再重开。旧S3 gen2由本合同S3 gen3替代，不先跑旧1Hz查询评分。原计划及历史失败只读保留。本合同采用2.5Hz更新作为≥2Hz目标的验证裕量、2Hz完整free-space查询；正式矩阵见下。

## 1. 唯一产品目标与范围

在当前x86、Release、ROS2 Humble、OctoMap1.9.8、2cm、discretize=false、powersave、不绑核、客户端4执行线程环境，30k有效输入、0/5/20箱，持续同时完成地图更新与free-space查询。更新正式代表目标2.5Hz（证明高于2Hz），查询2Hz。100k/5cm为原O5/O6能力矩阵，不凭2cm30k成绩宣称100k也达到2.5Hz。

交付包含：稳定快照接口、完整Surface+ROI读链、ROS2 placement surface_map_2d适配、完整候选体积的known-free验证、可重复评测命令和逐门报告。仅几何放置候选及现有policy消费测试，不启动Gazebo、机械臂、运动规划或生产迁移，不宣称机械臂可达/可执行。兼容旧接口，unknown/free/occupied不混淆。

## 2. 可用空闲与放置语义

- unknown是未观测，free是真实接受射线确认的空闲，occupied是占有；不能凭最高占用面之上为空或log_odds=0认定unknown/free。
- Surface提供支持高度和列摘要；候选物体必须对完整朝向体积做同租约ROI验证，并按ULD hull裁剪。接触支持面与内部体积分别判断，正体积碰撞或unknown内部不标KNOWN_FREE。
- 输出至少包含状态OK/NO_KNOWN_FREE/NO_DATA/STALE/PRESSURE/错误、完整快照身份、年龄、几何候选与验证范围。不用空数组掩盖失败。NO_KNOWN_FREE是合法语义结果，不是服务失败，更不能为了出位置把unknown填free。
- floor_prior仅表示地板存在，不能作为已观测空箱证明。已有placement政策允许的prior候选与严格known-free候选显式区分；不擅自改变既有policy。
- 适配轴序、坐标、floor相对高度、分辨率、confidence/source、epoch/reset、map/observed_revision、geometry/config/clock身份。surface_map_2d沿现有接口；epoch重置不可被单调map_revision检查误判或混图。
- 候选质量按现有placement配置（支持率/支持容差/顶空/间隙/朝向）冻结并报告。性能夹具统一两种探测箱0.30×0.20×0.20m、0.50×0.30×0.30m，yaw=0/90度；这些是离线查询输入，不是线上感知GT。每次查询两种尺寸交替、请求一种尺寸的所有几何候选；测量含候选生成与体积检查，不能只计Surface RPC。

## 3. 锁、快照与生命周期一次设计清楚

- 写入保持单worker；store写锁覆盖一致的概率/树维护及快照发布边界。
- 查询需要仓库句柄锁、缓存首次构建锁；数据读取用不可变快照的共享所有权。不得让整条查询与积分共用一个长持有的排他锁。
- 锁顺序表和每把锁保护的状态写入最近代码文档；不持仓库锁等待store，不持锁做ROS序列化/网络等待。reset/TTL/release与已开始读的线性化规则固定。
- 已有租约在store写锁被测试屏障持有1s时，应在200ms内完成ROI/Surface且结束时写锁仍持有；暂停是行为hook，不用于计时成绩。首次cache构建并发只构建一次，结果oracle一致。
- 新租约/WaitSnapshot使用已有有界等待合同，满8请求立即明确压力，恢复≤2s；所有错误路径finally释放。新目标正常流程不得靠无限重试保证成功。
- 资源表列明每类对象谁分配/共享/释放，尤其Snapshot与SurfaceCache对象、Surface返回buffer可有不同最后所有者。live/同刻peak/累计flow分开；以同刻read_view+cache峰计512MiB。所有者释放才扣，重复共享不加；不以RSS代替所有权、不以累计flow代替peak。

## 4. 固定通过门（统一目标合同，不是既有成绩）

| 门 | 工作量 | 通过条件 |
|---|---|---|
| F 语义 | 空树、已观测空箱、满箱、部分unknown、遮挡、悬空/夹层、边界、两版本、reset；每类10固定变体，两种箱体尺寸与两yaw | 与独立逐体素/hull oracle一致；假free/越界/碰撞候选0；明确存在离散可行位置的正例至少返回1个；unknown/无解输出合法NO_KNOWN_FREE；不要求对未观测区域输出位置 |
| C 并发/所有权 | 同revision共享、不同revision、仅外部buffer持有、8/9句柄、TTL、reset、异常、缓存并发、写锁屏障；每路径重复30次 | 内容身份不混；无死锁；屏障读门通过；释放后归属回基线；顺序复用不抬live/peak，并发才抬峰；注入超门峰能被正式判官拒绝 |
| P 正常2.5/2Hz | 0/5/20箱，各3×120s；独立绝对时钟发布和查询；每窗300输入/240完整查询 | 每ID恰一成功终态；零拒绝/unknown/重复处理；输入端到端P95≤350ms/max≤500ms；完整query P95≤250ms/max≤500ms；9窗AND；无积压增长，窗结束后0.5s内该窗任务闭合；正常快照年龄≤1s；health间隔<1s |
| H 调度裕量 | 同P采集发送/查询计划时刻、实际调用开始 | P95迟到≤20ms/max≤50ms，短周期抖动不跨窗平均；持续频率按计划发出数和每请求截止核算；不能将订阅线程数当吞吐 |
| R 过载恢复 | 原20箱10Hz×60s600输入，query提高2Hz；30s固定排空；随后2.5Hz×30s75输入、2Hz60查询，同节点同地图 | 过载所有ID终态闭合，允许明确拒绝，至少真实排队/拒绝一次；30s截止在途0；恢复首Wait≤2s且年龄≤2s，其余查询正常门；恢复75输入成功。不得重启清空代替恢复 |
| L 稳定/泄漏 | 20箱2.5Hz更新+2Hz完整query连续120min；前10min预热；另1000轮acquire/read/release和100轮reset/异常循环 | 运行无崩溃/死锁、任务与句柄无增长；RSS≤2GiB、归属组合峰≤512MiB；末10min RSS中位相对预热后首10min≤5%；固定地图每轮最后所有者释放回同一基线。有限时长证明，不宣称数学上永不泄漏 |
| O8 概率 | 沿原180s×3、另30帧饱和、no-input/遮挡各10s×3，保留duplicate单列 | 原概率/节点/RSS规则；全区域483516格（固定fixture）由region计算，不依赖自报cells；磁盘原始state/log_odds复算；无输入时新鲜度不前移 |
| O9 采集资格 | 原三操作30对AB/BA | P95 ON/OFF≤1.10、30/30完整trace、失败能重新执行产品；若新增候选链常开采集，另30对完整查询链ON/OFF同门，固定原输入 |

P门输入端到端从实际publish开始至完成终态可观测；query从客户端计划发起后的实际调用起点至完整候选结果及租约释放，另报调度迟到。每个500ms/400ms周期都保留deadline，不以总吞吐掩盖单次超时。预热与就绪先完成（首次接受输入/TF/服务和已知快照），起点后所有错误进入分母，启动失败另报，不用就绪门吞掉计分失败。无输入/遮挡的STALE是另类合法语义，不混入正常有输入窗成功率。

所有延迟P95=排序第ceil(.95n)个，max保留；有限样本100%满足各自状态/截止门，不能跨组平均。NaN/Inf/bool/负/缺字段/缺样一律不可通过。资源峰值缺覆盖NOT_EVALUATED。

原O5 4组、O6 96组全部保留为独立能力矩阵，按旧30k500/1000ms、100k2000/5000ms及每操作100/250ms门评分；不能用它代签新P门，也不能把新P门结果抹去旧失败。完整验收报告分别给目标交付和原广度矩阵状态，矩阵未评不得写完整矩阵通过。

## 5. 一次性验收工具审计，不逐轮补一个字段

实现者在长窗前提交完整acceptance matrix，每条门至少健康+独立负例；统一数据校验层覆盖所有数值/身份/内容/计数/截止字段，不能各入口自己写不同规则。

必拒负例：缺1输入/查询/终态，重复ID/终态，未知ID/全拒绝，错epoch/map/observed/geometry/config/clock，1格充当全区，少/重复/乱序页面，错误state/log_odds，伪SHA/缺文件/路径逃逸，NaN/Inf/bool/负/缺值，零预期样本，短时窗，超门但当前值低，输出false标志伪造，查询阻塞发送，资源只剩外部读者，首次/更新后cache与hit混用，过载恢复重启，timeout后追认成功。

验证必须跨实际adapter→artifact→judge；产品路径用真实触发或屏障，原始输入/边界/所有权证据先落盘。判官需要拒错与接健康，独立oracle不得调用被测同一算法或重复公式作为唯一证明。完整清单先审一次再长窗；后续只因违反冻结合同的可复现缺陷返工，不新增无关验收项。

## 6. 收口顺序与责任

A 合同冻结：本修订提交SHA即plan_revision，固定2.5/2Hz、查询链范围、fixture/config、placement接口与计时起止。具体最终输入SHA由实现任务按下述确定性程序生成并冻结。S2 gen10结果及Claim不改写。
B 实现闭环：同一eng owner保留已通过的gen10资源3路径回归，完成placement桥接/体积验证、锁规则、全部边界测试。评审一次覆盖F/C/判官负例和命令；任何未实现项不得先宣称完成。
C 非计分预演：同正式命令/adapter缩短次数验证全部接线。≤2h不含锁等待。输出无占位COMMANDS.sh，包含build→tests→rehearsal→freeze→O9→P/R/L/O8→O5/O6→judge→replay，每阶段可选且失败不自动继续依赖阶段。
D 冻结计分：同一candidate/输入/环境；串行持perf lock；先O9和正确性，再目标P/R/L与原矩阵。旧O5失败阻断旧O6/O7规则保留；新增目标计分与旧矩阵显式分报告，不借短测代签。正式总预算8h不含等锁，先预算估算，耗尽not_evaluated。修代码即结束候选保留失败，重跑受影响门，禁止拼版本。
E 一次最终报告：工程交付、目标2.5/2Hz、原矩阵、新旧恢复合同、历史1cm失败/未覆盖分别列pass/fail/not_evaluated；只有全部目标门通过才能称“可稳定提供known-free候选”。无论产品通过或失败，本轮执行任务都必须以真实结果关闭，不循环改测直到看起来全绿。

Owner沿eng/claude/glm-5.3；reviews只复核合同及证据，不把测试扔给另一角色。任务固定只有两项：实现与非计分资格、冻结最终验收；返工回原实现项，避免代次扩张造成目标漂移。后续重开必须列明违反哪条冻结合同及独立复现，不能以希望更完美为由追加要求。

## 7. 当前事实（2026-11-01）

候选702ed36e：未完成正式O5–O8评分，未证明2Hz query；旧O7 query仅1Hz。O9比1.0113/0.9958/0.9970只证明采集扰动；不表示绝对延迟/吞吐。最新独立123测试通过/11未复跑，O8磁盘健康样本可复算；共享cache对象重复计数、外部48000B buffer漏计、1格合法内容冒充483516格region仍失败，gen10已派发。placement转换尚未交付，不能说已稳定输出可放置位置。

历史O7 integrate P95 95–134ms、Surface中位101–145ms，9窗Surface失败；600过载仅523回执77unknown；恢复第一立即查询STALE。历史O6 1cm一组517.298ms超500ms，最大RSS535548KiB约523MiB。这些是旧版本成绩，均不给当前候选代签。


## 8. 决策绑定与可执行派工（2026-11-02）

| 子任务 | Owner | 基线 | 依赖 | dispatch_ready | 关闭条件 |
|---|---|---|---|---|---|
| OCC-FREESPACE-IMPLEMENT gen1 | eng/claude/glm-5.3 | 2ec2393122481d4ad8467c85e9971e13bc0f820c | OCTOMAP-2CM-REPAIR-S2 gen10 | yes | F/C全行为、接口兼容/真实消费者、完整判官正负例矩阵、精确命令、非计分接线、冻结及O9合格；清楚列未评性能 |
| OCTOMAP-2CM-REPAIR-S3 gen3 | eng/claude/glm-5.3 | 2ec23931为来源；领取前绑定实现候选全SHA | OCC-FREESPACE-IMPLEMENT gen1 | no | 同一冻结候选完成正式目标门和适用矩阵/独立复算/失败重放；按实测pass/fail/not_evaluated关闭，工程执行完成与产品通过分列 |

两任务parent均为OCC-QUERY-INDEX-20261025。实现任务在主workspace agent_start领取，从基线建独立worktree，避免改写g6已冻结树；主docs/agents统一协调。证据新根为docs/status/evidence/occ_freespace_final_20261102/，下设implementation、candidate_<SHA>、scoring_<runid>。继承原T0/T1常开、T2失败前后各5秒及完整地图前缀/生成manifest；512MiB/例、2GiB/run、8GiB总量，不能截断必要重放内容。实际placement桥接仅在独立测试节点/进程验证，不重配正在运行的生产节点。

### 输入与测量程序

1. P/H/R/L统一使用原O7 2cm chamfer几何、30k、outside=.5/cross=0、0/5/20箱及原生成器seed规则；几何/config从docs/status/evidence/occ_o6_o7_20261029/run01/fixtures/geometry_chamfer.yaml与config_chamfer_r0p02.yaml读取，记录SHA。不得改变点数/射线或裁剪难帧。L为20箱，周期输入需按原生成器seed序列固定，不能改用重复ID去重降低融合量。发送与query独立线程、client4执行线程，hook关闭。查阅docs/architecture/placement.md、frame_contracts.md、container_geometry.md及real_scenario.md，按现有ROS2 placement_solver消费契约适配，算法与ROS传输模块分离。
2. F为9类场景×10固定变体×2箱体尺寸×2yaw，共360例（两版本/reset每例含完整前后序列），manifest含固定case_id、seed=0..9、所有观测输入及独立oracle预期。空树返回NO_DATA；已有观测但无已知可行体积返回NO_KNOWN_FREE；不能把360例全返回无解当通过。已观测空箱和有空间正例每个尺寸/yaw必须有至少一可行候选并匹配独立枚举集合；所有返回候选均做体素/hull校验。F参考数据不进入线上推理。
3. P每窗就绪后独立起点t0，输入i=0..299计划t0+i/2.5，query j=0..239计划t0+j/2；各请求自调用开始500ms截止，调度迟到另按H门。正常NO_KNOWN_FREE在确实无已知空闲的oracle中是合法成功；NO_DATA/STALE/PRESSURE和服务错误不能算正常成功。快照新鲜度从观测时间计算，未接受输入不得推进。请求交替两种箱尺寸，既有支持policy参数冻结（yaw0/90、support_tol=.05m、min_support_ratio=.6、clearance_margin=.03m、same_layer_clearance=.05m）；新2cm地图轴序/尺寸不采用旧5cm硬编码。
4. R过载600输入/120完整查询，固定30s排空期继续60查询；随后同进程/epoch地图75输入/60查询恢复。过载允许压力/陈旧/有界timeout但每请求有明确结果和终态，不能无回执或崩溃；排空按真实完成时刻判30s截止，不能截止后轮询的状态代签。恢复首请求绑定第一恢复ID，2s内成功；其余59请求按P门。启动就绪与已有地图暖机均在计分前记录，不计分失败不得被悄悄重试覆盖。
5. L共120min内18000输入/14400查询，前10min为预热（仍记录所有错误），后110min的16500输入/13200查询逐请求沿P/H门；按独立120s窗口及全程汇总，全部AND。RSS每1s采样，至少每个10min桶600个计划样本，缺样不通过；归属峰从分配/释放生命周期记录。1000轮读释放循环每轮最后所有者放开并检查字节回当前地图基线；100轮reset/异常循环回空基线，错误不得留下租约。L和原O8分别输出，不互代。
6. C的9类行为各30次；8/9句柄和8/9等待均真实触发，压力响应≤500ms；写锁屏障按第3节200ms门，Wait截止按原合同。有限集合内100%满足；这些门先离线/core/live小行为通过，才进入计时资格。
7. 查询的所有候选指既定离散枚举格点、两yaw、完整箱体和现有policy的可行结果；不能用top-N截断来降低性能工作量。完整链从query调用开始至结果全部物化且lease释放，所有分页/序列化/候选计算/ROI检查均计入。若完整枚举达不到门，报告性能失败，不换成只返回Surface。每次只请求一种箱尺寸，按序号交替。
8. 所有产品当前值、峰值、终态、计时、原始health/RSS、probability内容、候选输出和canonical hash写入同一schema。判官需独立复算内容/身份/格数及所有门；错误不能过滤分母。whole-chain捕获ON/OFF的30对为本合同必测（不是可选），原三操作O9仍测。正常采集用的所有常开事件纳入ON；输入/配置/输出完整性不因OFF改变。

### 命令交付与验收顺序

实现owner首先固化COMMANDS.sh，所有可执行文件绝对路径、参数、ROS/RMW配置、seed/fixture/input manifest以及安装库SHA实填；不得依赖/tmp旧install或留下TODO。脚本需显式独立入口build/tests/rehearsal/freeze/o9/score/judge/replay，score默认关闭，待新S3绑定后打开。缺就绪/缺依赖/错SHA/dirty/错误候选必须退出非零且不得开始评分。正式执行程序：

1. 实现阶段：构建→F/C及判官全负例→短接线（P每窗2输入/2query、R同节点短过载/排空/恢复、L短循环）→固定候选→原三操作及whole-query O9。短样本不得满足正式判官，接线资格单列。就绪失败计次数/墙钟，不消耗正式样本。
2. 最终阶段：冻结一致性与F/C回归→已绑定O9资格→P/H→R→L→O8→原O5→O6。P不通过，停止依赖正常频率的R/L计分，但保留安全可执行的F/C/O8及原能力矩阵，报告目标失败；O5失败停止旧O6，旧1Hz O7不单独再跑，新P/R替代其代表目标覆盖，不能声称旧未跑窗通过。
3. 正式总预算8h不含perf lock等待，优先完成新目标门；原O6矩阵耗尽预算标not_evaluated，不影响如实报告已完成的新目标成绩，但不得称全部矩阵验收通过。预算预估在实现交付时写出，不能删掉120min长稳来给96组让时间。
4. 冻结后不得改产品/工具/配置修当前成绩；需修复则该候选评分结束，失败持久化，新候选按影响门重新冻结/验证；不为获取全绿无限重试。任何新要求必须新合同，不以review偏好改变门。判官自身不区分健康/坏输入属于当前合同缺陷，必须修复。
5. 实现完成后review一次检查验收矩阵覆盖和启动资格，绑定S3实际候选/命令/manifest才设dispatch_ready=yes。最终执行owner必须输出完整result matrix、原始证据SHA清单、实际执行命令、失败重放和未评原因。无论目标通过与否，都以真实工程执行结果收口。

## 9. 当前基线更新（2026-11-02）

S2 gen10候选2ec23931三条补正已独立复核通过（040c294d），151 passed/12未复跑；缓存共享72→72、外部48000B保持归属且最后释放归零、1格全区反例被拒，O8磁盘复算与O9 trace30/30有效。第7节是此前历史记录，不再作为未修缺陷派工。本合同不会再开S2 gen11。旧S3 gen2 superseded，由本合同S3 gen3接续；用户选定先统一目标再最终验收，故不先启动旧1Hz评分。
