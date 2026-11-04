# 双后端比较 G3：数据、统计、判官和最终验收

- date: 2026-11-04
- status: approved
- owner: eng/claude/glm-5.3
- parent: OCC-QUERY-INDEX-20261025
- subtask: OCC-BACKEND-COMPARISON
- generation: 3
- plan: docs/plans/OCC_DUAL_BACKEND_COMPARISON_G3_20261104.md

本附件是本轮一次性完整口径。对合同内具体缺陷返修；不因新偏好追加场景、统计方法或证明责任。历史产品验收不重开。比较工具/接口适配可由owner实现，产品算法不得在评分期间修改。所有未特别允许的缺失字段，不以0、空数组、当前时间或默认成功填充。

## 1. 版本与执行边界

G产品=a4ac889e，O产品=cbdd5364，共同消费者C=cbdd5364。工具来源18fa18ce/9cc59da1与最终工具T分别列明；9cc59da1不替代O的评分产品身份。记录各完整40位SHA，产品目录git blob及实际加载文件SHA256、二进制动态依赖、Python/NumPy/ROS/OctoMap版本、编译类型与flags、环境与配置。普通依赖同版本，不以整个两个分支的文件全集相同为要求。

消费者、消息兼容和适配C'可修改；必须将哪些文件属于G/O、C'、T及输入分别列入manifest，不可把算法改变藏进adapter。Adapter只能转换单位/轴序/身份、桥接服务/分页/合法状态、连接已有共同验证与真实回调；不能凭空补free、支持面、ledger、传感器时刻或validity。某必需能力没有真实证据来源则BLOCKED，不能补默认值过关。

每次建图/调用加载的模块必须来自显式G/O/C'前缀。各前缀独立构建；禁止主workspace overlay污染。工具独立提交；不提交build/install/原始证据到Git，证据通过manifest绑定并归档。dirty允许仅新证据输出，产品/C'/T/配置tracked和untracked变动均须为0后方可评分。

## 2. 输入数据的确定性定义

### 2.1 目录和选择规则

以O run03的FREEZE_INTEG输入清单和现有O5比较夹具作为唯一来源索引。为每个(hull,box_count,point_count)列出候选，要求几何hash可验证、box_count为**每帧**相同标签、原点/采集时刻/TF齐全；按完整source manifest SHA256字典序选第一份合格数据，不按运行速度选。完整源路径+hash保存在INPUTS.json，不只保存目录名。

每组合生成固定33帧规范回放：按源frame序排序，不足33则循环既有帧，记录source_frame_id和repeat_index，不宣称33份独立实测。B3用同一序列循环至300帧、每次新observation_id与回放采集时刻。静态夹具TF按源对应姿态复制；动态录制必须连同其对应TF一起回放，不能只改stamp而使用最新TF。

缺少某组合时允许现有场景点云生成器以seed=20261104生成**整个组合**并冻结generator SHA/参数/原始帧；不复制点以凑100k，不将30k与100k混为同负载。没有可用生成器/几何原点时该组合NOT_EVALUATED，其他组合继续；不得改成不同box_count。GT/箱模型可用于生成离线传感器夹具与独立判官，必须与在线输入分目录隔离，不能直接作为占用先验。

### 2.2 字段与工作量

每帧字段：`dataset_id, source_frame_id, observation_id, repeat_index, acquisition_stamp_ns, clock_domain, sensor_frame, map_frame, T_map_sensor[4,4], sensor_origin_map[3], raw_points_path/hash/dtype/shape, geometry_hash, config_hash, box_count, requested_point_count`。点数组little-endian float64 Nx3、米，行序固定；非有限原始点可保存，但规范有效点必须finite。raw数、Z过滤数、TF后数、crop后hit端点数、ray端点数分别记录，不能用一个point_count表示所有阶段。

FULL_RAY：光学Z范围统一[0.52,6.0]m（含端点），使用同采集时刻TF，两个后端收到同一规范payload字节hash。先保留所有有效ray及原点，hull外端点用于穿域free射线，不能为了裁剪hit把所有外部射线扔掉。30k/100k指Z/finite过滤后、TF及hull裁剪前的有效端点数，每帧必须精确等于30000/100000。输出ray支持情况和实际处理计数；关闭G限额、O discretize=false，重复端点原样保留，后端原生去重如实报告，不要求内部遍历计数一样。

DEPLOY：原始帧相同，参数取G/O各自固定产品部署配置的实际解析值，全部展开为CONFIG.json；不得留“默认配置”四字。记录不同过滤/限额/去重与保留计数，结果标为部署差异，不计算等工作量算法加速比。若缺部署启动配置，采用固定产品构造默认值并标`profile_origin=constructor_defaults`，不能宣称现场部署效果。

hull、分辨率2cm、全图ROI、探测箱及consumer算法两侧相同；几何以descriptor/hash为准，不凭字符串cuboid/chamfer判同。几何坐标精度1e-9m仅用于序列化往返检查，不用作跨后端surface等值阈值。

### 2.3 时钟

性能所有事件用同主机`CLOCK_MONOTONIC`整型ns，ROS/sensor acquisition stamp单列，不直接与monotonic相减。回放epoch起点生成明确ROS-time↔mono仿射映射，并记录源时间和映射；仿真暂停/时钟跳变则run无效，不用latest TF救场。假时钟只用于离线兼容反例，不用于真实性能计时。

B1/B2是完成驱动回放：开始每帧时生成该次回放采集时刻，目标输入与之前所有历史帧明确绑定，避免长串行运行把数据人为变陈旧。B3 t0为就绪后的绝对起点：input i=0..299目标t0+i*400000000ns；query j=0..239目标t0+j*500000000ns，起点包含、120s端点不包含。两侧拥有相同相对时间表，不要求两个串行run绝对时间相同。

## 3. 标准事件、身份和终态

### 3.1 每条事件公共字段

`schema_version=1, run_id, profile, matrix, group_id, backend, block_id, event_seq, event_kind, mono_ns, pid, tid, operation_id, input_id, request_id, stage, state, reason, payload_hash, snapshot_identity`。不适用字段写null并在schema列明；时间/计数/bytes非负整数，bool不能充当数字；枚举未知为INVALID_DATA，不吞掉。字段存在但空身份等同缺失。每进程event_seq严格递增，落盘loss_count必须0；不同进程不要求共享序号。

snapshot身份包含backend、epoch、map_revision、observed_revision（若本地没有该字段，保存可验证的观测覆盖集合而非伪造数字）、geometry/config/clock、last_integrated_stamp、handle/lease/page游标。原生没有digest允许标unsupported，但必须有稳定原生快照ID+观测覆盖证据；缺这两者则该完整查询能力BLOCKED。身份在各后端内闭合，不要求G与O的revision/digest相同。

### 3.2 请求账本与截止

预生成全部计划ID与expected_count。每个ID包括planned_ns、actual_send_ns、server_receipt_ns（不可观测则null）、terminal_ns、deadline_ns、native_status、normalized_terminal和result artifact hash。异步发布返回不能当INTEGRATED；必须有实际入图回执/只读可见证据。

完整更新、完整query、placement链都只有一次30s绝对deadline，从本次实际send/链开始计；分页、poll、重试不能延长。B1/B2正常评分不做透明重试；通信修复请求保留原ID+attempt_id，原请求超时仍是TIMEOUT。B3请求在120s内发出，排空到t0+150s；terminal有效截止=min(各自send+30s,t0+150s)。planned但未发为NOT_SENT，不虚构actual_send。

归一终态：`OK, NO_KNOWN_FREE, NO_DATA, STALE, PRESSURE_REJECTED, INVALIDATED, INVALID_REQUEST, RESOURCE_LIMIT, ERROR, TIMEOUT, NOT_SENT`，保留原生reason。结果必须恰一个终态；deadline后到达另写LATE_RESPONSE事件，不把TIMEOUT改OK，不双计。reset反例允许对应INVALIDATED等，但B1–B3无主动reset，不把这些当正常成功。

`valid_completion`：更新仅实际INTEGRATED→OK；查询为同租约完整分页/身份/释放成功的OK；placement为通过几何及身份校验的OK或合法NO_KNOWN_FREE。无地图/错误/过期/拒绝不能加入valid_completion。NO_KNOWN_FREE不需有候选，但应有本次真实求解/体积验证或合法穷尽证据，不能硬编码返回它。

## 4. 每个性能数的精确定义

| metric | 起点→终点 | 必需条件 |
|---|---|---|
| shared_preprocess_ns | 原始帧准备完毕→规范端点/TF/原点产物就绪 | 同源只算一次，单列；两侧总链报告含此项但不重复计 |
| send_lateness_ns | max(0,actual_send-planned) | B3逐请求记录；负偏差另报early_send，不当低延迟收益 |
| update_visible_ns | 输入实际发送→独立读者首次验证含该输入的地图可读 | coalesced必须证明输入被目标revision覆盖；只看最新stamp不足；不可观测则该指标NOT_EVALUATED |
| integrate_ns | 后端实际融合入口→事实提交/回执 | 有原生打点才报告；否则null，不用visible耗时替代 |
| full_query_ns | acquire开始→全部Surface+全hull ROI页内容到达校验且release完成 | 包含等待/分页/解码/转换/释放；借最新缓存不能漏计，失败release使完整query失败 |
| consumer_wait_ns | 已确认目标图可读→真实使用方接纳目标身份 | 必须由真实回调/消费证据确认，publish返回不算接纳 |
| placement_rpc_ns | 客户端发服务请求→完整响应到达 | 两侧同C/C'；记录实际使用快照而非仅请求声明 |
| solver_ns / volume_check_ns | 对应真实函数入口→返回 | 不可观测写null；嵌套/重叠不能相加冒充总耗时 |
| placement_e2e_ns | 本帧原始输入就绪→合法选位/合法无解响应到达 | B2每次先更新；包含预处理、地图等待、消费和服务；不加stage中位数拼总时延 |
| snapshot_age_ns | 查询开始/响应时刻映射为ROS时刻减实际last_integrated_acquisition | 两时点都报；只改metadata不刷新观测时刻；负数为时钟/身份错误 |

完整查询的ROI覆盖整个hull的2cm栅格，分页size固定4096 cells；若原生页上限更小则按共同最小上限冻结并双方一致。逐页必须同handle/epoch/地图身份，cursor不重复、无漏页/重复cell、最终cell集合等于该descriptor下预期域；容器外cell另标domain_outside，不混进unknown。

B1每次先更新，再一次完整query；两个计时串行但分别定义。B2可以复用本次同一已读快照给选位，不得复用上一请求句柄或把重复acquire藏在计时外；实际调用次数入账。所有adapter/IPC成本包含在系统端到端数，纯后端内核成本只在真实可观测时另报。

B3 query在实际开始时绑定该后端已经确认INTEGRATED的最大连续input前缀；若有coalescing保存成员覆盖证明。实际返回必须覆盖这个前缀或明确拒绝，不能为追逐未来地图无期限等待。使用方时效沿C/C'既有规则；额外报告age≤1s比例，但本轮不新增“全部≤1s”产品SLA。

## 5. 样本数、统计和比较规则

### 5.1 样本集合

B1 24组×每后端2块×每块30=每后端1440计划更新和1440完整query；另每块3预热不评分。B2 12组×60=每后端720计划选位链（探测箱每种30）；B3 6组×(300输入+240query)=每后端1800输入/1440query。score启动前生成EXPECTED.json，按组/块/类型核对，不按事后收到的事件数倒推期望。

只排除明确标注的预热；GC、首个计分调用、慢样本、超时、失败均不得剔除。每个层次同时报告N_expected/N_sent/N_terminal/N_valid/N_error/N_timeout/N_not_sent/N_late，计数可以从账本独立重算。计时数据缺字段不计作0ms；该指标覆盖不足单列。

### 5.2 公式

按backend/profile/group/operation分组，两个块分别报，再合并等权原始请求（不是先平均块P95）。成功延迟仅valid_completion且计时边界齐全的集合；终态响应耗时集合包含所有有真实响应的终态，明确命名，不能拿快速拒绝改善成功延迟。

- median：排序后奇数取中间，偶数取中间两数算术均值；P95：nearest-rank `x[ceil(.95*n)-1]`，n=30取第29，n=60取第57，n=240取第228；max保留。至少30个valid完成才给成功P95；B2两种探测箱分别至少30，合并60只作附表。
- 超时/NOT_SENT属于右删失/无完成样本，不给它们伪造30s“成功延迟”。报告每个deadline、已观测等待下界和超时比例；不能在成功率不同时只看成功P95宣称优胜。
- 错误率=N_error/N_expected；超时率=N_timeout/N_expected；有效完成率=N_valid/N_expected。legal NO_KNOWN_FREE另外报告count/N_expected、有效候选count/N_expected，不把有效完成率当成功选位率。
- B3发起频率=N_sent/120s；有效完成吞吐=N_valid_terminal_in_window/120s；排空完成另列，不借150s平均宣称120s目标吞吐。积压Q(t)=sent且未终态数，每1s及窗尾/排空尾采样；输出max、Q(120s)、Q(150s)、真实末终态减120s。发送负载资格固定为全部300/240实际发出、每流send_lateness P95≤20ms且max≤50ms；超过则标load_delivery_invalid，保留原始数据但不得声称已测到目标开放环负载，不把客户端迟发直接归为后端变慢。这是发送器资格，不是新增后端时延SLA。维持目标吞吐还需N_valid_terminal_in_window分别为300/240（迟至排空才完成不算窗内吞吐），deadline全部满足；仅排空Q(150)=0不足以宣称维持频率。
- 时延比固定 `ratio_G_over_O=stat(G)/stat(O)`，>1表示G更慢。只在双方同组同profile、各自资格合格且有效完成样本足量时计算；DEPLOY比值标“部署配置系统比”，不是算法加速比。分母≤0或非有限直接INVALID_DATA。不同箱型、hull、点数和profile不合并成一个总加速比。
- 不报告显著性/长期可靠性或置信度优胜；现有每组2块不是充分独立重复实验。表中可描述本次观测差异。若一方快速失败较多，标`ranking_not_supported`，仍展示绝对值及完成率，不强判赢家。

每个请求原始ns保存整数；最终ms最多3位小数仅展示，判门用未四舍五入原值，1.10按有理数比较`10*num<=11*den`。不通过舍入把1.10004变通过。

### 5.3 资格与有限可靠性

每个后端的代表资格组固定FULL_RAY/chamfer/20箱/30k，用两个相同33帧进程块（即各3预热+30评分），严格同样顺序/GC/trace/profile。更新visible及full_query两个P95的max/min均≤1.10，两个块30/30 valid才具备时间可比较资格；G/G和O/O都合格才做时间优劣解释。失败只将本run时间比较NOT_EVALUATED，正确性/分配/资源及绝对诊断可继续收集，不自动改阈值。

有限行为AND的可接受违例数为0；单元是一个请求或一个消费者case，任何一次可复现错身份/非法free授权即失败。这个零观测门不估计场景总体单次成功概率，不用长串AND推断99.9%可靠性。B3的拒绝/超时具名报告，不把“有压力结果”当正确性失败；完整性与业务性能分开。

## 6. CPU、内存、环境与开销

owner在PREP冻结进程分组：backend owner/replica/export及其工作子进程；C'服务/适配器；client/collector工具。PID+starttime防PID复用；工具成本单列。A/B逐侧运行，双方允许CPU集合相同并从进程读回；不要求特定governor、不改BIOS/不杀其他任务。记录起止CPU、online/affinity、governor、loadavg、线程环境、ROS executor线程数，变化事件记录在案。

CPU以进程`utime+stime`差量加已退出子进程rusage计秒，采样100ms。相同子进程不能父rusage和独立PID重复计费；工具必须给PID计费表。块mean cores=CPU_seconds/wall_seconds；%one_core=100*mean_cores，不除整机核数冒充低占用。B1/B2报告整链CPU/valid链和CPU/attempt；有函数/线程级计数才细分到更新/查询。B3是混合负载，只报告整体CPU与总体业务计数，**不把同一总CPU分别除更新数/查询数后声称独立成本**。

每100ms记录同一采样tick的所有计费进程RSS/PSS。sum RSS明确有共享页重复；sum PSS作进程树物理占用近似；二者取tick总和的max，不能把各进程不同时刻峰值相加冒充同时峰值。PSS不可读记unknown；不能用RSS代填。后端原生唯一对象bytes另列，不与RSS/PSS相加。单位bytes，图表MiB=bytes/1048576。

2GiB保护按backend+C'的同时sum RSS、连续3个tick超限触发；保存后停止组，状态RESOURCE_LIMIT；这不是精确瞬时峰的验收证明。缺少PSS只使PSS项NOT_EVALUATED，不阻断其他可信指标。

T1必须低开销，首次计分前每后端代表组做一次30对ON/OFF短操作资格：同序列、状态重置配对，15ON/OFF+15OFF/ON；更新visible/full_query的P95 ON/OFF≤1.10、计数/身份/内容不变，才宣称仪表化时延可代表该profile。OFF仍保留端点时钟与响应，仅禁详细T1 collector；不能关闭产品本身固有逻辑。若不满足，可减少工具采集后另冻T，仅受影响资格重测；预算内仍不满足则延迟只报instrumented_diagnostic，不签后端速度比较。此为本次新采集资格，**不是重跑旧O9全套/长稳**。全部正式组使用同一已资格profile。

perf锁覆盖资格、计分与最终取数；记录父PID、锁路径、获取/释放mono_ns。启动等待不消费样本；所有子进程退出/错误分支释放锁。控制程序行为测试验证竞争者拿不到锁、异常退出能释放，不能只打印locked。

## 7. 正确性、共同使用方与无解定义

consumer清单范围固定为G/O/C版本的luggage_perception、luggage_packing、luggage_planning、luggage_bringup及uld_occ_observer中，由CMake/install/launch加载或被这些活跃节点import/call的生产路径。跟踪服务/订阅和同进程调用，不纳入所有历史实验脚本。每个排除项要有未安装/ROS1/实验开关关闭的证据；配置关闭但本次共同部署要求启用的节点不能排除。

主计划将错误geometry/frame/单位分为3个独立case，每消费者共10种状态、每后端10次，共20个case；共享fixture允许，但每消费者必须有真实入口观测。正常positive不得只用mock输出，placement和planning admission必须在真实ROS回调/实际函数链上验证输入身份；禁止调用trajectory执行、attach或硬件命令。现有服务若绑定运动，使用不连接执行器的同一规划回调及固定真实格式机器人输入，输出仅落盘。

其余7类保持主计划定义；reset测试事件顺序固定：接受旧epoch→发请求并拦住返回→失效→放行旧响应并确认拒绝→新图valid但旧机器人样本仍拒绝→更新至transition后机器人样本→正常恢复。只测有限序列，不新增长稳。

至少每hull一组具名可行正例与一组occupied/unknown反例，由独立几何oracle验证；正例用测量射线形成足够known-free，不能修改地图初始化为全free。all-NO_KNOWN_FREE的判官必须拒绝“全面功能通过”；若实际fixture无可行位置，则该fixture可合法无解，但不能替代正例覆盖。

地图正确性按各自独立已存在ray/log-odds/domain oracle，输入同manifest，校验known/free/occupied和unknown来源；通过后才用两后端互比图像/分类。共享ROI的2cm域按hull定义cell集合，输出分类confusion矩阵(3×3)及每类计数、同为已知surface列的高度误差median/P95/max、缺失列/unknown比例。差异本身不设零门；任一后端违背自身射线/域合同为FAIL，不以双方碰巧相同作正确证明。

候选oracle固定使用C的冻结放置policy参数（support ratio/容差/倾角/yaw/clearance/hull）及独立几何判定；manifest必须展开数值和源码hash。输出每个返回候选的支持、hull、occupied碰撞、unknown内部体积结果，完整朝向体积而非中心点。floor_prior可表示地板存在，不能表示全箱空；若仅满足prior政策，标PRIOR_ONLY，不能计为KNOWN_FREE。正体积碰撞或内部unknown仍标KNOWN_FREE=假合法，零容忍。

规划接纳验证使用同快照/geometry/clock与有效机器人样本，检查规划前/提交前二次身份；这里只证明规划消费边界，不证明机械臂可达或物理执行成功。真实运动路径算法若为本consumer既有调用且可离线运行，照常记录其结果；不为比较新建运动场景。

## 8. 完整验收表 C0–C8

| Gate | PASS | FAIL/BLOCKED/NOT_EVALUATED |
|---|---|---|
| C0 版本/输入 | manifest锁定G/O/C'/T、载荷/配置/install hashes；24组输入与当前运行一致；dirty规则满足 | 错版本/篡改/语义冒充=FAIL；缺组合=该组合NOT_EVALUATED |
| C1 判官/执行器 | 下述负例全拒、正常控制全过；退出码传递、锁竞争/释放、无自动补跑 | 任一假通过/吞错=工具FAIL，不进入评分 |
| C2 消费者 | 所有必需消费者20状态case均正确；positive真实通过、非法zero授权；能力矩阵无未解释项 | 真实违例=FAIL；缺能力/适配范围超产品边界=BLOCKED；不可运行依赖=NOT_EVALUATED |
| C3 比较资格 | 同源同操作/同C'、trace资格、G/G及O/O时间资格通过 | 时间资格不足=时间NOT_EVALUATED，允许非时间证据；工作量不同只可DEPLOY口径 |
| C4 B1完整性/正确性 | 每组每侧60条更新与60query账本完整，逐帧对应及oracle通过；有效样本数和失败分母如实 | 错身份/错误事实=FAIL；合法超时/拒绝保留完成率，成功P95不足30=该指标NOT_EVALUATED |
| C5 B2真实业务 | C2相关行PASS；每组每侧60条、箱型各30；实际读取目标身份，所有返回候选合法 | 非法授权=FAIL；缺输入链/不能证明实际消费=BLOCKED或NOT_EVALUATED；合法无解不是FAIL |
| C6 B3负载 | 300input/240query预期ID全对账，各状态保留；正确性通过；2.5/2达成率与积压按定义报告 | 漏/重账为工具FAIL；真实NOT_SENT/TIMEOUT是业务结果，不伪造计数；宣称维持目标需§5.2发送资格和窗内有效吞吐同时满足，Q(150)=0、每请求截止有效 |
| C7 资源/统计 | 单位、进程计费、窗口、ns原值、n/分母/分位/比值全部可重算，缺测具名 | 错聚合/双计费/虚假数=证据FAIL；缺PSS等只该指标NOT_EVALUATED |
| C8 归档/收口 | 新目录、原始失败保存、manifest校验、独立judge从磁盘复算、精确命令和提交、最终四分结论 | 缺重放关键载荷=INCOMPLETE_EVIDENCE，不能写相关PASS |

“比较任务完成”不等于“两个后端产品全部PASS”。报告固定四栏：①工具/数据有效性；②使用方兼容；③各矩阵完成度及后端结果；④共同集成ready/blocked。所有必需consumer通过且C0–C8有效/相应矩阵完成，才称comparison complete/ready for integration；任何BLOCKED/NOT_EVALUATED逐行保留，不能只汇总一个绿色PASS。

没有“必须比另一方快”的产品阈值，不继承1cm/O5/O6压力门，不要求新一轮reset长稳。比较可得结论为：本次相同输入、所列profile下的可观测差异及能力缺口。不同采样、成功率、缺失能力、资格失败时明确限制速度排名，不臆测实机结论。

## 9. 判官有限行为测试（一次列全）

每类至少1个坏例及同路径1个健康控制；可复用现有工具夹具。所有坏例必须拒绝或具名NOT_EVALUATED，且judge退出码非0；健康控制PASS。固定类别：

1. 产品/输入/配置hash错或混版本；2. expected ID漏1/重复1/未知1；3. terminal超deadline后伪成功；4. 多页漏页/重复cell/身份或cursor错；5. 旧epoch/过期/geometry-frame-unit任一错仍授权；6. unknown或occupied内部被标KNOWN_FREE；7. 全部返回无解但正例被声称通过；8. NaN/Inf/bool/负时延/缺样；9. 分母删失败或对不足30样本声称有效P95；10. 混profile/不同consumer/不同工作量却声称全射线比值；11. CPU子进程双计、RSS不同时刻峰值相加；12. stage失败退出被driver吞、性能锁未持有或异常不释放；13. baseline期望被写回/篡改、失败没有可重放载荷；14. B3发送/排空边界差1ID或只查吞吐隐藏未闭合；15. 取证ON/OFF改事实/漏身份、trace丢事件未报。

判官自测先于正式评分；真实评分暴露同条款未覆盖路径时修该路径，不新增无关类别/指标。工具完整日志即便FAIL也需要可读，不能abort后不写SUMMARY。

## 10. dump矩阵与工件格式

| 边界 | T1 | 触发 | T2内容 | 窗口/格式/预算 | 重放 |
|---|---|---|---|---|---|
| 原始depth→预处理→入图 | 帧ID/hash/TF/过滤计数、发送及回执 | 输入不符、丢失、超时、oracle错 | 冻结全前缀引用+最近3帧原点/TF/数组、失败输入及原生回执 | NPZ/JSON，失败前10s后5s；大点云引用已冻结原始文件，不复制整个数据集 | replay --case ID --stage ingest |
| owner→可读快照/副本 | epoch/revision/覆盖集合/发布接收事件 | 目标输入未覆盖、错代、传播超时 | 故障前后快照/增量、发布载荷、解析结果、进程日志 | JSON/二进制原格式+SHA，前10s后5s | replay --case ID --stage snapshot |
| acquire→Surface/ROI→release | handle/page/cursor/状态/全链时间 | 页错、租约失效、释放失败、几何错 | 该租约全部页与请求、返回数组、状态转换 | JSONL+NPZ，完整故障链最长30s另加前10s后5s | replay --case ID --stage query |
| 使用方→选位→planning接纳 | 实际消费身份、请求/候选/拒绝理由 | 非法候选/旧图授权/服务失败 | surface/ledger、完整箱体、solver配置、关节/EEF固定输入、候选及校验结果 | JSON/NPZ，故障链+前10s后5s；无硬件执行数据要求 | replay --case ID --stage consumer |
| 计时/资源/driver→judge | tick资源、PID归属、planned/actual/terminal | 环境资格、保护限额、统计/退出错误 | 完整轻量请求账本与资源窗、实际argv/退出码、summary输入 | JSONL压缩，前10s后5s；原始完整轻量账本每run保留 | judge --run RUN；replay --case ID --stage driver |

每run≤8GiB、全任务≤32GiB；冻结输入去重+hash引用，JSONL/NPZ可无损压缩。额度含失败产物，剩余不足时停止下一个组并归档NOT_EVALUATED，不能删失败换空间。T2先落盘并校验manifest、fsync/正常close后再拆进程；T3只在已有T2定位后具名开启，T3数据不混入正常性能。

capture health：所有预期事件ID可join、drop_count=0、页/数组载荷hash能重读；人为一次故障可回放到同类拒绝/错误且原始期望不变；捕获缺失即对应结论INCOMPLETE_EVIDENCE。前10s覆盖B3多个发送周期，完整30s故障链单独保存以免窗口截断超时案例；后5s用于响应/释放闭合，不延长请求deadline。

最终必交文件：MANIFEST.json、INPUTS.json、CONFIG.json、EXPECTED.json、CAPABILITIES.csv、COMMANDS.sh、processes.json、events/*.jsonl、resources/*.jsonl、cases/*、junit.xml、SUMMARY.json、FINAL_REPORT.md及可独立运行的judge/replay。所有摘要均由盘上记录重算，summary本身不能是唯一判据。

## 11. 自动推进、预算与返工边界

Claude领取后独立完成：清单→适配/工具→有限自测→短预演→冻结→资格→B1/B2/B3→磁盘复算→提交/角色记录/Result。按主计划8h准备+8h计量预算；等锁不计。准备门过后自动评分，无新用户批准环节。发生必需能力超范围修改，停止依赖项并交付具名修复清单，独立公共比较仍可做；不为等全部绿色无限循环。

准备时生成一次机器可读CONTRACT.json，逐条映射C0–C8、本附件字段/统计规则/fixture/预期样本与工件，之后hash冻结。可执行脚本尚需实现，但本附件已经定义应执行的程序及预期观测；不得让“实现时再决定阈值”进入合同。运行结束以真实pass/fail/blocked/not_evaluated收口；工程outcome:pass表示完成授权工作并提供有效结论，必须同时注明各产品/比较判定，不混淆。

性能失败不是修到通过的授权；产品修复另候选。G1–G6结构与reset旧验收不因本轮工具变化重跑。后续审查只能指出本合同具体违例及受影响重测；不能追加更多hull、分辨率、long-run、显著性、实机或因果解释要求。共同集成另派任务，master本轮不改。
