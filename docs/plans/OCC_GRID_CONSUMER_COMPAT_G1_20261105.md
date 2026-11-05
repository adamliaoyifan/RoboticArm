# 新 G 与共同消费者兼容执行合同 G1

- date: 2026-11-05
- status: approved_for_dispatch
- parent: OCC-QUERY-INDEX-20261025
- subtask: OCC-GRID-CONSUMER-COMPAT
- generation: 1
- owner: eng/claude/glm-5.3
- grid_base: a4ac889e9130cc8ab0fa6d81f0060d8b480337af
- octomap_control: cbdd5364206dca70c89c251b43c681c708e0c5cf
- consumer_base: cbdd5364206dca70c89c251b43c681c708e0c5cf
- tool_reference_only: 15f56d73fc48ae1e6b3d5a56d921996d71467211
- dispatch_ready: yes

## 1. 目标、完成含义和边界

让真实 depth point cloud 驱动的新 G 经生产发布/查询接口，被同一份 placement/planning 消费者正确接纳；初始化、reset、延迟旧响应和恢复均保持身份、时效、known-free与候选合法性。共同消费者修改同时在 O 固定产品上验证。交付新G提交、共同消费者C1提交、工具T1提交和可离线复算证据，为后续共同集成提供候选。

本轮不要求G与O生成相同地图或相同候选；要求各自忠于实际观测、满足同一消费合同。没有“必须更快”的门。本轮不做B1/B2/B3性能评分、最大吞吐、长稳、Gazebo、实机运动、生产迁移或master合入。自研结构整理与OctoMap历史reset验收不重开。后续保留两套后端，由有效比较决定默认启动方案。

本轮允许G真实validity生命周期、必要查询/发布身份绑定、共同消费者兼容与确定的C'修复。禁止混入体素算法、采样策略、分辨率、候选排序/支持率/碰撞容差优化，禁止Python→C++重写。Python成本后续单独优化；本轮只保持明确快照所有权、同请求可复用的快照和窄接口，不能为了零拷贝暴露可变内部状态。

## 2. 独立源码与版本纪律

G开发从grid_base建立`agent/eng/occ-grid-consumer-compat-g1`；共同C1从consumer_base建立`agent/eng/occ-common-consumer-compat-g1`；O使用只读detached control worktree。建议分别位于`/tmp/elfin-occ-grid-consumer-compat-g1`、`/tmp/elfin-occ-common-consumer-compat-g1`、`/tmp/elfin-occ-compat-o-control`。路径已存在先核对，不覆盖。协调只在主workspace docs/agents中进行；不取主workspace在途改动作为隐含依赖。

G算法/节点包与C1消费包分前缀build；冻结SOURCE_MAP与INSTALL_MANIFEST，列每包source SHA、安装产物hash、实际import路径、可执行文件与动态库路径。两后端用同一C1安装前缀与同一policy，只切换明确的后端接口映射。禁止PYTHONPATH意外导入旧/tmp/install；ROS图中同一端点只允许一个选定后端发布者，不做运行中热切换。

旧T只作缺陷参考，不整包认定可信。C'的Time.nanoseconds修复、bridge入口、datum转换逐项移入对应产品/适配模块并加行为测试；`g_epoch_bridge.py`的31位截断方案禁止复用。v2发布器必须从真实生产快照生成，不把比较工具的假发布器作为正式依赖。

功能验收前提交G1/C1/T1并冻结完整40位SHA、diff、dirty=0（构建/证据目录排除且解释）。运行中改变任一代码/配置/夹具/install则停止受影响验收，保留当前run，另立新冻结、新run。未受影响证据复用必须有依赖hash证明，不能仅说“只是工具修改”。本文件提交后才具备确切plan_revision；派发须另写同parent子任务、agent_start领取。本文件本身不代表已派发或已领取。

## 3. 必须解决的实现点

### I1. 单一真实性来源与状态机

G store/其拥有的ROS-free生命周期对象是epoch、generation和有效性的唯一写入者；节点只做消息/TF/服务转换。不得由独立计时器、consumer或测试脚本猜测VALID。保留现有整次更新事务锁及提交顺序；不为本任务改锁性能设计，也不在锁内等待ROS消费者。

状态：启动未就绪保持INVALID/BUILDING；reset线性化点先使旧身份失效，再允许新观测；至少一次**新epoch实际接受观测**、新快照完整性通过且首次发布age≤1s，才能转VALID。floor prior、仅收到消息、空回执或更新stamp不能满足恢复。后续时效按当前消费合同判断，不能把一次VALID视为永久新鲜。

reset携带reset_id与预期epoch：同ID同内容重试幂等；同ID不同内容拒绝；旧epoch请求拒绝；BUILDING中再次不同reset使前一次重建失效。保留现有无ID legacy reset接口语义，若需要新增幂等接口则在G/公共消息包中实现，不依赖O内部实现或改造legacy请求为伪幂等。

提供可查询当前状态及可重放状态发布（可靠、transient-local或等价机制）；status和map须来自一致事务状态。节点/消费者重启后先关闭接纳，取得当前状态及新鲜输入后才恢复。没有validity不得执行旧的“尚未seen便放行”分支；共同部署中`validity_gate_enforce=false`必须明确拒绝启动或拒绝生产请求，不能作为测试成功路径。

### I2. 无损身份与时钟

G原生epoch为UUID字符串，O现有为整数；共同C1内部以**无损规范字符串**比较epoch，O整数字段在adapter中转十进制字符串，G完整UUID保持原样。原生服务继续使用各自真实类型；adapter保存可逆映射，不做hash、截断或按到达序临时编号。generation为单调有效性版本，仅在同一生产者生命周期内比较；新G进程/新epoch不能因generation从小值开始而被永久当作旧流。状态查询及受控重连确定当前生产者，延迟旧状态不能切回旧epoch。后端标识纳入绑定，禁止G/O epoch同值串用。

身份贯穿validity、Surface v1/v2、ROI/known-free、句柄、候选与planning admission token：backend、完整epoch、generation、revision、geometry/config/clock identity及原生可用digest。必需字段缺失、generation=0、格式/类型错误不是兼容默认值；生产共同链拒绝。O wire不需为此改变uint64接口，若现有O能力不足须具名BLOCKED，不静默修改控制产品。

mono时钟用于本机截止/事件顺序；ROS采集stamp用于观测时效，两者对应关系记录。robot数据要有真实采集stamp/TF与内容；禁止零stamp改当前时刻来满足新鲜度。接收旧robot消息不等于获得新采样。重复相同VALID发布不得无限重置恢复屏障，低generation/乱序/冲突内容必须明确忽略或拒绝并记录。

### I3. reset与并发的真正边界

reset前已提交更新归旧epoch；reset后试图提交旧更新必须拒绝，不能写入新图。旧handle在reset后不得继续产生可授权的查询结果；内部计算可完成，但响应交付及消费提交须检查原始token。Surface分页、ROI和free-space同一次查询绑定同一不可变快照，不能混用新旧页或重试追最新图。

placement/planning均需入口与提交两次校验：提交检查**计算开始时捕获的token**，不能读取当前generation重铸token。失效后清除旧缓存、候选与尚未提交的plan。已交付结果在后续planning admission重新检查，不能认为一次成功永久有效。只证明停止新授权，不声称撤销已经执行的运动；本轮无真实运动。

恢复须当前VALID身份的图及在有效性转换后采集的新robot状态，两者都满足现有时效/TF合同。任一缺失/陈旧继续关闭。共同消费者没有此能力则修消费边界，不在工具中伪造fresh robot或空地图已接纳。

### I4. 共同消费数据与known-free

根据CMake/install/launch/import调用图冻结active inventory：placement_planner_node实际ComputePlacement及V2、placement_solver、occupancy_place_paths、hardware_place_planning的实际planning admission，以及所有被其调用的Surface/ROI/known-free读路径。每个排除项给出ROS1未安装、纯测试或本配置未启用的依据。外部MoveIt/机器人依赖不启动运动；测试停在真实planning admission/计划提交边界，不把假规划器输出当完整运动规划成功。

每入口列G/O endpoint、消息、调用函数、单位、height datum、frame、时效阈值、错误码映射与测试ID。缺必需写接口/ledger等能力必须独立具名BLOCKED；不得用空ledger冒充已实现，也不把发现的全新算法需求偷偷扩大到本任务。

适配器允许转换单位/坐标表达及消息格式，所有转换依据明确descriptor与TF；禁止把unknown当free、把surface-known当体积空闲、用floor existence推断整箱无障碍，或由GT/计划箱位生成在线占用。G/O状态词不同必须显式语义映射。单位错误用违反声明schema/单位标记的输入及已知datum样例验证；不能要求消费方神奇识别一个仍合法但真实单位谎报的浮点数，也不能直接PASS。

候选oracle独立于solver成功标记，核验所有返回候选的完整朝向体积、hull、occupied交叠、unknown内部、支持率、间隙/clearance及来源；保存每候选坐标/尺寸/yaw/身份。policy从C基线实际参数展开冻结，默认值必须成为具体值，修改policy属于本任务失败而非提高通过率。几何浮点比较只允许1e-9 m的边界数值容差，不扩大free范围；离散cell分类按冻结2cm网格半开区间和同descriptor裁剪。接触支持面与正体积重叠区分。不能只测中心点或候选数。

## 4. 测试输入与矩阵

本轮功能夹具固定2cm，cuboid/chamfer两hull，各3场景：S-free有足够传感器射线覆盖的可放区域；S-occupied指定候选与实测障碍正体积冲突；S-unknown指定候选内部存在未观测区。优先复用原30k夹具，正式验收前验证实际有效点/TF/原点和覆盖，不默认其正确。不能满足正例可在非验收准备阶段用确定性传感器射线生成器seed=20261105构建整场景并冻结源码/参数/载荷，不复制点凑数量。点数非本轮性能门，实际值逐帧记录；不引入100k性能矩阵。

固定两探测箱0.30×0.20×0.20m、0.50×0.30×0.25m。每hull的S-free必须预先保证至少第一种箱在**观测known-free**上可放，第二种如无解须独立证据，不强制它成功。输入只经实际depth/Integrate入口；传感器仿真GT只用于生成离线depth及离线oracle，在线不得读GT/正确答案/成功标志。robot夹具为显式测试输入，经真实订阅入口进入，包含冻结关节/EEF/TF与采集序列，不冒充实机。

### V1. 每个必需消费者的10状态 × 2后端 × 2hull

| case | 注入/操作 | 必须观测的结果 |
|---|---|---|
| K01 正常观测 | S-free、新鲜robot、真实VALID | 真实消费者接纳准确身份；placement至少1合法候选，planning真实admission及commit成功；所有返回候选oracle通过 |
| K02 occupied冲突 | 真实S-occupied；固定冲突候选送实际验证入口 | 冲突候选不被授权；solver可返回其他合法候选，不要求人为全场无解 |
| K03 unknown | S-unknown内部覆盖不足的指定候选 | 不被标KNOWN_FREE/不获该候选授权；其他候选只允许独立合法者 |
| K04 无数据 | 新启动无validity/无图；仅floor prior | 不接纳地图/不产生可执行候选；拒绝原因、空可用载荷及零下游admission |
| K05 过期 | 合法图超过冻结consumer max_age；另带不匹配revision | 两个子探测均拒绝，不追最新/旧revision取得成功；新鲜控制正常 |
| K06 geometry错 | 改geometry identity，保持其余输入相同 | 真实边界拒绝，零授权 |
| K07 frame错 | 错frame/不匹配TF | 真实边界拒绝，零授权 |
| K08 unit/datum错 | 错声明单位/height datum；合法转换控制 | 错输入拒绝，合法控制解析后的物理坐标匹配；未携带可判单位的接口由上游明确验证 |
| K09 reset旧响应 | reset后投递旧Surface/ROI/候选/token | 旧结果不恢复VALID、不接纳、不提交；旧payload保留为证据而不可授权 |
| K10 恢复 | 新epoch观测后依次缺robot、旧robot、新robot | 前两步保持关闭；最后使用新图和新robot重新计算/提交；不能复用旧候选 |

每必需消费者40个case；有子探测的case所有子探测都要满足。共享实现可复用单元测试，但每个活跃入口必须有实际调用证据；纯读者K01验证读/接纳，候选/commit断言由其真实下游链完成，不要求只读parser承担动作授权。消费者清单的数量在inventory结束时冻结，正式运行不得删失败行或将必需项改N/A。

### V2. 确定性并发/重启12种时序 × 2后端

每种1个受控case，用barrier/event证明先后关系，不能只sleep赌竞态；共24个case，不做长稳概率结论。每个case含健康恢复控制。

1. reset先于旧更新提交；2. 旧更新先提交再reset；3. 已acquire旧handle、reset后read/交付；4. 读在reset前完成、reset后消费者提交；5. placement开始→reset→恢复→旧计算提交；6. planning开始→reset→恢复→旧plan提交；7. BUILDING中第二次reset；8. 同reset_id同内容重复及异内容复用；9. VALID/INVALID/Surface乱序与重复到达；10. consumer重启且生产者INVALID；11. consumer重启且生产者VALID但缺新robot；12. G生产者重启/重连后投递旧流（O控制检验其现有对应重连语义）。

新旧winner由实际线性化/提交事件判定；旧更新被拒的状态只有在reset确实先发生时才算正确。reset前已经完成的合法结果可保留历史成功，但reset后的新提交必须受当前状态约束。test barrier只控制时序，不替代product状态、查询、判断或返回值；若实际目标路径未命中，case=NOT_EVALUATED而非PASS。

### V3. 正式判官反作弊测试

同一正式磁盘判官入口接收完整健康run副本和坏run副本。健康控制先通过，再分别做下列12类变异；每类健康控制通过、坏例退出非零且给出具体case/规则才算通过：

1. SHA或实际安装载荷错；2. expected ID缺失/重复/未知；3. summary写PASS但原事件拒绝或缺失；4. reset后旧token被授权；5. 缺新robot却恢复；6. VALID无新epoch观测或仅floor prior；7. epoch截断碰撞/跨backend串用/缺generation；8. 页身份混合或候选内部unknown/occupied却标合法；9. 只返回候选数、删除完整候选/实际消费身份；10. 超绝对截止伪OK或迟到改写原终态；11. 所有正例无解或偷偷跳过失败；12. 缺失败重放载荷/输入hash失配/重放到错误case。

禁止仅直调check函数测试；judge必须从归档根读取事实，忽略summary自报PASS，并对重新计算的统计与summary差异报错。至少把run793已知的缺身份/只计数量类错误做成真实结构反例，不以注释说明代替测试。

## 5. 判定指标、样本规则与不可假通过

| gate | PASS数值/条件 | 未满足时 |
|---|---|---|
| A0 版本/输入/构建 | 所有运行依赖hash一致；两侧C1同SHA；必需入口覆盖100%；实际import/install正确；产品dirty=0 | 错版本/篡改FAIL；缺依赖BLOCKED/INCOMPLETE_EVIDENCE |
| A1 判官接线 | 12类坏run全部拒绝，12对应健康控制全部通过；退出码传至顶层 | 任一假通过FAIL，禁止功能验收 |
| A2 G生命周期与身份 | V2 G的12case全部命中并正确；旧身份授权=0，重复reset额外生效=0；准备/恢复谓词可从原事件复算 | 真实违例FAIL；未命中NOT_EVALUATED |
| A3 共同消费者 | 每必需consumer V1的40case全部满足；每hull每backend有真实合法正例；非法授权=0；缺失必需字段默认放行=0 | 缺能力BLOCKED；错误事实FAIL；缺观测未评 |
| A4 O兼容回归 | 同C1在固定O上V1及V2 O的12case通过，O产品未变；无删功能/关门规避 | 共同修改破坏O则FAIL，不降门或改控制产品 |
| A5 回归与证据 | 受影响pytest/ROS行为测试failure=error=0，要求case skip=0；账本ID完整率100%、重复终态=0、capture drop=0；失败重放同case同失败类别 | 缺载荷INCOMPLETE_EVIDENCE；不得只看pytest总数 |

case是确定性合同场景，不是独立同分布随机试验；AND门要求已列场景零违例，不宣称99.9%可靠性。每个计划case恰一个终态，expected由冻结inventory/matrix产生，不能按收到事件数倒推。错误/超时/未发送/跳过保留分母；缺依赖允许工程交付BLOCKED但不是兼容PASS。正常请求deadline固定发送后30s，整链一次，不随分页/重试延长；超时是失败/未完成，不算负例的“拒绝成功”。设计要求拒绝的case须观察明确拒绝及零下游授权，单纯无响应不算通过。

本轮记录逐请求端到端时延、阶段边界、复制/查询调用次数和载荷bytes供后续定位，不签性能结论、不加1.10性能门。小样本不报P95优胜；只有同指标有效样本≥30才可按nearest-rank给描述性P95，仍标非性能评分。不要把各stage中位数相加成e2e。资源若记录须每PID去重、工具单列；缺资源证据不据此声称2GiB保护成立。本轮不沿用run793的错误保护器。

**禁止路径**：硬编码pass=True、缺字段补0/当前时间、关闭validity、旧图失败后floor fallback、工具直写store内部或consumer admission、替换真实solver、只检查源码字符串、以进程存活当服务就绪、未触发竞态却说通过、抓异常统一NO_KNOWN_FREE、把期望目录写成actual、重跑挑绿、只保存候选数、跨版本拼样本。健康对照必须证明系统能够授权正确输入，拒绝全部输入同样不能PASS。

## 6. 实现步骤与回归范围

1. inventory：冻结真实调用图、两侧接口/身份/单位/错误码表、policy实际数值、环境与6个夹具、每个测试ID。发现必需能力超边界立即具名，不隐去consumer。为身份/validity接口的规范变更同步更新最近architecture文档与rule，明确仅本共同部署启用严格接纳，不声称修复所有历史规划缺陷。
2. G核心：实现生命周期/状态查询/真实发布绑定，在已有store reset与registry边界加入必要失效/提交检查；ROS-free单元测试先通过。focused模块承担状态与适配责任，不继续把逻辑堆进大node。
3. C1：无损epoch、启动关闭、严格字段/时效/robot、入口与提交token、v1/v2/ROI/known-free一致性；产品化已审查C'修复。保持同一solver/policy与G/O共用admission。
4. T1：真实ROS生产链驱动、deterministic race barriers、完整响应与独立oracle、正式磁盘judge与12变异、故障dump及重放。preflight可自由重复，但每失败留原因，不能把预演当正式通过。
5. 原有回归：G的store/reset/confirmation/projection/query contracts/registry/paging/wire相关现有tests；C1的admission/validity wiring/solver/occupancy_place_paths/hardware_place_planning相关tests；消息改动做依赖包build+实际ROS导入/序列化。用按安装依赖导出的TEST_SELECTION.json列准确pytest路径与期望case数；不能用run793的180 passed替代本次回归。V1/V2验证实际行为，不接受仅断言源码含字符串的wiring测试作证。
6. freeze：提交、构建、记录imports、hash复算，A1通过后运行V1/V2与受影响回归；失败保持当前run，修复后新版本新run，报告全部尝试及原因。
7. closeout：独立归档judge重算A0–A5，重放至少一个预演注入失败并验证原输入hash不变；报告G新能力、O回归、未解决项、确切提交及集成建议。未自动启动性能或合master。

## 7. 工具命令合同与证据

owner实现新的`scripts/occ_consumer_compat.py`，以下是**待实现CLI接口**而非声称现有命令已可用。manifest传绝对路径；退出0仅表示该子命令目标完成，verify/judge仅全部必需gate PASS时退出0。BLOCKED/NOT_EVALUATED退出非零并落完整结果。

```bash
python3 scripts/occ_consumer_compat.py inventory --manifest MANIFEST.json --out PREP
python3 scripts/occ_consumer_compat.py preflight --manifest MANIFEST.json --out PREFLIGHT
python3 scripts/occ_consumer_compat.py selftest --manifest MANIFEST.json --out SELFTEST
python3 scripts/occ_consumer_compat.py freeze --manifest MANIFEST.json --out FREEZE
python3 scripts/occ_consumer_compat.py verify --freeze FREEZE/FREEZE.json --matrix V1,V2 --out NEW_RUN
python3 scripts/occ_consumer_compat.py judge --archive NEW_RUN --out NEW_JUDGMENT
python3 scripts/occ_consumer_compat.py replay --archive NEW_RUN --case FULL_CASE_ID --out NEW_REPLAY
```

逐case键包括run/backend/consumer/hull/case/subprobe/request；所有输入→store提交→validity/发布→读快照→实际consumer→candidate→planning commit事件可join。EXPORTED身份、实际消费身份、admission token、robot采集stamp、reset线性化点、终态、deadline都须保存；“请求声明的身份”不替代真实消费证据。

证据根`docs/status/evidence/occ_grid_consumer_compat_20261105/<run>/`。必需文件：MANIFEST、SOURCE_MAP、INSTALL_MANIFEST、CONFIG、INPUTS、EXPECTED、TEST_SELECTION、COMMANDS.sh、events.jsonl、完整响应/候选/页载荷及hash、pytest/junit、judge/VERDICT、SUMMARY、FINAL_REPORT及teardown记录。manifest包括缺件/drop/捕获完整性、每工件相对路径与SHA256。归档判官不得隐读/tmp、源码工作树、网络或历史expected；重放确需构建时从归档源码引用和确切依赖可复现准备，缺依赖明确BLOCKED，不能假装自包含运行。

| 边界 | T1必记 | T2触发与载荷 | 窗口/预算 | 检查 |
|---|---|---|---|---|
| depth→store/validity | 帧/TF/输入hash、accept、epoch/revision、转换理由 | 更新/reset/恢复错误：原点云/TF、前后状态、线性化轨迹 | 前5s后5s并保留完整失败30s请求；每case≤128MiB，输入可hash引用已封存文件 | replay对应case |
| snapshot→Surface/ROI | handle/cursor/域/完整身份、release | 混页/旧图/缺字段：请求、全部相关页与状态事件 | 同上 | judge页域/身份校验 |
| consumer→solver→planning | 实际接纳图、token、robot、完整候选、commit | 非法授权/错误无解/过早恢复：输入policy/图、候选、oracle逐项结果 | 同上 | judge独立候选及时间关系 |

每run≤4GiB、任务≤16GiB，输入去重无损压缩；空间不足先停止新case并报告，不删失败换空间。故障窗口结束不延长业务deadline；先flush/hash确认T2可读再拆进程或重跑。无故障正式run也保留正例完整候选；人工负例需实际重放至同类失败，不能仅重打印旧summary。capture overhead本轮只记录运行事实，不据此签性能资格；正式比较再执行既定ON/OFF门。

就绪预演每次最多120s，验证服务可调用、ROS graph后端唯一、输入与状态能被真实接纳；启动前失败不计为已执行功能case，但必须落预演日志。建议单次验收总墙钟上限4h（构建/依赖准备另报），到限保存并列未执行case，禁止静默延长或自动无限重跑。需要改变本合同阈值/矩阵/产品边界时在同一lineage更高generation记录，不改已领取要求。

## 8. 最终交付与后续

最终四栏：①工具/证据有效性；②新G真实兼容；③固定O上共同C1回归；④共同集成候选ready/blocked。只有A0–A5全部PASS才称本轮兼容完成；缺必需consumer或任何未命中必需case不能以工程完成替代。完整提交列表区分G、C1、T1；声明未测性能、未实机、未合master。

下一阶段以本轮候选G1/C1和固定O为比较输入，先完成已有R01–R14工具返修，再冻结新的比较generation。Python热点与原生加速独立任务；默认后端选择必须来自合法比较，旧86×及错误2GiB触发不能作为选择依据。

## References

- docs/architecture/perception_architecture.md
- docs/architecture/placement.md
- docs/architecture/real_scenario.md
- docs/plans/OCTOMAP_RESET_VALIDITY_NEXT_20261103.md
- docs/plans/OCC_DUAL_BACKEND_CONSUMERS_G3_20261104.md
- docs/agents/reviews/2026-11-05_1416_occ-compare-consolidated-open-items.md
- docs/agents/reviews/2026-11-05_1426_occ-correction-verification.md
