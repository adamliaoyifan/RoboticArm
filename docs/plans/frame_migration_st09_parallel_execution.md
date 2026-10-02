# ST09 并行执行计划：三箱闭环、规划收敛与 QP 保留

- parent: FRAME-MIGRATION-20260929
- status: proposed; not dispatched
- date: 2026-10-02
- technical_baseline: docs/plans/frame_migration_st09_planner_convergence.md
- recovery_base: 2ea75165dd05de4ce042084e300a6b2f725447e3
- recovery_checkpoint: docs/status/evidence/frame_migration/ST09-g5-dirty-checkpoint-20261002/

## 1. 生效范围与不变义务

本计划将技术基线拆为可并行任务，覆盖其串行顺序和QP保留策略；技术基线其余要求、ca8426f及frame_migration_stages.md原ST09验收继续有效。
目标分为：M3三箱非评分闭环，R最终ST09发布；M3不能替代R，也不能替代独立scored INTEGRATION。
当前generation 5仍按原线程认领。本次仅编制任务包，不发送消息、不Claim、不创建worktree或运行agent。
reviews取得旧owner停止确认后，在原ST09 lineage绑定高generation；模块任务用同parent的稳定subtask ID及明确依赖，不复活已关闭的QP问题线程。

生产路线：placement -> reachable directed graph -> MoveIt 2 ->统一认证->执行。
QP保留独立实验分支，离线诊断和同输入对照；不通过原carry_planner开关悄悄加入生产。
QP后优化原始MoveIt边为实验目标，是否进入生产另设评审及验收，不能成为M3/R依赖。

## 2. 任务、建议owner与依赖

以下worker ID为拟派单目标，不声称会话已注册。reviews可替换具体worker，但每个runnable行必须绑定实际agent/model、base、plan revision、generation、acceptance和dispatch_ready。
各owner自己实现、测试、修复、提交、记录证据，不把日常单测移交test角色。

| ID | 建议agent/model | 依赖 | 交付 |
|---|---|---|---|
| ST09-C0 | cursor/grok-4.7 | ST08及g5停止确认 | 恢复提交、共同接口、fixture、文件归属、参数/验收manifest |
| ST09-A | codex-placement/gpt-6 | ST09-C0 | 精确hull、深度排序、观测需求 |
| ST09-B | cursor-route/grok-4.7 | ST09-C0 | 有向图、MoveIt补边、多IK/朝向、缓存 |
| ST09-C | codex-certifier/gpt-6 | ST09-C0 | 完整轨迹认证、地图更新策略、停止策略 |
| ST09-D | claude-map/glm-5.3 | ST09-C0 | 观测覆盖、异步地图、measured integration |
| ST09-E | codex-evidence/gpt-6 | ST09-C0 | runner、dump/replay、三箱extractor及coverage |
| ST09-Q | claude-qp/glm-5.3 | ST09-C0 | QP保留分支、blocked-knot诊断、离线对照工具 |
| ST09-I | cursor/grok-4.7 | A、B、C、D、E | 合并接线、删除旁路、mock whole-chain、M3 |
| ST09-R | cursor/grok-4.7 | ST09-I | 原两箱/注错/capture/proof/freeze、关闭ST09 |

表中A–E依赖均指完整ID。Q不在I/R depends_on中。
建议四并行worker第一波A/B/C/D，空出的worker随后做E、Q；资源充足可六线同时开始。
小任务可以同一个人顺序执行，但不得让一个会话假装同时持有多条可写任务。

```text
               C0（一次共同接口提交）
                |
        A   B   C   D   E       Q（独立实验）
        \   |   |   |  /       : 同一fixture/认证协议
             I                 : 可消费B已保存轨迹
             |
      mock全链 -> M3三箱
             |
       R原ST09完整关闭
             |
     单独派发scored INTEGRATION
```

## 3. C0：先冻结边界，不先做全量重构

串行完成以下一个可review基线提交C0_HEAD；其余模块都从此提交建独立worktree。

1. 校验检查点MANIFEST及237文件恢复。选择性恢复生产代码，同时将完整QP源码/配置/测试/replay制作成可恢复Git实验分支并记录commit和来源hash；保留原失败证据。
2. 生产基线默认graph且不能激活QP；QP实验分支有独立环境清单，不向生产安装注入OSQP或消息overlay。
3. 冻结现有运行identity、frame语义、margin、关节限制、起点/FK/tool tilt/J5/新鲜度/停止deadline等数值及来源到resolved_planning_contract.json。
   缺值必须在C0用mock/离线fixture定义验证，未解析值阻塞派单，不授权live临时调宽。
4. 提交共享ROS-free contracts及小fixture，不把生产重写塞进C0：

| 数据/接口 | 必需信息 |
|---|---|
| PlanningSnapshot | geometry/revision/acquisition stamp/occupancy digest/commit revision、immutable arrays |
| PayloadIdentity | measured dimensions、T_tcp_box、cargo/attachment身份 |
| AttachmentRecord | 完整刚体变换、尺寸轴定义、纳秒measurement stamp、frame/calibration、request/cycle、incarnation、来源/证据digest及validity；详见技术基线5.1 |
| PlacementCandidate | 完整箱体pose、支撑层、depth、geometry verdict、score、稳定ID |
| ObservationRequirement | 待观察footprint/走廊、缺失cell及原因；不含GT |
| GraphNode / Edge | 实际q、FK、朝向/载荷状态、完整timed trajectory、静态identity |
| PlannedRoute | request/admission、snapshot、payload、slot、进入/释放后撤离段、边/轨迹hash |
| CertificationResult | pass/reason、检查范围、contact/clearance/FK/joint/time诊断 |
| MapRevalidationDecision | continue/replan/stop/fault、revision/epoch、受影响边 |
| EffectTrace | causal sequence、request/route、阶段、before/after计数、允许的cancel/hold |

5. 冻结调用协议：A generate_candidates ->候选+ObservationRequirement；B plan_route ->未授权route或具名失败；C certify/revalidate ->结构化verdict；D integrate_view ->新snapshot；E记录这些边界。
6. 共享schema、fixture、reason taxonomy由C0/I唯一修改。并行owner在本模块提供adapter，不复制定义。
7. 冻结case清单、三箱工作负载、命令模板和文件owner清单。M3默认carryon,standard,carryon；spawner输入只属测试激励，生产只读实测尺寸。
8. manifest提供完整requirement->task->test->artifact映射，配置及命令不得含TBD。原试验失败只是基线，不判C0产品成功。

共同约定：箱体/TCP通过attachment转换；q/-q与pi/-pi等价不误拒；IK family标签只用于搜索/诊断，真实连续性和安全约束决定拒绝。
图可达先验不等于空闲或支撑，地图unknown不伪造成free。

C0冻结 attachment provider 协议及离线fixture；B独占纯目标合成模块，D独占实测attachment provider及其证据/TF适配模块，C负责失效/认证策略，I独占消息和共享node/driver接线。D可用C0 fixture独立开发，不等待B。provider与货图积分各自独立模块，禁止扩展成同一个manager。
C0核查恢复后的实际v2字段和生产测量来源，输出字段复用/缺口清单；不足才由I扩展v2并重建所有消费者、刷新proof，v1布局保持不变。现有仿真GT锁定变换不能登记为measured来源；若没有生产测量路径，D必须实现并通过fixture和I的真实观测门，不能留下“后续感知任务”才可运行的硬依赖。
冻结attachment测量年龄、TF同步误差、不确定度/滑移检查策略、尺寸轴与对称性声明；全量字段及reason写入共同schema。沿用原冻结数值，缺项按C0第3项补齐后才派单。

## 4. 并行任务包与独立验收

### ST09-A：placement几何与排序

独占写：luggage_packing内placement_solver.py、placement_planner_node.py及其focused tests；C0指定的公共hull模块。不得修改motion executor、mapper或driver。
技术基线S1完整适用。提供纯函数输出ObservationRequirement，由D实现观察，不在A里发运动请求。
候选按支撑层低优先、同层depth桶深优先、route最小净空、原placement score、执行时间/关节行程/稳定ID排序；有route的最终比较由B使用同一key。
不得以某条直线插入失败判BIN_FULL；将几何合法性和路线可行性分别记录。
输出容器坐标的完整箱体目标和两个footprint类，不用工具yaw改写theta；吸附前可初选，吸附后由B用实际attachment重验证。theta+pi只由声明的箱体对称性允许，不意味着TCP或机器人路径对称。

验收：保存的24个斜壁候选全部拒绝；2 yaw×2 scene×4深处状态共16 cases；纯几何center/corners<=1e-9；deep可用选deep，blocked/无支撑/unknown返回正确结果；unsupported tilt拒绝。
交付candidate_replay.json、全部候选与原因、fixture期望值、测试node IDs、commit。

### ST09-B：有向图和MoveIt适配器

独占写：reachable_graph.py、grid_path_search.py、新route模块和MoveIt edge adapter、focused tests。
禁止写trajectory_selector.py、motion_executor.py、launch或消息定义；最终接线由I处理。
可调用C0定义的certifier协议；C未完成时使用有显式fake标记的fixture结果，不能生成真实认证证明。
实际q为主节点状态，姿态/载荷/IK family为属性。多seed/goal、Cartesian、RRTConnect属于同一edge builder。
先构建portal/净空区/3–6个slot approach/exit/return的小图。缓存未经证据验证的历史折线不能作为合法边。
运输朝向固定于狭窄段，净空区允许旋转伴随平移；偏心attachment、释放后箱体障碍和空载撤离必须建模。
实现技术基线5.1的纯目标合成，完整T_tcp_box取代place分支PCA yaw推导；同时保留对称heading的多个TCP/IK goal，不按当前tool yaw提前只留最近解。atlas姿态未覆盖仅为seed/prior缺失，经统一有界补接判断路线，不能新增两bin白名单或经验关节差硬拒绝。
总体10s、单新MoveIt边2s、最多4次补边（含失败），预算耗尽具名失败。保存实际调用耗时并明确这些是开发预算，未测前不宣称可达性能。

验收：基线S2八类正负各1例；完整进入/释放后撤离route；joint order/velocity/acceleration/ns全部保留；同XYZ不同q不合并；warm健康fixture10/10认证路线且每次<=10s，cold1次单列；无fake certifier混入真实输出。
交付graph/trajectory hashes、MoveIt request/scene、edge失败原因、benchmark、commit。
另交付技术基线S2新增合同矩阵：8个sim/deploy target fixture、表示等价与偏心theta+pi对照、atlas域外可解/不可解/预算耗尽；纯目标误差<=1e-9。D/C提供同schema的独立输入，I完成ROS往返<=1e-6验证。

### ST09-C：认证与执行策略

独占写：新trajectory certifier、MoveIt validity adapter、map revalidation/stop policy纯模块及mock controller tests。
不写motion_executor.py、motion_planner_node.py；提供明确接线API和fault fixtures给I。
实现基线S3：完整连杆/吸盘/传感器/箱体碰撞、FK、限位、时间/衔接、姿态、occupancy、support contact受限豁免。
10ms及空间运动界自适应采样/保守连续检查，报告方法边界；不能用TCP sweep代替全机器人验证。
持续map updates不能锁mapper；before-send epoch检查和in-flight cancel/hold共同处理竞争。
认证未知/超时必须fail-closed。QP输入和MoveIt输入使用同一接口和限制。
attachment缺失、来源不合格、过期、frame/request不符、重吸附或滑移均使route失效；release也必须验证所需实测变换。执行器不得生成自动撤退，恢复由B规划、新admission授权。验证技术基线S2九类attachment负例，pre-effect拒绝业务effect增量为0，in-flight只允许cancel/hold等必要处置。

验收：6类map情形×sim/deploy；无关更新继续、相关更新重搜、全封闭拒绝、执行中可控提前量注错停止并保真空、stale和restart拒绝。
变异：漏velocity/time、位置替代完整姿态、插值间碰撞、错误joint order/attachment、伪造旧cert全部拒绝。
记录stop latency、停止包络、deadline及effect delta；pre-effect零业务effect，in-flight允许必要cancel/hold。
交付certification_results.json、mock traces、stop measurements、commit。

### ST09-D：观测和异步货图

独占写：mapper/integration公共模块及新observation policy模块、focused tests。
不写cargo_map_integrate_eval_driver.py等共享driver；提供integration adapter给I。
输入ObservationRequirement；最多2个预验证view，使用真实观测时间、TF和mask/cloud，不引入GT或catalog geometry。
覆盖待放区域与走廊，不把一个sensor cell解释为全面空闲。floor prior只按既有合同使用。
map采用immutable snapshot，积分继续异步；revision/commit idempotency和acquisition joins正确。
持箱时观测失效返回carry fault，不自行执行只适用于空载的aim。
同时交付独立attachment provider：同测量时刻实测箱体pose与TCP TF计算T_tcp_box、吸附确认后冻结、来源与stamp关联、失效通知。禁止用单独PCA yaw、计划slot或GT填充measured attachment；吸附前估计来源按技术基线5.1显式声明并验证。缺失证据返回具名invalid，不发布伪造有效记录。

验收：empty/missing depth/TF stale/duplicate view/hash conflict/commit重复/部分覆盖/覆盖足够各1例，sim/deploy变换各跑；缺失传感器不得提升free/support；三次有效观测依次产生可追溯revision，重复积分不重复计数。
先用保存RGB/depth/cloud验证，不需Gazebo。真实view的IK/可见性由I唯一live owner补证。
交付coverage/revision traces、两个view策略、adapter、commit。
attachment独立验收：有效观测+吸附确认恰好发布一次record；重复事件幂等；重新吸附更换incarnation；错时TF、缺pose、失效来源、滑移、未确认吸附各1例不发布有效新记录并输出原因。sim/deploy各跑，identity/stamp/digest完整保留；I接线真实观测后才可声称live provider已验证。

### ST09-E：证据、replay与runner

独占写：ST09 verify/extract/freeze/replay及dump工具、相应tests。最终proof发射和release result归R。
消费C0接口，使用fixtures开发，不修改planner业务决策。两箱/三箱extractor按实际n解析，不硬编码恰好两箱。
required sample inventory计算coverage；输出目录排他创建；0成功/1失败/2不完整。
GT与measured分别封装frame_dump/2，完整轨迹和输入snapshot可重放；fixed轨迹replay确定性，重新跑OMPL只比较安全/结果/预算，不要求bitwise一样。
freeze/replay必须显式install，拒绝偷用旧overlay。采集预算、窗口、三对回放指标沿用基线S6。

验收：完整fixture pass，逐项删除必需stream/sample得到incomplete，错schema/datum/identity拒绝；被截断轨迹不能得到通过；故障replay复现code和effect delta；三箱每箱事件和两个revision交接均正确join。
交付sample_manifest、dump/replay schema、fixture capture reports、commit。

### ST09-Q：QP保留、诊断与实验优化

独占写：实验分支的qp_carry/、QP yaml、replay_qp_carry.py、新edge优化benchmark及tests。
禁止写生产selector/executor/launch、共同certifier或消息。明确experiment=true，无controller/scene/vacuum/map mutation能力。
保留七模块、原tests及全部有用失败样本；参数改动都进入hash。
先重现box-0 blocked-knot：iteration/knot/q/TCP/payload pose、corridor bounds、occupied cell、violation、trust radius及停止原因。
box-1非法hull输入应在进入QP前拒绝，不能靠QP把非法slot“优化合法”。
提供 `optimize_edge(original_edge, snapshot, payload, budget)` ->候选轨迹/结构化失败；端点任务约束不变。
共同fixture即可开发，不等待B。B真实认证边可用后再做相同输入对照。
优化失败只返回失败；原轨迹由统一planner在当前snapshot重新认证后决定是否使用，Q模块不自行fallback或执行。

验收分两层：

- Q-core必须交付：原30 tests实际收集并通过（不硬编码历史总数）；两个历史失败有完整原因；无controller effect；预算内终止；至少1合成可行正例经真实C认证或独立相同合同验证。
- Q-benefit选做且不阻塞M3/R：B提供3条认证边后，每条原始/优化各5次，报告认证率、求解耗时、clearance、J5 metric、duration、joint travel。新增碰撞/约束违反数必须0；无认证结果记not_evaluated，无优势如实记no_benefit。

Q-core完成是保留及诊断工具交付，不宣称QP真实场景已认证或性能提升。Q-benefit未完成不把Q-core或ST09保持无限open。

## 5. 分支、文件归属和合并规则

- C0冻结共享contracts/fixture、build/package依赖与file-owner manifest；所有模块基于同一C0_HEAD，独立branch/worktree/install。
- 示例分支：implementation/st09-converge-{a,b,c,d,e}；QP为experiment/st09-qp-preserved。名字由I在C0确认无冲突后创建，不覆盖已存在分支。
- 主workspace docs/agents是唯一协调树；各worktree只有代码和本任务证据，不创建第二mailbox。
- 共享入口trajectory_selector.py、motion_executor.py、motion_planner_node.py、place_service.py、waypoint_generator.py、所有driver、launch、runtime registry及接口schema由I独占。
- A写现有hull时C/B用冻结kernel协议开发，A完成后在集成分支替换；下游不得复制几何内核。原模块参数变化走共同schema revision。
- 修改他人文件需给I提交具体patch请求；I判断接口影响并发布新共享commit，受影响worker显式rebase/重测，不后台改别人的dirty tree。
- I按A/D/C/B/E顺序合并review过的commits，每次只跑相关contract tests；全部合入后跑完整集成。跨模块失败由拥有该边界的owner修复，I保留唯一集成入口。
- mock ROS按任务分配不同domain且启动前核查占用，87留最终graph、42留replay、7留唯一Gazebo；不得并行污染同一MoveIt scene。
- 不并发全套CPU性能测量；benchmark独占固定资源窗口。Gazebo始终单owner。

## 6. I：接线与三箱里程碑M3

I合并A–E，迁移七类caller并删除独立生产旁路；QP完整实现已在独立分支保留，不需把实验依赖带进production install。
各caller经同一route/admission。typed pick/place segments、attachment、release、map mutation和return等既有v2合同全部保留。
先通过offline和mock whole-chain，包括in-flight stop/cancel/hold的最小行为门；不能为了三箱速度推迟这些门到live之后。
可以延后的是性能优化、长期atlas扩大、QP收益证明；不能延后合法性、地图一致性或载荷保持。

M3 workload固定carryon,standard,carryon，三个实际检测箱体，slot不预置，选点基于每轮map和实测geometry。
一次独占domain7、GUI/RViz off、非评分session<=20min。三箱全部成功才pass，任何箱失败保留原run并判fail/incomplete，不以重试混合成三箱通过。
每箱必须pick->attach->进入->release->measured verify/integrate->map publish->retreat/return完整。
第二箱消费第一箱更新revision；第三箱消费第二箱更新revision。无lost payload，0 floor-through，至少1 sensor-occupied cell并满足实际route所需coverage。
每箱GT限制沿用：XY<=0.04m、abs Z<=0.03m、drift<=0.02m、abs roll/pitch<=2deg、center inside hull、footprint>=0.40且分母>0。
记录选择depth及替代candidate；不能宣称对未知后续箱体全局最优或图中中心可达就一定可放。
返回/观察路线也必须经统一入口认证。

M3是1个三箱unit、AND全部3箱，要求该unit100%通过，只证明这个固定场景的闭环，不估计总体可靠性。
产物：immutable candidate、fresh install manifest、三箱事件/trajectory、两次revision交接、独立GT和capture_complete报告。

## 7. R：保留原ST09 release，不让三箱替代它

技术基线S5–S7完整执行：每profile三次healthy startup、所有fault/mutation、64-case各适用边界和driver、匹配base1795a69回归、原两箱carryon->standard、pre_pick/pre_place各一次、paired replay三对、F23与F24证明。
三箱的前两箱trace仅在冻结workload/配置/事件/观察窗口完全相同且runner支持严格证据子集复用时复用；否则单独两箱，不能手工口头替代。
captured replay <=1GiB/live run、campaign总计<=4GiB（包括三箱、两箱及fault包；去重按hash但各run依赖必须可恢复）。预计超限先阻塞新run，不删失败证据。
pre=ceil(2*max upstream latency)，post=ceil(max downstream latency)，各必需链路正样本>=1。
domain42三对off/on replay，median throughput degradation<=10%，完整stream joins/checksums，exactly one clock publisher。
失败replay复现拒绝码和effects；损失输入/跳过工作不能计作吞吐提升。
修复后新candidate/new evidence，不在同一冻结run改artifact。

runner CLI沿用技术基线的offline/graph/rehearsal/all及显式ws/install/out/domain参数；E新增并测试 --sequence-ids carryon,standard,carryon，只覆盖测试激励。
最终all须要求M3与原ST09所有sample inventory齐全，exit0；两个contract checker exit0，无pending/stale。
R统一刷新所有受影响proof，F01/F21–F23 verified，F24 isolated；独立install/ament index/全过程graph均无legacy。
最终clean源码与loaded artifacts、scene/URDF/SRDF/atlas/graph、schema、观察位、extractor、配置hash匹配。
QP分支hash作为保留资产列入交接，不标为生产certified artifact。
更新INTEGRATION计划的candidate来源及route事件规则并由reviews绑定；scored INTEGRATION仍单独派给eng/claude/glm-5.3。

## 8. 派单包、命令与关闭规则

每行派单复制本计划对应节，并填写：确切owner/model、稳定subtask、generation、base=C0_HEAD或I集成head、plan_revision、depends_on、exclusive file list、证据路径和commit验收。
禁止未注册worker自动调度；无CLI会话时保持file-based待领取。共享接口变化后生成新版本并记录影响，不能改写已claimed要求。

每任务将自己的tests放入C0约定的独立tests/st09_parallel/<task>/目录；具体实现可适配现有package fixtures，但统一执行入口如下：

```bash
python3 -m pytest tests/st09_parallel/A -q
# B/C/D/E/Q使用各自任务目录；B/C的真实MoveIt cases需先启动隔离mock graph
bash scripts/check_agent_contract.sh
```

命令是实施交付接口，现阶段不声称目录已实现。每目录必须至少有一个健康行为正例及本节全部负例，collected=0失败；B/C仅fake测试不满足最终验收。
实际构建/source/启动/回放命令由C0/E固化到machine-readable manifest，包含所有参数，不留依赖个人shell状态的隐含overlay。
raw evidence放docs/status/evidence/frame_migration/ST09-parallel-<task>-<revision>/，T0/T1/T2窗口/矩阵沿用技术基线S6；每task记录required/observed samples、缺项、checksum和环境。
模块离线测试是可用交付，不等于完整ST09通过。owner通过后提交并agent_complete；I/R以独立结果关闭全链。
QP实验失败如实保留，不能影响已经合格的MoveIt三箱主线，也不能因为三箱通过就宣称QP成功。
