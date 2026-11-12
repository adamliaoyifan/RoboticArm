# MoveIt atlas 与整链抓放规划执行计划

- parent: MOVEIT-ATLAS-JOINT-PLAN-20261111
- generation: 1
- status: proposed_execution_plan
- dispatch_ready: no
- base_revision: 599bf289f66d9474b8117eb47f6d58c40d00587f
- plan_revision: uncommitted; 首次派单前固定包含本文的准确提交
- scope: 仅MoveIt；观测后联合规划、条件化装箱域、atlas分支图、成功轨迹复用
- authorization: 用户本轮要求执行计划与排序；本文不创建派单，不启动实现或机器人动作

## 1. 已确定的产品行为

1. 在pick观测位获取箱体尺寸、有向姿态、有效顶面，与ULD的同身份OCC/静态场景、当前q联合规划；观测位不等于最终pick接触位。
2. 整个吸盘投影加现有margin必须完全包含于同箱有效顶面，不能用95%覆盖替代。沿用当前保守footprint和其他质量门。
3. 本轮空箱，不引入重量、偏心力矩或机械偏心距限制；箱体/工具/传感器碰撞几何与原包络不变。
4. 联合选箱心、箱体四个独立朝向(0、π/2、π、-π/2)、合法偏心抓取、工具相对角、完整pick/place IK分支和携带通道。π与-π不重复枚举空间姿态，关节winding仍独立。
5. 两端产生候选、中间验证携带连接；完整预测动作链成立前不发align/pick执行。J6调整属于已检查的完整关节轨迹，不做规划后独立改角。
6. 抓取前携带段使用预测附件；抓取后以确认附件和实际q验证并优先复用，不匹配才有界修复，禁止改写attachment使计划匹配。
7. 目标箱心不自动吸附到atlas格中心。深且可执行优先；浅且快是后备，并避免不必要地关闭剩余深位通道。第一阶段采用原solver离散合法箱心，不新增连续挪点容差。
8. release目标与settled箱心分开；释放后箱体保留为世界障碍，最终观测确认后由原OCC owner更新。
9. 当前请求图维持清理。经验库独立、有界、可失效；未评估/预算耗尽不当作物理不可达或BIN_FULL。

## 2. 排序、责任与依赖

以下owner为拟定的串行责任人，不启动子代理。每项owner自行完成实现、相关测试、修复、提交及证据。正式领取前按任务实际依赖提交更新base_revision，不能使用旧基线认证新代码。本文整体暂未派单；每项plan_revision在派单时固定为包含本计划及该项冻结清单的准确提交。

| 顺序/ID | 优先级 | 拟定owner | 依赖 | 有界交付 | 通过条件 |
|---|---|---|---|---|---|
| ST-0 | P0 | eng/codex/gpt-6 | none | 当前生产/安装接线核对，输入与指标基线、共享契约、冻结回放清单 | 路径与参数可追溯，正反例非空，阶段计时与失败捕获可用 |
| ST-1 | P0 | eng/codex/gpt-6 | ST-0 | 完整吸盘包含域、有向抓取候选、目标引导评估接口 | 所有包含/方向/变换行为门通过；不再把未评估点冒充accepted |
| ST-2 | P0 | eng/codex/gpt-6 | ST-0 | 当前atlas结构化读取，paired contact/transit真实消费、来源追踪 | atlas配对进入实际refine/checker；旧artifact缺边时明确fallback |
| ST-3 | P0 | eng/codex/gpt-6 | ST-1,ST-2 | 两端候选和中间连接的整链planner；先plan-only | 目标反推抓取/工具角，换pick分支可恢复；仅完整链返回PLAN_READY |
| ST-4 | P0 | eng/codex/gpt-6 | ST-3 | 实际typed调用和执行阶段机、实际attachment复核、阶段场景切换 | 从观测到完整计划再动作，满足事件屏障；匹配时复用携带计划 |
| ST-5 | P1 | eng/codex/gpt-6 | ST-4 | 新分支图artifact及有向原语，多portal/分支与局部补采 | 新增信息全部有consumer；身份/边完整性门与真实MoveIt验证通过 |
| ST-6 | P1 | eng/codex/gpt-6 | ST-5 | 条件化箱心候选域、深位优先/浅位后备、typed结构批次续取 | 四朝向/抓取正确求并，已抓只用实际grasp；unknown后备和共享预算成立 |
| ST-7 | P1 | eng/codex/gpt-6 | ST-4,ST-5,ST-6 | 有界成功经验库，验证旧轨迹/连接后复用 | 热命中不调用全局重规划；变化正确重验；不污染请求图 |
| INT-1 | P0集成门 | eng/codex/gpt-6 | ST-0..ST-7 | 整链回归、容量/时间对照、部署身份核对与回滚 | 功能/正例/性能/容量各门独立有结果，失败证据完整 |

顺序理由：先形成可观测、可验证的消费者和最小真实闭环，再花成本扩展采样；先形成成功路线和稳定场景身份，再做持久缓存。ST-1与ST-2技术上独立，但本次不派并行工作。阶段优先级不授权越过依赖。

## 3. ST-0：基线、契约和验收输入冻结

范围：复用现有exact replay、capture与phase accounting，核对src/install与实际typed入口，不修改采样或搜索策略。架构增量先更新frame/height消费者清单、placement_prior、placement和相关规则。任何尚未合并的region分支不在本任务隐式merge。

冻结JointOperationPlan记录：plan_id、source identities、current/start q、grasp candidate+phi+T_BG、pick pregrasp/contact/lift、settled/release box pose、place TCP、完整关节分支/winding、timed leg trajectories、attach/release事件、预测/实际附件依赖及认证状态。禁止以单一place_pose字段混淆TCP与箱心。

把stage timings拆成 observation/admission、candidate、atlas lookup/refine、scene preparation、endpoint validity、edge planning、certification、attachment revalidation、cache lookup/validation及总时间。每个阶段记录计数、耗时和未评估原因；总时间与非重叠阶段和的误差<=max(5ms,总时间1%)，并显式说明并行阶段不作简单相加。

输出固定 `docs/status/evidence/moveit_atlas_joint_plan/g1/ST-0/manifest.json`：输入路径/哈希、模型/场景/URDF/参数、实际模块路径、命令、每请求deadline、fixture来源、预期结果。使用当前profile原有budget，不偷偷放大。ROS_DOMAIN_ID及sim独占规则按运行时登记，不硬编码与已有栈冲突的域。

输入分三类：A固定实际attachment的离线路由回放；B抓取前联合选择的预测附件回放；C实际执行/attachment/落稳反馈。A不能证明改变pick后的效果；B不能证明实机执行；C由最终集成验证。

性能候选矩阵为24个输入：3个深度带×2个占用等级×2个尺寸档×2种有效顶面域。每个输入的准确几何、姿态与初始q必须由manifest给出，不由评分阶段临时生成。测试用合成尺寸允许，但标明synthetic，不输入生产感知。无法从现有证据组成矩阵时先用非评分采集或固定合成fixture补齐；缺格not_evaluated，不擅自减少分母。

最少6个具有完整有效轨迹witness的不同输入，覆盖中心抓取、偏心、90度、180度、第一IK分支失败后恢复、非空ULD各项。witness在非评分阶段用相同模型/场景独立检查，不能以待测系统自己的success字段作为唯一真值。找不到witness的输入保留为未知诊断，不当作必成功测试。

ST-0通过条件：manifest所有路径可读且hash一致、必要正/反例非空、baseline命令可重复执行一次、T2失败捕获能复现一个固定失败；无完整性能矩阵则只允许不依赖它的功能子任务，INT-1性能不得进入评分。

## 4. ST-1/2：先让现有信息进入正确接口

ST-1责任边界：感知evaluator唯一负责patch质量及完整包含；planner只能请求候选点/朝向。保持当前0.18m保守footprint、15mm margin和质量阈值。增加有向候选身份，旋转后不复用旧accepted结论。分页/会话保证目标有用点在提前停止前得到评估机会；实际pick最多3次的既有规则不变。

ST-1行为矩阵：矩形内部、1mm越界、恰好满足margin、大吸盘小顶面、45度轮廓、凹口、未知孔洞、95%覆盖但局部越界、跨箱面共9类；只有完整包含且原质量门通过者接受。变换覆盖4个箱体yaw×5个抓取偏移×2个箱高=40例，正反变换误差<=1e-8m/rad；另测非平凡容器变换、π偏心、名义/包络中心不等、退化归一化轴。

ST-2责任边界：新AtlasProposalSet reader/adapter与RouteProducer窄接口。返回真实sample pose、branch id、q、contact/transit pair、transit target/lift、质量、覆盖/失败原因。当前旧NPZ没有可重放完整边时明确标缺失并走在线连接，不伪造已认证边。contact与release查询分别记录，精确目标不被格中心替换。

ST-2行为门：构造首分支拒绝第二分支通过、contact通过而第一transit失败、不同transit_target、unknown、hash失配等fixture；检查实际refine与checker调用参数、最终选择和fallback，而非字符串或日志存在。至少一个既有artifact回放观察到paired transit被实际使用。有限采样unreachable硬拒绝的变更必须与架构合同同改；未批准前不能静默改变旧语义。

## 5. ST-3/4：最小完整产品闭环（里程碑M1）

ST-3建独立联合proposer/planner模块；RouteProducer只编排。先固定当前离散箱心，不扩展连续目标挪动。目标反推T_BG和pick pose，求完整pick分支，从实际lift q连接place入口；J6只作为完整q的一维。工具phi不直接等于J6。

候选首批最多8组、深入验证最多3条不同分支/通道作为初始明确配置；失败续取共享原deadline，保留旧系统完整路线reserve，不给每批重置预算。配置写入manifest。unknown/scan cap返回SEARCH_INCOMPLETE，不当空间不可达。候选完整包含/双端IK/端点检查/原语连接/整链认证按成本逐层进行。

ST-3用例：phi-A可pick但carry失败、phi-B整链通过；同pose两个pick q只有一个可接portal；J6限位拒绝；入口外转向；退出被刚放箱阻挡；深位失败有浅位fallback；固定箱体初末姿态不能用pick预转伪造零旋转。全部断言实际轨迹及场景，不只返回码。无完整链时PLAN_READY数量必须为0。

ST-4接入真实typed入口及状态机：OBSERVE→SNAPSHOT_READY→JOINT_PLAN→PLAN_READY→ALIGN/PICK→ATTACHMENT_CHECK→CARRY/PLACE→RELEASE/RETREAT→OBSERVATION_COMMIT。保留观测采集stamp，不因相机随后移动重解释旧世界系几何。

ST-4回归：空载接近时box为world；携带时为attachment；释放后为world且退出不漏障碍。预测附件匹配时复用规划段（全局规划调用数=0），实际变化导致轨迹无效时不继续执行旧段；可有效重验则不强制全局重规划。未PLAN_READY时执行请求数=0；alignment是受检全关节轨迹；候选/地图/模型身份错配拒绝。

M1交付：一个目标在完整吸盘约束下，能够先输出完整pick/place轨迹方案，再按事件屏障完成整次流程。先plan-only与回放，再按sim独占规则完成模拟链路；不以离线PASS宣称实机通过。首次M1不要求新atlas，也不要求持久缓存。

## 6. ST-5：扩展采样与闭环补采（里程碑M2）

新artifact含分支节点、有向joint/Cartesian原语、portal连接、静态检查与失败覆盖信息。保留旧reader兼容；生产配置显式选择新schema/hash，不覆盖旧artifact。不同原语不混证据，branch/winding保留。

先保留15cm/4yaw粗层，最大保留分支建议配置6，定向区域细化15→7.5→3.75cm；独立seed尝试即使已有成功也可触发。所有数量/采样耗时/裁剪记录。1.70m接触和1.55m carry限制逐项核对部署依据：保留时标政策范围，修改前非评分复现实测，不直接删除。多portal与不同lift均在原任务姿态合同内。

验收：artifact所有边引用有效、FK和记录目标残差满足当前solver合同、边重放符合原语约束及相同静态场景检查；未知/未评估与碰撞分开。消费者对新增pair/edge/quality信息都有实际调用或排序证据。至少分别覆盖“备用分支恢复”“备用portal恢复”“旧边fallback”三类正例。

报告每采样秒新增有效分支数、artifact大小、查询耗时、refine成功/尝试数、载箱有效节点/已检查节点、完整路线成功/请求数；不要求reachable比例必然增加。样本分母0报告not_evaluated。采样修复可在非评分阶段重复，进入对比后冻结artifact。

## 7. ST-6：条件化装箱域与容量策略（里程碑M3）

箱心域由完整箱体合法位置、合法有向grasp、atlas连通分支共同生成；按横向/高度/箱体yaw/尺寸/grasp条件化，最大深度仅用于展示和索引，不能填平域中的孔洞。抓取前合法grasp求并，抓取后只有实际grasp。

状态分为proven excluded、supported proposal、unknown/policy limited。静态空箱域对动态占用只是先验。完整载箱/退出认证仍然执行。浅位后备保留，但优先域内深位；用现有走廊/支撑/间距的廉价信息避免明显封堵，不引入整套EMS未来箱枚举。

将结构候选与next_batch接到当前typed入口；先生成/验证真实候选，再atlas排序/批次保留，避免Top-N被拒后无续取。保留惰性网格fallback与同请求总deadline。历史未合并region分支只作参考，不整包merge。

验收：同深度不同高度/横向结果不同；域内空洞保留；四有向姿态不被footprint去重；deep不可达不会无限重试；深位失败可在reserve内进入浅位；浅位堵通道的固定反例中优先无堵候选；续取能找到首批之外可执行解；预算耗尽不BIN_FULL。完整性只在遍历完授权候选全集时声称。

## 8. ST-7：成功轨迹经验库

独立有界store保存认证成功及实际执行成功的不同标签；先分段缓存，不要求完整整条完全匹配。匹配包含完整attachment、模型、ACM/padding、阶段约束与场景身份。近似匹配只给proposal，不能复用有效性结论。

顺序：旧轨迹当前场景验证→起终端连接→完整认证→必要时重规划。使用现有certifier的内点和时间约束，不把离散waypoint状态检查当整路证明。当前request图清理不变。

初始store限1000段且总落盘<=256MiB，任一超限按LRU淘汰；写入原子化、schema/hash错误隔离拒读，不改变机器人状态。阈值为资源上限，不是规划质量阈值。

验收：有效精确命中不调用全局planner且返回经当前验证的轨迹；新OCC/抓取偏移/工具/ACM变化触发重验或失效；旧边碰撞fallback；请求清理后经验仍可检索且图节点数不累积；容量和并发写故障保持有界。cold/warm各自统计，不用热缓存成绩冒充首次规划性能。

## 9. INT-1：整链验收与量化门

环境与版本：使用ST-0锁定的ROS Humble、deploy_update、机器人/工具模型、传感/OCC与规划profile；baseline和candidate同机、同输入、同deadline。实现版本在隔离clean checkout测试，相关代码/artifact hash固定。共享主工作树脏改动不替他人清理；部署前核对加载模块路径与已验收版本。所有必要依赖缺失/skip按not_evaluated处理。

过程：先运行各项已登记pytest行为测试，零新失败；再运行manifest逐输入的exact replay命令，最后实际sim/产品路径执行序列。ST-0将现有或新增runner的准确命令写入manifest，后续owner只执行冻结命令；manifest未冻结不能评分。启动/接线故障先在可重复非评分rehearsal解决，首次评分单元前失败不消耗产品单元预算。

### 9.1 功能和正例门

全部必要固定反例必须拒绝，全部6个不同完整witness案例必须产出符合检查合同的完整路线。此AND门针对确定性程序不变式，不是随机运行连续成功率门。预测与实测attachment路径分别有实际样本；仅plan-only不能关闭执行集成。

### 9.2 延迟与可靠性

24个冻结输入每个5次，共120请求/策略；比较baseline、闭环无缓存、闭环热缓存。预热单独记录，冷/热不混算。请求成功单位定义为预算内生成完整认证操作链，A/B/C来源分别报告，未知物理可行的诊断输入不混入已知可行可靠性分母。

已知可行的冻结子集：期望健康每请求成功率>=95%，本批观察接受线>=90%，并且成功率不得低于baseline；<=80%的明显失败系统不能通过观察门。该门是固定工作负载的经验验收，不宣称跨场景统计置信保证。5次同fixture重复不冒充5种场景。用Wilson区间报告请求统计并注明场景重复相关性，不以该区间作未经验证的总体保证。某case新增失败必须定位并判断回归，不能用其他case成功掩盖确定性缺陷。

性能门：共同成功样本>=20且来自至少6个不同输入时，first-complete-certified轨迹P95<=baseline的80%；不足为not_evaluated，不能PASS。P95使用nearest-rank ceil(.95*N)。同时报告含失败/超时的全部请求时间分布，失败计数不从分母删除。该20%是本计划工程验收目标，不是当前收益承诺。性能未通过时可记录功能完成，但不得称性能优化验收通过。

### 9.3 容量与周期

冻结3条各6箱的序列：大箱为主、混合尺寸、具有90/180度与偏心选择价值；具体输入/初始OCC/观测条件由ST-0 manifest给出。每条baseline/candidate各执行一次，比较可装数量、测量箱体体积和总周期；每序列candidate装箱数量和体积不得低于baseline，几何/轨迹/观测确认不得违规。单位是完整18箱尝试和3条序列，不能声称随机长期可靠性。测量缺失或仿真未完成则not_evaluated。

报告对应最终有效OCC、每次箱心目标与落稳误差、释放/确认失败、总周期；放稳误差必须满足现有执行profile已冻结的容差，不新增未经标定阈值。不以跳过难箱、更改尺寸、减少边界margin或缩短输出阶段换取速度。

### 9.4 证据和回滚

根目录 `docs/status/evidence/moveit_atlas_joint_plan/g1/<task>/<run>/`。T0源/环境/dirty/hash/manifest/命令；T1计数、阶段延迟、认证、cache统计、容量和失败分类；T2保留失败输入、有效面与完整footprint掩码、实际与预测attachment、目标/seed/atlas边、PlanningScene、MoveIt请求响应/轨迹、碰撞对/失败采样位置及事件屏障。必要T3仅局部扩大。

先持久化失败证据再teardown/rerun；捕获缺失先补仪器再重复。一次只运行一个Gazebo栈，按sim-lifecycle和stop_sim规则。不能以一次成功覆盖之前失败。

发布采用旧/新profile和artifact并存，先plan-only对照再启用完整链；回滚恢复上一已验收profile/artifact，禁止只切回代码却保留新schema输入。执行中失败按原保持/恢复协议处理，不在带载时切换抓取或清空世界。产品启用只在整链验收结果明确后进行；本计划编写不执行启用。

## 10. 里程碑与开始条件

- M0=ST-0：可复现基线和确切接口/验收输入。
- M1=ST-1..4：利用当前atlas的观测—联合规划—抓取—放置完整闭环。
- M2=ST-5：新采样信息进入真实消费者，分支/通道可重放。
- M3=ST-6：条件化有效装箱域和容量友好的深浅后备。
- M4=ST-7+INT-1：成功经验复用及完整性能/容量验收。

先实施ST-0。它不要求先重新采atlas、放大J6范围、移机器人底座或改变吸盘。派单前固定本文plan commit并建立同lineage任务线程；后续任务由依赖的Result/commit/evidence放行，不能仅凭目录存在认定完成。用户后续授权执行时无需再次讨论已经确定的空箱/完整吸盘/整链先规划原则。

## Pointers

### 2026-11-11 execution authorization and missing-atlas handling

API checkpoint decision: the user approved an independent complete-chain
preparation interface. It returns bound pick/place trajectories with event
barriers; execution consumes that plan and revalidates actual attachment after
grasp confirmation. Existing prior and per-segment interfaces remain compatible.
This decision authorizes the ST-3/4 boundary, not a claim of implementation.

Observation checkpoint (2026-11-11): the user confirmed depth-aligned RGB and
optional LiDAR, both mounted on the end effector. The user explicitly selected
active box-view acquisition before grasp. With a rigid attachment, moving the
arm does not change sensor-to-box pose; post-grasp wrist rotation must not be
credited as a new box viewpoint. Use acquisition-time transforms and existing
preprocessor/deskew ownership. An additional pre-grasp observation motion needs
its own collision-checked plan, with the ungrasped box retained in the world.
After observation, prepare the full operation from the actual resulting q and
new observation identity. No align/contact/carry execution is authorized by an
observation-only plan. Post-grasp observations may refine attachment/environment
evidence but cannot retroactively authorize the initial suction contact.

No numeric hole-filling policy is approved. Whole-footprint observed support,
inferred support and unknown remain distinct; missing returns, segmentation,
bbox or fitted rectangles alone cannot prove physical top-face continuity.
Actual new depth/LiDAR returns may add support only after same-box association,
registration and contradiction checks. Sensor availability is not proof that
the live synchronized/deskewed LiDAR adapter is already wired or qualified.

The user explicitly authorized development of ST-0 through ST-4, with no matching
atlas assumed for current `scene_tf_deploy_update`. These stages are now active
work; ST-5..7 remain outside this implementation authorization. ST-0 records the
actual identity result rather than relying on the existence of an old NPZ.
An unavailable/mismatched atlas supplies no seeds, ranking or negative verdict.
The joint planner must remain functional using bounded online IK/MoveIt, with
structured atlas support verified using identity-bound fixtures and any genuinely
matching artifact available. Building a minimal matching artifact, if needed to
verify ST-2 against this deployment, is allowed prerequisite work, not authorization
for the broad ST-5 sampling campaign. Missing atlas cannot block observation or
geometric candidate generation; actual IK, collision and complete-route gates stay.

ST-0 starts with the deterministic baseline suite:
`python3 -m pytest -q src/luggage_planning/test/test_atlas_runtime.py src/luggage_perception/test/test_suction_patch_evaluator.py src/luggage_planning/test/test_placement_prior_pick.py`.
Run under source-first PYTHONPATH with ROS Humble if needed. Persist JUnit, complete
log, import paths, revision, dirty state, and scene/atlas hashes under
`docs/status/evidence/moveit_atlas_joint_plan/g1/ST-0/`. A test environment error is
not a product pass. This initial baseline is not the later 24-input performance
campaign; that campaign is not a prerequisite to starting deterministic ST-1..4
development. No stage may be closed on its tests alone if its typed/native or
execution acceptance remains unverified.

- `docs/plans/MOVEIT_ATLAS_GRASP_CLOSED_LOOP_20261111.md`
- `docs/agents/reviews/2026-11-11_0943_moveit-atlas-route-retrospective.md`
- `docs/architecture/placement.md`
- `docs/architecture/placement_prior.md`
- `docs/architecture/frame_contracts.md`
- `.cursor/rules/debug-evidence.mdc`
- `.cursor/rules/sim-lifecycle.mdc`
- `scripts/stop_sim.sh`


### 2026-11-11 production-source checkpoint resolved

The user explicitly confirmed: (1) the box can be treated as stationary from
pre-grasp acquisition until confirmed suction, and the supplied input xyz is
the box suction/contact point, not its centre; (2) use the default O observer
map; G need not be enabled in this iteration. Preserve the original box
acquisition stamp and the actual confirmation-time TCP stamp independently.
The stationary premise does not authorize timestamp relabelling, unobserved
geometry, changed box identity or replacement of the frozen full-cup evidence.
The observed dimensions, actual top attitude/domain and full-six-joint plan
remain required. A contact-point input must never populate a box-centre field.

Production map adoption for stages0-4 is O-only. The existing G fixture path
may remain explicitly selectable for compatibility, but no G process or flag
is a prerequisite for the default complete-operation entry. One immutable O
lease supplies both surface and volume. Require VALID response generation,
epoch, exact snapshot/config/hull identities and complete bounded pagination;
unknown active volume is blocked. New ordinary map revisions are allowed only
through fresh native checking of the original remaining timed trajectories;
changed epoch/config/model/hull invalidates the original source binding.
Fresh post-release O observations, joined by their real acquisition stamps,
provide observed-map adoption. Planned geometry cannot count as observation.

ST-3-O-SOURCE is the first production integration unit (codex/gpt-6, base
 a9a334189ff9c3ddcab71d669843f454457885f7, dependency ST-4-PREP-OBS).
It replaces the preparation root's unconditional G dependency with default O
lease ownership and an explicit legacy G option. Inputs include the observer's
exact geometry/config files; recompute its config digest using the native
loader's byte contract, never assume a grid resolution. Source identity must
be bound to the produced snapshot and prepared issuance. All calls share the
original request deadline, including source revalidation. Release every lease
on success and failure; uncertain transport retains single-flight ownership.

Acceptance: deterministic ROS tests, one run per named case, AND across
functional cases (not a reliability claim). Actual O observer + actual prepare
root + actual frozen quality and native MoveIt ports must return at least one
complete six-leg plan without any G services/publications. Native O leases
must match surface/volume config/epoch/revision and preserve unknown cells.
Reject non-VALID/reset generation, stale/no observation, mismatched config or
hull/epoch/revision, incomplete/duplicate/nonprogressing pages and late source
change; all reject with inactive plans and zero motion/map-write effects.
Retain explicit G compatibility tests. Pure conversion tests cover x-fastest
indexing, partial knowledge, clipped inactive hull cells, dimensions and
malformed pages; they do not substitute for the actual O-root positive.
Run source-first Humble pytest on test_joint_operation_observer.py and
 test_joint_operation_observer_native.py plus existing preparation/issuance
and frozen-contact regressions after changes. Build changed installs in /tmp,
verify source/install parity. Evidence under
 docs/status/evidence/moveit_atlas_joint_plan/g1/ST-3-O-SOURCE/ (<=128 MiB):
T0 revision/dirty/config/model/executable hashes, exact command and inputs;
T1 per-RPC stage, identity, cursor, clock/stamp and duration; T2 full failed
case original CDR, immutable source files/clouds, scene and native request/
response before cleanup; one case is the bounded pre/post capture window.
Require durable hashes, complete capture and joined workers/native exit0.
Missing native positive or missing evidence is not_evaluated, never pass.

ST-4-INTEGRATION generation1's two source questions are resolved by this
checkpoint. Supersede it with generation2 before dispatching its final whole
operation execution and regression; retain its six positive witness matrix
and negative effect invariants. Do not close stages0-4 on source adoption or
component counts alone. ST5-7 and performance/capacity campaigns remain outside
this implementation scope.


### 2026-11-11 pre-seal contact selection and stationary attachment qualification

User confirmed: before suction the planner may replace input contact xyz with
another point on the same observed box face; after suction the box/tool rigid
relation is fixed. Return final pick pose/J6 and bound complete trajectory before
motion. Never treat contact xyz as the box centre, change G after sealing, or
edit J6 in an issued trajectory. Existing target-side back-projection and full
observed-footprint plus quality gates remain the candidate mechanism.

ST-4-STATIONARY-ATTACH (codex/gpt-6, base2cbe1397bf5ea806b0d59839bcd93d4792bf611a,
depends ST-4-PREP-OBS) qualifies the user-approved stationary acquisition path
before final runtime integration. Preserve original box acquisition and actual
seal-time TCP stamps separately, original full rigid poses, generation, issued
plan and nonrenewing perception binding. Admit only an actual successful
VacuumCommandV2 measured-attachment response and an independently accepted
actual-footprint check from the real frozen observation owner. Its provenance
binds both stamps, original retained observation, actual contact evidence,
response bytes and conservative geometry. No timestamp relabelling, GT, guessed
support, relaxed legacy same-stamp path or mutation of a valid sealed relation.
Keep the existing provider acquisition-age bound; storage retention is not age.

Acceptance: deterministic source-first Humble functional AND gate, one run per
case, not physical reliability. Actual prepare/frozen owner and native complete
plan -> real vacuum boundary adapter with synthetic actuator -> real frozen
actual-contact RPC -> existing attachment provider must produce a route-grade
record with original acquisition stamp and independently preserved later TCP
stamp. Cover centered and allowed pre-seal offset contact; exact duplicate is
idempotent. Refuse missing/failed/unmeasured vacuum, wrong request/scene/box/
generation/dimensions/observation/model, changed or expired frozen evidence,
whole-cup boundary failure, reversed/future/stale stamps, changed actual geometry
or new proof after sealing. Legacy same-stamp provider regression stays green;
legacy unequal-stamp path still refuses. No motion, vacuum release or map-write
port is added to this qualification layer. This is not full ST-4 completion.

Procedure: add test_joint_operation_attachment.py and native coverage in
 test_joint_operation_attachment_native.py; run those plus existing
 tests/st09_parallel/D/test_attachment_provider.py, execution/bindings and vacuum
revision propagation regressions. Reuse isolated Humble/native prefixes and
verified-free diagnostic domain; no hardware/Gazebo. Install changed Python and
verify source/install hashes. Only exact owned files staged; unrelated dirty
scene/atlas retirement preserved. Required evidence under
 docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-STATIONARY-ATTACH/ <=128MiB:
T0 revision/dirty/config/command/workload and hashes; T1 observation retention,
actual vacuum receipt, actual footprint result, provider freeze/refusal and both
real stamps; T2 each failed case plus one upstream/downstream boundary, original
CDRs, frozen inputs, plan and native requests/responses before teardown/rerun.
One case is the bounded trigger window. Verify complete joinable capture and
worker/native exit0; missing real positive is not_evaluated, never pass. Reuse
existing tolerances and acquisition-age policy; no performance claim.


### ST-4-INTEGRATION generation2: actual complete-operation adoption

Source checkpoints are resolved: stationary pre-seal box, contact xyz may change
before suction, sealed G is fixed, current deploy_update geometry, O default,
G unnecessary. Implementation owner codex/gpt-6; baseaca85896aa1c35bae230c4eb6666ccd71eb0943b.
Dependencies ST-4-OWNER, ST-4-PREP-OBS, ST-3-O-SOURCE and ST-4-STATIONARY-ATTACH
are complete. This final integration also carries forward every unclosed
ST-3-ASSEMBLY producer/native acceptance; neither that broad stage nor0-4 is
closed until this whole-chain gate passes. This replaces generation1's source
block and avoids its producer/execution dependency overlap without dropping
producer requirements.

Implement one typed complete-operation execution entry under the actual
MotionPlannerNode's existing MotionOwnership, plus a driver that prepares the
entire chain before any align/J6/pick motion. Goal carries the issued plan and
original measured detection/domain. Independently check the real preparation
owner's original identity, frozen token and original O source. Consume original
six timed trajectories through StrictOperationMotionPort, actual vacuum
request/response, qualified stationary actual attachment and actual frozen
full-cup check. Native-check the entire remaining original path in current O
and full MoveIt scene at each motion/event boundary. No global replanning or
post-seal G/J6 rewrite. Real release precedes released world obstacle admission,
empty retreat and fresh post-release observed O adoption. G remains off.

Current-map snapshots may advance ordinary revisions, including unchanged
classified occupancy. Bind original and current typed O source independently;
epoch/config/hull/model/validity generation changes refuse. A changed current
volume needs native validation of the same remaining knots. Before submission,
require either the same checked map or a freshly verified collision-equivalent
classified volume; bounded retry/refusal if changing obstacles prevent admission.
Never treat GetValidity transition-time version as latest. Preserve ordinary
source association and original issued certificate; current native receipts are
separate evidence. Unknown active cells stay obstacles. No guessed floor free,
planned-slot O commit, timestamp relabelling or unqualified source callbacks.

Retain generation1's six positive matrix: centered, fully contained offset,
directed90, directed180, alternate branch, nonempty ULD. Each must complete
through actual prepare/frozen/O owners and native MoveIt with a synthetic
actuator/controller boundary; at least one complete current deploy_update
whole-chain witness must use the newly qualified atlas. Every original knot,
full six-joint winding and derivative/time remains byte/field identical at the
controller. Global planner calls during execution=0; motion before PLAN_READY=0;
map writes before confirmed release=0; automatic release on carrying fault,
cancel or shutdown=0. Uncertain action transport retains ownership until actual
terminal consumption, and blocks new legacy/operation motion in between.

Negative matrix: forged/expired/revoked owner evidence, changed box/model/cup,
wrong winding/stale q, actual-G target miss, new obstacle, ordinary O update,
new epoch/config/hull, release failure, blocked retreat, cancel/late acceptance/
uncertain transport, stale post-release observation, duplicate completion and
shutdown carrying. Each named deterministic case once; functional AND, not
physical reliability. Refusal stage and forbidden-effect counts must be captured.
Post-release success requires actual O acquisition newer than confirmed release
and settled view, not the planned world obstacle. Preserve existing age/contact/
endpoint/settle/timing policies. No new hardware/Gazebo launch.

Procedure: source-first Humble test_joint_operation_cycle_native.py actual-owner
fixture plus focused changed module tests; unchanged existing native/source/
attachment/strict-motion/ownership regressions once after final behavior edits.
Build generated interfaces and Python in isolated /tmp; prove installed/source
parity. Record exact revision+unrelated dirty files and only commit owned files.
Evidence under ST-4-INTEGRATION/g2/ <=256MiB. T0 commands/config/input matrix,
source/model/atlas/binary hashes, counts and shutdown. T1 every prepare/issuance/
map capture/native check/controller goal+terminal/vacuum/settle/post-release
boundary with real stamps, IDs and state. T2 on any unexpected refusal/error:
freeze that complete case, original plan/detection/domain/clouds, full scenes,
all native/owner/vacuum/action requests/results and raw measured q before
teardown/fix/rerun; one case is the bounded trigger window. Preserve raw CDR for
replay, canonical complete typed fields for identities (native padding
counterexample established in ST-4-STATIONARY-ATTACH/native_01). Assert complete
hash/UUID joins and all workers/native exits. Missing positive or missing capture
is not_evaluated. Do not substitute component counts for full execution.

### 2026-11-12 checkpoint: conditional static ULD, collision exception

User confirms that only the current box moves inside the ULD in normal
operation, and explicitly notes that collision can displace previously placed
cargo. This resolves the environmental premise question, not the detection or
map-consumption policy. Keep generation2's live O/native checks while those
interfaces are reviewed. Do not enable frozen-background execution solely on
this confirmation or alter claimed generation2 requirements in place.

Review the actual ROS collision/protective-stop/controller-error signals and
their freshness before dispatching a replacement map policy. Current runtime
inspection establishes measured joint and action-result checks plus fresh O
capture; it does not establish a connected physical collision monitor. SDK
sample declarations are capabilities to investigate, not evidence of live
robot support or a connected ROS signal. Request user confirmation of the
available real interfaces; do not create force/current thresholds to pass a
case. Visual observations cannot prove that occluded cargo stayed stationary.

Follow-up user answer: joint torque/current are unavailable by default, with
possible later confirmation. Do not wait for them to develop the planning and
map ownership boundaries, and do not depend on them for admission or invent a
proxy contact detector. Availability of collision/protective-stop ROS status
remains unconfirmed. Existing action/joint/O checks remain usable but do not
establish immediate detection of every physical collision.

Required invalidation behavior for that replacement: a collision/protective
stop, execution fault, or observed displacement requests stop, revokes the old
world assumption, blocks the remaining original legs and retains unresolved
motion ownership. If carrying, preserve suction and the actual sealed G.
After confirmed release, never reattach automatically. Recovery requires
fresh observations and independent admission; no automatic reverse-path
retreat. Reset/config/hull/model/clock and observation-age gates remain intact.

Before changing the runtime, dispatch the higher generation with explicit
signal loss/staleness cases, collision while carrying, displaced old cargo,
post-release displacement, and no-fault current-cargo observation cases. Each
deterministic case once, functional AND: no further controller goal after
invalidation, no automatic vacuum release, no unresolved-owner clearance,
and no old-plan resumption. Missing real signal coverage is not_evaluated,
not a claim that collisions are detected. Preserve generation2's complete
positive/negative matrix and deployment witness requirements.

Preserved native_cycle_02 response now identifies four non-support contacts
at empty-retreat sample0 between suction_contact_frame and
placement-prior-occ-voxel-{0,1,2,3}; native_contact_summary.json contains the
decoded contacts. This establishes the rejected collision pair, not whether
the voxel representation is excessive or the physical path is clear. Do not
delete voxels, lower margins or exempt the contact pair on this evidence alone.
