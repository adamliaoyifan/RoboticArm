# ST09 planning convergence — final execution proposal

- parent: FRAME-MIGRATION-20260929
- subtask: ST09
- status: proposed successor; not dispatched
- proposed_owner: eng/cursor/grok-4.7
- revision_date: 2026-10-02
- recovery_base: 2ea75165dd05de4ce042084e300a6b2f725447e3

执行拆分及 QP 保留策略以 `frame_migration_st09_parallel_execution.md` 为准。
本文 S0–S7 作为技术要求和原 ST09 验收基线；并行计划明确覆盖本文的串行开发顺序、QP保留方式和新增三箱里程碑。
原有两箱、capture、fault、proof 义务继续有效。

本文替换同名计划的早期草案，汇总规划收敛、朝向、深处优先放置、异步货图及原 ST09 P4–P6 义务。
当前 Q-20261001-1 generation 5 的认领及 ca8426f 绑定不变。
本文尚未派单；reviews 在同一 ST09 lineage 上绑定下一可用 generation，记录旧 owner 停止确认、精确 base/plan commit 后派单。
生产规划使用 MoveIt 2 + 可达有向图；QP源码、测试和回放保存在独立可恢复实验分支，离线运行并共用认证接口，不成为生产自动fallback。

## 1. 范围、证据和必须纠正的假设

恢复来源：`/tmp/elfin-st09-g3` 以及
`docs/status/evidence/frame_migration/ST09-g5-dirty-checkpoint-20261002/`。
70 条未提交变更、237 文件恢复验证属于检查点证据，不能替代干净候选。
复用 `ST09-g5-rehearsal-52` 和 `ST09-g5-qp-replay-20261002` 的原始失败输入。
不复用其不完整轨迹作为认证通过的缓存边。

- 图中蓝色 atlas 是指定高度、姿态和载荷条件的接触/TCP 可达先验，不能作为箱体中心任意高度的可达域。
  用实测 attachment transform 将目标箱体位姿转换为 TCP 位姿后查询。
  箱体可伸出蓝色区域，但完整箱体必须留在实际 hull 中，且完整机器人路径必须可行。
- 该图开口在容器负 X 侧，深入方向为正 X；通用实现用开口内法向，禁止按图片的正 Y 或世界坐标硬编码。
- revision-2 失败同时涉及候选与真实斜壁不一致、地图观测不足及 atlas unknown 未提供硬拒绝。
  修正 hull 只能剔除非法候选，不能凭空使未观测空间成为 free。
- yaw +pi/-pi、四元数 q/-q 是表示问题。等价姿态不能因字符串或角度数值跳变被拒绝。
  箱体 180 度占地对称不保证偏心抓取、吸盘或机械臂姿态对称；每个目标必须重新通过 attachment 和 FK 校验。
- yaw 变化并非天然非法；在狭窄段固定、净空段连续转换。J6 travel 不设独立经验拒绝门。
  IK family 标签用于搜索和诊断，不能仅因标签变化拒绝一条连续且合格的轨迹。
- 边的载荷包络包含抓取偏移/姿态、吸盘及传感器几何，不能仅比较宽深高。
- 不从“IK 有解”推断存在进入/释放/撤离路径；不从该图推断深放策略已验证。

继承 ca8426f、frame_migration_stages.md 中所有 ST09 验收，不降低 F01/F21–F24、64-case、capture/replay 或隔离义务。
不执行真实硬件运动；Gazebo 只做独占非评分排练。scored INTEGRATION 仍独立派发。

## 2. 最终架构与实现落点

唯一业务链：

```text
实测箱体 + 当前地图快照
 -> 合法 placement 候选及排序
 -> 可达有向图搜索（姿态/关节解为节点状态）
 -> MoveIt 2 补接或生成边
 -> 统一完整轨迹验证
 -> admission + 最新地图重验证
 -> 执行/监测
 -> release + measured verify/integrate/commit
 -> 空载撤离/观测/return
```

| 所有者模块 | 修改位置 | 最终职责 |
|---|---|---|
| 几何公共层 | luggage_description 的现有 hull/frame 模块 | 统一 hull、aperture、位姿变换和具名 margin |
| placement | placement_solver.py、placement_planner_node.py | 当前箱体的支持面候选、完整 hull、深度排序、拒绝诊断 |
| graph/route | reachable_graph.py、grid_path_search.py、trajectory_selector.py | 唯一搜索入口、多关节解、朝向转换、完整进出 route |
| MoveIt adapter | 从 motion_executor.py 分离的 focused adapter | IK、Cartesian/OMPL 边规划、时间参数化，不自行执行 |
| certifier | focused ROS-free policy + MoveIt validity adapter | 完整机器人/载荷、occupancy、FK、关节/时间验证 |
| executor | motion_executor.py、motion_planner_node.py | admission、起点/地图校验、执行、cancel/hold/recovery |
| caller | place_service.py、waypoint_generator.py 和生产 driver adapters | 提交任务约束和消费 route，不拼 fallback 路线 |
| map/observation | mapper、cargo_view_integration、现有观察 driver | 异步观测、版本化地图、有限观察动作 |
| evidence | ST09 verify/extract/freeze/replay 脚本 | 可计算 gate、原始输入、join/checksum、证明 |

不为每个步骤新增 ROS 服务。优先复用已有 v2 端点传递 route/admission 信息；确有字段缺口才扩展 typed v2，全部重建生成消息、刷新受影响证明。
v1 布局不变，v2 profile 不允许 legacy mutation。更新最近 architecture 文档及对应规则时与实现同提交。

## 3. S0 — 恢复、基线、配置锁定

1. reviews 完成旧 owner 停止确认与高 generation 派单；owner 用 agent_start.sh Claim。
2. 在新 worktree 恢复检查点，按文件记录 hash、来源、保留/移除原因；不清理原始失败证据。
3. 记录源码/ROS/MoveIt/控制器版本、CPU、profile、atlas、scene、展开 URDF/SRDF、overlay 和运行命令。
4. 以干净提交构建独立 install，跑当前行为基线。将已知失败按 test node/reason 保存。
5. 输出 resolved_planning_contract.json：起点误差、终点位置/角度误差、关节限制、tool tilt、碰撞 margin、J5 门、时间参数化、地图新鲜度、cancel/hold deadlines。
   从原冻结合同继承全部数值和来源；缺失时在离线 fixture 中定义并校验后提交，禁止在 live 失败后临时放宽。
6. 将后续 workload/case list、命令和必需 artifact 列表写入机器可读 acceptance manifest。

退出：恢复 hash 全匹配、文件 disposition 完整、配置没有未解析值、干净 commit/install 一致。
基线复现失败是定位项，不得冒充候选通过。

## 4. S1 — 修复候选合法性、观测不足与深处优先

### 4.1 合法性

用同一 hull kernel 判定完整 oriented box，矩形 inner_size 仅用于枚举上界。
采用当前 pick 的实测 dimensions、pose、height source 和 attachment；分别校验落姿、开口通过与实际路径，不能因某条直线 insertion corridor 失败就宣称容器已满。
保留两个轴向 placement footprint（0、pi/2）；180 度等价 heading 作为实际 TCP/IK 的额外解处理。
所有候选在 top-N 截断前通过 hull、支撑、overlap、顶部余量；容量不足、观测不足和路线不可行使用不同 reason。
margin 按静态壁、货物、支撑接触分别具名；统一来源而非把所有 margin 强设为相同数值。

### 4.2 首箱优先深放

定义 `depth = dot(box_center - opening_center, inward_normal)`。
先满足所有硬门和完整进入/释放/撤离可行性，再按以下确定性 key 选择：

1. 支撑层高度从低到高，按 surface resolution 分层（保留 floor-first）。
2. 深度从深到浅，按同一 resolution 分桶。
3. 同桶内完整路线最小净空从大到小。
4. 同净空下沿用已冻结 placement score，从高到低。
5. 再按总执行时间、总关节行程从小到大，candidate_id/route_id 稳定打破平局。

深度优先应用到当前箱子的候选集合，不承诺解决全局最优装箱。
禁止为一个更深但超时的候选无限搜索。总体 route 请求默认预算 15 s（2026-10-05 用户将验收时限由 10 s 提高到 15 s）、单次新 MoveIt 边调用上限 2 s、最多 4 次补边尝试；缓存命中不计补边次数。
搜索所有已有图路径可行候选，依上述 key 验证；预算耗尽返回 PLANNING_BUDGET_EXHAUSTED，不冒充 BIN_FULL，也不执行未认证路线。
记录每候选 depth、层高、支撑、进入/撤离可行性、净空、预计耗时和最终选择原因。

### 4.3 地图未知

atlas unknown 和 cargo-map unknown 分开处理；atlas unknown 可触发有界 MoveIt 验证，不能作为 reachable 通过证据。
货图 unknown 保持未知；floor_prior 只遵守既有合法使用条件，不在已有地图被拒绝后恢复为空箱。
可达区域缺乏支持面/空闲证据时返回 OBSERVATION_REQUIRED，执行现有主动观察流程后重新计算候选。
每个箱子最多 2 个预验证观察位（portal aim、第二个可见区域），逐次积分并检查目标区域的可用证据；没有可行 slot/route 则停止并保留不足原因。
优先在 attach 前完成观测和初选；attach 后发现地图失效不能擅自带箱执行空载观察动作，进入 carry-fault/recovery。
观察目标不只是 YOLO mask 非空，还包括待放 footprint 和进出走廊；n_sensor_occupied>=1 仅是原排练最低门，不代表全域已观测。

退出：保存的 24 个斜壁候选全部被拒绝；同输入不产生假合法槽位；新增深/浅对照 fixture 在相同层高且两路均可行时选深处，深处碰撞/不可撤离时选合法浅处或具名失败；观测不足用例不得伪造 free。

## 5. S2 — 姿态与有向图

### 5.1 Attachment 与目标合成合同

统一使用 `T_a_b` 表示将 b 坐标转换到 a 坐标的刚体变换。`T_tcp_box` 是箱体坐标到实际 TCP 坐标的完整平移和单位四元数；TCP 的局部 Z 按真实 link 定义，不能另称“tool frame +Z up”。尺寸沿固定箱体轴记录，不随长短边排序悄悄交换轴。
AttachmentRecord 包含 schema version、cargo/cycle/request ID、attachment incarnation、TCP/box frame、实测 dimensions、完整 T_tcp_box、纳秒 acquisition stamp、scene/calibration identity、观测来源及证据 digest、validity 和拒绝原因。psi 可作诊断派生量，不作为执行合同。

吸附确认后由唯一 attachment provider 校验并冻结记录。使用同一测量时刻的实测箱体 pose 和 TCP TF，计算 `inverse(T_world_tcp) * T_world_box`；记录来源、时间关联和误差界。吸附前的实测 pose 若用于接触时刻估计，必须显式标注估计方法、无滑移假设及验证证据，不能伪称吸附后新观测。没有合格来源时拒绝 place。仿真 backend 的 GT 相对变换只作参考验证，不能直接充当生产 measured attachment。
滑移、重新吸附、frame/calibration 变化使旧记录及依赖它的 route 失效。执行期间按冻结策略检查 attachment 状态；失效进入 carry-fault 并保持真空。测量/TF年龄、同步误差和允许不确定度必须在 S0 冻结并具备边界测试。

纯函数 `resolve_place_targets(box_target, attachment)` 输出完整 TCP 目标及诊断，不读 TF/MoveIt/atlas，不选择路线。目标由 `T_world_box * inverse(T_tcp_box)` 得到，中心高度不再重复加半箱高。供初选的吸附前几何候选可生成；最终带箱 route 必须使用有效吸附记录。
packing 定义容器坐标内两个 footprint 类及完整目标 pose；只有载荷模型声明 180 度对称时，才展开 theta 和 theta+pi 两个候选箱体 heading。它们是不同物理旋转、具有相同对称箱体占地；偏心 attachment 下必须分别计算 TCP 位置和姿态。q/-q、pi/-pi 则是同一旋转的表示等价。不得用四元数符号指示腕部关节族。
不先按当前 tool yaw 只留最近解。图规划以实际 q、限位、整条路线净空及时间评价所有预算内候选；预算耗尽不能宣称所有 heading 不可达。禁止用 pick PCA yaw 改写 place 目标。

### 5.2 Atlas 与图搜索边界

atlas 的 footprint 分类与实际 TCP orientation coverage 分开。两 bin 命中或 pi 周期折叠只能提供有来源标记的先验/seed；不得把 folded seed 视为实际目标的 IK 解。所有 seed 对未改写的真实 TCP 目标重新求 IK、验证 FK 和碰撞。
不匹配采样 yaw 时返回 ATLAS_ORIENTATION_UNCOVERED，进入同一 graph planner 的有界 MoveIt 补接；该状态不等于 orientation_mismatch 或不可达。高置信不可达仅在实际姿态、位置、载荷、attachment 和认证适用域全部覆盖时才能硬拒绝；折叠或域外查询不能传递这个结论。不得硬编码两个 bin 为姿态白名单，也不得通过私有 `_inner` 读取合同参数。
不增加 J5/J6 2.5 rad 或 J2 翻号这样的 seed 硬门；可用于尝试排序和诊断，执行拒绝依据完整轨迹的真实限位、连续性、奇异裕度和碰撞检查。

节点主状态为完整关节解 q，附带 FK TCP pose、区域、载荷状态和 IK branch 标签。同 XYZ 不同 q 不合并。
复用可达 atlas 的多 seed 和已保存且可验证的 portal/raised/exit 状态；从小图开始，不重建全空间 atlas。

- placement_yaw 是箱体最终朝向；transport_yaw 是箱体运输朝向。
- 全程由 `T_world_tcp = T_world_box * inverse(T_tcp_box)` 得到末端目标。
- portal/狭窄 traversal 保持已选箱体朝向；insert/descend 保持最终朝向。
- 外部或内部净空区允许受认证的连续 orientation-transition，旋转可伴随必要平移，不强制原地转或只能在舱内转。
- 对任意转换，随轨迹计算实际 oriented payload，不能固定 footprint 扫描。
- 四元数按旋转等价比较；关节绕回必须遵守实际 limit，禁止盲目取最近 2pi 解。
- family 标签改变触发额外检查，不构成单独拒绝原因。真实关节跳变、配置的奇异裕度失败或碰撞才拒绝。

每条边有 source/destination q、方向、载荷/空载状态、静态身份、完整 timed trajectory 和验证结果。
进入和释放后空载撤离分别认证；释放后的箱子变为场景障碍。
长期图持久化静态身份；动态 occupancy 只附加本次 validation revision。
仅相同抓取变换或严格被认证包络覆盖的载荷可复用边；否则重新认证，不能仅凭尺寸更小放行。
MoveIt 2 是唯一边生成器，Cartesian 与 OMPL/RRTConnect 是内部手段；多个 seeds/goals 的尝试结果都归入同一 route。
缓存边衔接、局部补接、时间参数化之后验证最终拼接轨迹，禁止分别合法但连接处速度/位置不连续。

退出：0->0、0->90、90->0、near-square、偏心 attachment、q/-q 与 pi/-pi 等价表示、多 IK family、无可行转换区域共 8 类 fixture 各有正/负例；正确接受或具名拒绝且无旁路。

新增合同验收：sim/deploy 各跑 centered、eccentric、非零相对 yaw、允许范围内 tilt 四类 target fixture，独立矩阵期望的中心/八角点误差<=1e-9，ROS 往返<=1e-6，身份和纳秒完全相等。q/-q 输入输出旋转相同；theta+pi 的偏心目标 TCP 必须与独立期望一致。缺失/NaN/非正尺寸/非法四元数/过期/错frame/错request/重新吸附/GT伪装measured各1负例，全部具名拒绝且无新增业务 effect。atlas未覆盖姿态各含可解及不可解 mock 正负例；可解者不得因bin缺失被拒，预算超时明确记 PLANNING_BUDGET_EXHAUSTED。所有必需样本AND通过，缺样本为not_evaluated，不据fixture通过宣称live可靠率。

## 6. S3 — 统一认证与异步地图

认证输入为不可变 snapshot、完整 robot/payload 模型、最终 timed joint trajectory、request/admission identity。
MoveIt scene 检查完整机器人自碰撞、静态容器、attached payload、已放物；独立 occupancy 检查货图 occupied/unknown 与实际 FK 后的机器人/载荷几何。
TCP 半径或几个点的 sweep 不能代替完整连杆碰撞；保留简化 sweep 仅作保守预筛。

碰撞采样同时受时间和空间约束：时间间隔<=10 ms，几何运动界<=min(map_resolution/2, smallest_positive_clearance_margin/2)。
使用可靠运动界自适应细分或保守连续碰撞检查；无法界定采样间隙则拒绝认证，不能只采 knots。
认证输出明确检查过的范围和方法，不把离散样本宣称为数学上的连续安全证明。

动态地图逻辑：

1. mapper 异步发布；规划固定 snapshot，不持锁等待运动。
2. 接受请求、发送控制目标前核验最新 revision/geometry、真实 start joints、attachment、runtime lease/incarnation。
3. 新版地图若不影响未执行边，重认证并记录 validation revision；有影响则 mask 边、在同一 planner 内重搜索。
4. 若地图更新发生在控制器执行期间，监测未执行轨迹及停止扫掠；相关冲突触发 cancel/受控停止、确认静止并保真空，进入 carry-fault。
5. 实测并冻结 detection/cancel/stop latency 与停止包络；测试障碍须在该可控提前量之外注入。响应超 deadline 或停止包络无法验证，判失败；不声称任意突发障碍都可安全停止。
6. 地图 revision 不变不代表新鲜，使用采集时间和 receiver monotonic 时间分别检查观测年龄与 lease。
7. controller send 前以 map/scene validation epoch 复核；变化则重试有界验证。执行中由监测补足 check/send 竞争，禁止声称单次读取消除了竞争。

landing/support 接触豁免只覆盖目标支撑接触，不能删除整列障碍或相邻已放箱体。
恢复 readiness 不自动续跑；重新生成 admission。operator start、request correlation 保留。

退出：无关更新通过；相关更新换路；全部封闭拒绝；执行中冲突停止并保真空；stale/hash/restart 拒绝。
执行中故障以故障后的 effect delta 验证（允许必要 cancel/hold），不能要求整轮零运动；pre-pick/pre-place admission 拒绝后规划/执行/attach/scene/map 业务 effect delta 为 0。

## 7. S4 — 移除旁路，接通完整生产链

在新 worktree 完成替代入口和测试后删除：geometric 直接执行、QP 生产接线、executor 内部 seeded-hop/raised-via fallback、driver 自建 exit、绕过 selector 的 OMPL fallback。
已有几何函数可保留为候选/图节点生成工具，但不得直接获得执行权限。
只有统一 planner 能输出路线；executor 不再改写目标、姿态、关节轨迹或调用另一规划器。
不固定新增 yaw_align 原地旋转或 transit Cartesian 折线作为执行旁路；其位置和方式由统一 edge builder 选择并认证。失败后 executor 只 cancel/hold，保真空并进入 carry-fault；若需要恢复，由统一 planner 从实测停稳 q、最新地图和 attachment 生成新 admission，不能自动发送回 outside-high 的关节目标。

迁移 hardware driver 和 pick-retreat、place-smoke、place-only、pack、cargo-map-integration、motion-occupancy 七类 caller。
BuildMotionSequenceV2 pick/place 必须给出完整 typed motion_segments；保留既有 GoToRobotPoseV2、scene、vacuum、map v2 admission 和 release 前 measured-transform 验证。
所有 production 运动（含观察、撤离、return）经统一 route/admission；观察允许其明确的朝向约束，不套用带箱 tool-down。

完整 route 保存 request_id、payload/attachment digest、scene/robot/graph identity、planning revision、每次 validation revision、slot、真实箱体与 TCP 姿态、节点边和完整轨迹。
沿用 typed API；如必须扩展则记录 affected endpoint inventory、重建所有消费者并刷新证明。拒绝旧 layout 混装。

退出：七类 caller 各独立通过适用的 64-case adapters；任何 legacy/QP/direct-controller 绕路行为测试都被拒绝，零新执行副作用。

## 8. S5 — 离线与 ROS graph campaign

补全现有 runner，不能靠 source-text 搜索或硬编码 missing list 判通过。
统一 exit：0=完整成功，1=断言/运行失败，2=覆盖不足。
接受输出目录必须新建；required sample inventory 与 artifact manifest 联合计算 coverage。

| Gate | workload / repetition | 判据 |
|---|---|---|
| 几何/frame | established 64 cases，每个适用边界和 caller 独立跑 | centers/8 corners/normals/grasp，纯几何<=1e-9，ROS<=1e-6，identity/ns exact |
| 深放 | 两朝向×两场景×深处可用/被占/不可撤/unknown，共16 cases | 层高相同优先深，硬门无违规，unknown 不冒充 free |
| yaw/IK | S2八类，每类正/负至少各1例 | 姿态/附件一致，真实跳变拒绝，表示等价不误拒 |
| map | S3六类（无关/相关/全封/执行中/陈旧/重启），sim/deploy各1 | 对应通过/改路/停止/拒绝，delta 合规 |
| startup | sim/deploy各3次，mock hardware | <=180s READY，<=90s teardown |
| common negatives | unknown schema/conflicting datum/mixed types/missing peer/stale atlas/missing URDF/duplicate provider/wrong overlay/restart，每profile各1 | 具名拒绝和无禁止 effect |
| mutations | wrong-frame/position-only rotation/dropped velocities or times/legacy bypass/QP activation，各1 | 行为 gate 检出，不是检查源码字符串 |
| 性能 | 保存的可行图 fixtures，同机10次warm请求、1次cold | 每次<=10s完整 route 或具名预算失败；已定义健康fixture必须10/10找到认证路线，记录p50/max与MoveIt调用数 |
| regression | tools、ST08、contract及affected packages，base1795a69与candidate相同环境 | 仅原packing exact node+reason相同失败可豁免，其余全过 |

聚合为全部必需case的AND，要求测试集内100%正确；3次startup和10次fixture只证明观察到的重复性，不估计总体成功率。
正例必须正向成功，不能靠所有输入均拒绝来过 gate。负例不能用已失败的前置步骤代替真正触发目标故障。

命令模板（CANDIDATE_WS 和 EVIDENCE_ROOT 在 run manifest 固定为绝对路径）：

```bash
source /opt/ros/humble/setup.bash
cd "$CANDIDATE_WS"
colcon build --build-base build-st09 --install-base install-st09
source install-st09/setup.bash
bash scripts/frame_migration_st09_verify.sh offline --ws "$CANDIDATE_WS" --install "$CANDIDATE_WS/install-st09" --out "$EVIDENCE_ROOT/offline" --ros-domain-id 87
bash scripts/frame_migration_st09_verify.sh graph --ws "$CANDIDATE_WS" --install "$CANDIDATE_WS/install-st09" --out "$EVIDENCE_ROOT/graph" --ros-domain-id 87
```

runner负责两profile及所有suite，必须输出实际命令、node IDs、sample数量和结果。
若现有CLI/runner尚不具备所述能力，补齐属于S5交付，不可用手工摘要代替执行。

## 9. S6 — P4/P5/P6原有排练、采集与发布义务

离线和graph通过后冻结干净实现commit及fresh install，才开始Gazebo。
独占domain7，GUI/RViz off，persistent startup probe，predecessor-relative deadline，每session<=20min，startup<=180s、teardown<=90s。
保留失败记录；修复源码/配置后使用新不可变candidate和新目录。

1. 一次完整在线 `carryon->standard->carryon` 三箱健康排练；不硬编码 candidate ID、不要求先深后浅，每箱基于实测尺寸和当前 OCC 重新生成候选。前三箱中的前两箱仅在 workload、配置、事件与采集窗口完全一致时满足原两箱样本，避免重复 live run。
2. pre_pick、pre_place各一次故障排练；前者无业务effect，后者保真空并经明确recovery，未授权不得detach。
3. 每箱GT：XY<=0.04m、abs Z<=0.03m、drift<=0.02m、abs roll/pitch<=2deg、center inside hull、footprint coverage>=0.40且分母>0。
4. 三箱均完成release、measured integration、map publication与return；无lost payload、0 floor-through、至少1 sensor-occupied cell；第二/三箱分别使用前箱积分后的revision且已验证地图足以支撑其route。
5. 事件用request/route/causal sequence关联pick execution、attach、place execution、release、integration、map publish、return/recovery；采集时间单独保存，不以不同clock timestamp排序。
6. capture pre=ceil(2*max upstream latency)，post=ceil(max downstream completion latency)，每条必需链路至少1正样本，否则not_evaluated。
7. domain42三对capture off/on replay，同bag、同输入、交替顺序、相同资源；每对记录成功完成单位/时间，median(1-throughput_on/throughput_off)<=10%。失败/丢数据不能算吞吐改善。
8. failure replay重现同一拒绝码及effect counters。固定记录轨迹可确定性重放；重新跑OMPL不要求字节相同或边序列必然相同，要求安全不变量、任务结果和预算通过，并记录seed/build。
9. exactly one /clock publisher，所有必需stream joins/checksums完整，lossless replay <=1GiB/live run、<=4GiB该campaign合计。原历史失败不删除以腾预算；空间不足先阻塞新run。
10. F23 dump统一versioned envelope，measured与GT分离；F24 fresh install、ament executable index及全过程sim/deploy graph均无legacy executable/node/connection。
11. 候选级确定性失败必须具名记录并在同一请求内自动换点；至少一条行为证据证明首选路径失败而后续候选成功。请求级失败须列出 attempted/untried 和逐候选原因，不能由单个候选耗尽预算后笼统返回。
12. 本轮冻结 `same_layer_clearance_m=0.05` 并进入 runtime/freeze hash，未来改值必须新 generation。每个同层旋转 footprint pair 的实测边界距离达到 0.05 m；叠放只对明确 support parent 豁免水平间距，并记录 parent、support coverage、top clearance 与无穿透。冻结 candidate ID 或预规划轨迹不能作为健康排练输入。

排练CLI保留现有 --fault pre_pick/pre_place；删除 --carry-planner geometric/qp选择，生产仅graph。
freeze/replay脚本必须接收显式 --install，不可偷偷source旧默认install。

证据矩阵：

| Boundary | T1 / trigger | T2 payload | window / format / replay |
|---|---|---|---|
| sensing/map | stamp/join/revision；VIEW_EMPTY/stale/unknown | RGB/depth/mask/cloud、TF、原map/commit | 上述实测pre/post；bag+NPZ/JSON；mapper replay |
| placement | size/hull/depth/reasons；无候选/越界 | 完整snapshot、所有候选、margin与attachment | 同窗口；JSON；placement replay |
| route | graph/seed/yaw/family/budget；无路/timeout | 图digest、MoveIt scene/request、尝试边及完整trajectory | 同窗口；JSON+bag；route replay |
| execution | admission/start/revalidation/effects；cancel/collision/restart | joint/controller/scene/remaining route与stop trace | 同窗口；bag+JSON；controller mock replay |
| release/integration | causal IDs/GT metrics/revision；verify或join失败 | measured transforms、独立GT、观测及commit | 同窗口；frame_dump/2+bag；extractor replay |

每个bundle manifest记录实际replay命令、hash、缺项和capture_complete/replay_possible；未齐全先补采集，不能盲目再跑。
冻结脚本将测得窗口和views纳入配置hash，capture开关只允许改变采集开销，不改变业务路线输入。

## 10. S7 — 最终关闭与独立INTEGRATION交接

执行命令：

```bash
bash scripts/frame_migration_st09_verify.sh all --ws "$CANDIDATE_WS" --install "$CANDIDATE_WS/install-st09" --out "$EVIDENCE_ROOT/all" --ros-domain-id 87
python3 scripts/check_frame_contracts.py --mode migration
bash scripts/check_agent_contract.sh
```

all负责顺序offline->graph->独占domain7 rehearsal->domain42 paired replay->final proof/release gates，内部domain不得继承错到Gazebo。
支持复用同candidate/config/install/hash的完整已通过证据以避免无谓live重复；否则新目录实际执行。缺任何必需sample均exit2。
刷新所有受影响的F01/F06–F24证明，F01/F21–F23 verified、F24 isolated；proof/checker 复算候选尝试/拒绝、同层旋转 footprint gap、stack parent 支撑与无穿透，两个checker退出0且无pending/stale。
源代码、消息typesupport、graph/certifier配置、观察位、scene/atlas、dump/extractor、expected events及安装artifact全部hash匹配。

更新INTEGRATION g2的候选来源为最终被接受的高generation head；删除其允许旧seeded-hop/executor fallback的事件规则，改成route-owned边规划/重验证事件。
不原地重写已认领的INTEGRATION合同，未派单计划由reviews绑定新plan commit。
ST09 owner记录commit、proof hashes、冻结manifest、生命周期结果并agent_complete.sh关闭。
scored INTEGRATION单独派给既定eng/claude/glm-5.3，合并由reviews/用户决定。

## 11. 提交与验收顺序

S0恢复基线 -> S1几何/深放/观察 -> S2图与朝向 -> S3认证/异步地图 -> S4七类caller与删除旁路 -> S5离线/graph -> S6排练/capture -> S7证明/关闭。
每步独立可review提交，代码与近邻文档及行为测试同步。禁止用QP测试通过代替路线认证、用单箱成功代替两箱map闭环、用文档checker通过代替ST09发布。
第一次live前必须完成S1–S5；blocked-knot SQP调优不在生产收敛关键路径。
