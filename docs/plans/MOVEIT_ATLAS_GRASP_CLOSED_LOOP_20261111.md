# MoveIt atlas 采样、抓取偏置与路由消费闭环设计

- status: discussion_draft
- dispatch_ready: no
- date: 2026-11-11
- base_revision: 599bf289f66d9474b8117eb47f6d58c40d00587f
- scope: MoveIt；离线采样、抓取前联合选择、抓取后精确路由、成功经验复用
- authorization: 本轮仅设计与讨论，不改产品行为、不采样、不运行仿真、不派发实现

## 1. 用户约束与目标

1. 按空箱建模；本轮不增加偏心距上限、抗倾覆力矩或质量/质心标定任务。几何偏置范围由完整吸盘在顶面内的约束决定，不代表箱子碰撞几何可以删除。
2. 整个吸盘在接触面上的投影必须完全包含在同一个箱子的有效顶面内，否则不得作为抓取候选。部分覆盖率不能替代完整包含。
3. 使用当前完整工具和传感器碰撞模型；保留现有接触质量、支撑、内腔、间距、保守包络与完整轨迹认证。
4. 箱心可比 TCP 工作区更深入；偏心抓取优化发生在抓取前。抓取后实际附件变换冻结，不允许修改附件来使目标变得可达。
5. 优先保持期望箱心；扩大抓取选择和 IK/通道选择，不默默把目标改成更浅位置。目标可接受区域/容差及失败后的目标让步策略仍需讨论。
6. 后续夹爪通过不同抓取候选提供器接入相同工具—物体变换接口；工具模型变化使依赖该模型的 atlas/轨迹身份失效。

## 2. 现有基础及必须修正的消费缺口

- 现有 update atlas 有 paired contact/transit seeds、transit_target、transit_lift、opening_connected、joint_margin、manipulability、neighbor_confidence。
- 生产 atlas prior 主要提取 contact seeds；初始 portal/approach 使用当前状态及固定 tool-down seeds，未完整消费 transit 对及其目标。
- builder 在传播已成功时不强制探索其他独立分支；max_branches=3 是保留上限，不是完整性证明。
- 最高两层中心 1.775/1.925 m 无 reachable/marginal；builder 有 container-absolute 1.70 m 接触截断和默认 1.55 m carry cap。必须记录限制来源/适用部署；不能把策略未采样写成物理不可达。
- 现有吸附配置为 0.18×0.18 m 保守 CAD footprint、边界余量 0.015 m、10 mm 候选格、5 mm 高程格；它不是已标定的密封环尺寸，本计划不缩小它。
- evaluator 的 mask coverage=0.95 不足以证明完整包含；现有 margin 检查也不能代替整个吸盘区域的包含判定。需新增独立完整包含门，保留其余质量门。
- evaluator 当前最多保留5个候选，扫描上限400且满足候选数量后可提前停止；中心距离/边界净空排序可能使目标有用的偏心点未被扫描或保留。
- 有 GraphStore 和请求内边复用，但生产 factory 没有持久经验加载；历史 candidate 经 adapter 仍重新规划。候选图清理必须保留，经验库存储另设边界。

## 3. 统一几何与参考系

记 C=container_link，B=实测箱体有向坐标，G=suction_contact_frame。T_AB 把 B 中坐标变换至 A。所有新 atlas 坐标采用 container-absolute z，所有 size 只表示尺寸。实现前将新消费者加入工作区 frame/height 清单并同步架构文档。

抓取前候选保存 G_B=T_BG（箱体中的工具姿态）；抓取后从确认附件读取 T_GB，二者互逆。

目标工具姿态：T_CG*=T_CB* T_BG=T_CB* inverse(T_GB)。必须复用现有目标合成契约，不能出现第二套目标数学。

平放箱的接触原点在箱体系的平移 r_B=(u,v,h/2)，则 t_CG*=c_C*+R_CB*r_B；反向得到 c_C*=t_CG*-R_CB*r_B。这里 c 是名义箱心，不是保守包络中心。真实顶面/工具姿态使用完整变换，不能把所有检测强制成这个简化式。

若容器深度方向为 d_C，且 R_CB*r_B 的水平分量沿 -d_C，则 TCP 比箱心更靠近开口，箱心可向深处多伸出该偏置。偏移方向由最终放置朝向反算至抓取箱体系，不能硬编码成世界X或当前取件台上的“近机器人侧”。

例：目标箱心沿深度坐标为0.90 m，选择 r 的深度分量=-0.12 m，则所需TCP深度为0.78 m。两者共享同一把持关系，不是在放置时滑动箱子。该例只是几何算例，不声明此机器人的可达界限。

工具四元数来源仍由 candidate waypoint builder 的 attach target 生成；不能将感知 outward-normal quaternion 直接当TCP朝向。0与π抓取分支保留为不同有向状态。

### 3.1 整个吸盘的包含域

在箱顶面二维坐标中，P 为同一观测的有效顶面区域（保留凹口、孔洞和未观测区），F_phi 为按相对抓取角phi旋转后的整个吸盘投影，M 为现有边界余量形状。

U_phi = P erosion (F_phi Minkowski-sum M)。只有 (u,v) 属于 U_phi 才允许继续接触质量检查。顶面外框或四个角点检查均不能替代非凸P上的完整区域包含。未知区域不授权通过。

实现可用保守栅格腐蚀：将完整投影及margin覆盖到的所有格子作为 footprint，所有这些格子必须属于已确认同箱顶面；报告离散化带来的保守损失。也可用权威多边形的差集面积为零及边界判定，但不可用凸包填平未知孔洞。

理想轴对齐矩形顶面L×W，矩形吸盘a×b，边界余量m时：|u| <= (L-a)/2-m，|v| <= (W-b)/2-m。任一上界<0即该朝向无可用吸附点。

旋转矩形的投影半宽为(|a cos phi|+|b sin phi|)/2、(|a sin phi|+|b cos phi|)/2；这只能给外框必要限制，实测顶面完整包含仍须独立判断。

例：L=0.70，W=0.45，a=b=0.18，m=0.015，则理想几何范围|u|<=0.245、|v|<=0.120 m。实际域还受观测有效区域及原有表面质量门限制，这不是统一授权偏心上限。

归一化坐标用于比较和检索：eta_u=u/u_max、eta_v=v/v_max；零宽退化轴单独处理，不做除零。不规则顶面仍以U_phi为真，不能以|eta|<=1冒充可行性。所有执行/碰撞计算用米和完整SE(3)。

## 4. Atlas 分层数据设计

不把箱长、宽、高、偏心距作为静态atlas全部枚举的维度。工具静态图保持尺寸无关；实测箱体和抓取变换构成在线附件层；真实成功路线形成经验层。

### 4.1 Artifact identity 和采样策略

保存schema、单位、坐标/高度语义、机器人/工具/容器几何哈希、关节顺序与限位、ACM和padding摘要、碰撞检查器版本、边插值策略、方向集合、采样分辨率、seed策略与预算、每项高度限制及其来源。changing tool/padding/ACM不得沿用旧碰撞结论。

### 4.2 每个实际采样位姿

保存pose_id、精确position_container和directed quaternion、索引格与局部细化层级；不能只有格中心和一个status。

保存尝试/完成计数、各失败阶段、政策未采样原因、NOT_EVALUATED、所有保留分支ID。将“没找到IK”“碰撞”“未连接”“未评估”分开；有限采样失败不得声称全连续空间不可达。

### 4.3 每个IK分支节点

| 字段 | 作用 |
|---|---|
| q、joint_names、winding、branch_id | 真实关节解；尊重限位和关节绕圈，避免把π朝向或受限关节的2π当成同一解 |
| FK目标/残差、IK求解条件 | 精确目标修正与诊断 |
| contact/transit配对ID、实际transit_target/lift/kind | 从落点反向找到采样时实际可用的接近姿态，不假设都是正上方0.3m |
| q限位余量、Jacobian sigma_min或条件信息、manipulability | 分支排序；定义尺度和计算方式，不能混用不同量纲 |
| 静态arm/tool最小距离与碰撞对 | 排序及局部加密依据；不冒充载箱净空 |
| 连通分量、入/出边ID、可达portal集合 | 区分端点可解与接近/退出可接通 |
| seed来源与耗时/尝试计数 | 评价传播和独立分支探索是否有收益 |

分支等价不仅依赖0.25rad距离：记录数值去重、关节winding、路径连通证据。相近q是去重提示，不是拓扑等价证明。默认优先保留最多6个有区别且有价值的分支，作为待基准比较的采样配置，不是已批准运行时预算；记录被截断数量。

### 4.4 每条有向边/运动原语

保存from/to节点、边类型（joint/Cartesian）、完整可重放q序列、对应工具路径、固定姿态/直线/跳变等检查条件、碰撞采样精度与结果、最小净空及位置、关节行程、时间参数化策略/是否已定时、建边耗时、静态身份。

显式保存 predecessor/branch 连接，不只保存opening_connected布尔量。q直线插值无碰撞与笛卡尔直线插入是不同原语，不能互相认证。卸载退出方向单独记录；反向边必须在其约束下检查，不能默认反转时间戳即可执行。

每条边允许附保守扫掠AABB/分块包络作为动态障碍筛选提示。包络重叠只表示需要精查，不能当成真实碰撞；无重叠也不能免除未包含的机器人或载荷检查。

### 4.5 在线附件覆盖层

键包含 measured dimensions、完整T_GB、工具/模型/场景/地图/ACM/padding身份、原语及检查策略。保存节点载箱有效性、边载箱有效性、拒绝碰撞对/位置、认证覆盖与耗时。

尺寸相同但偏心不同不能命中同一有效性结论。近似匹配只能复用种子或轨迹提案，必须重新检查。负缓存受相同约束；旧障碍下的失败不能跨新场景永久拒绝。

### 4.6 成功经验层

独立保存完整认证通过的载箱/卸载轨迹段、精确端点、约束、附件/场景/模型身份、认证与执行结果标签（计划成功和执行成功分开）。不往当前请求图永久堆积节点。

载入后默认为candidate_only。优先验证旧轨迹，再尝试起终端连接，失败再规划。旧轨迹验证不调用同一个“读缓存再全量重新规划”的路径。复用原有certifier并检查插值内点、时间参数化与当前场景；状态检查API本身不等同完整运动认证。

## 5. 离线采样流程

1. 冻结模型/场景/策略身份，保留当前artifact为基线。先校准坐标与高度限制；策略限制不得变成永久负结论。
2. 保留15cm/4有向yaw作为粗层。样本范围覆盖实际接触与release目标、portal/approach及允许入口外侧；外部网格范围由当前有效场景确定，不从容器box内部尺寸推导。
3. 开口从单一固定点改为允许区域的多个portal候选。每个portal从多类seed求IK，并检查相应静态连接；真实在线起点仍需连接，不能用离线home替代。
4. 波前优先传播上一格全部保留分支；即使已有成功，也在分支不足/边界/高需求格尝试独立seed，避免一个易解分支垄断。各尝试有固定上限与失败原因。
5. 对contact/transit配对构建真实原语并记录边；保留高低接近高度备选，不强制所有高层目标使用统一0.3m升高。
6. 选择性细化：高查询频率、粗格精确refine失败、缺少portal连接、可达边界、低静态净空等区域，按15→7.5→3.75cm细化。数值是首轮讨论配置；不承诺细化消除几何不可行。姿态新增只针对实际合法目标方向，第一阶段不扩展tilt任务合同。
7. 运行图完整性检查：每个边端点FK一致、引用可解析、原语约束独立验证、所有采样空白可解释、未评估不伪装碰撞。
8. 新artifact原子发布并保留旧版本，线上只读。在线失败写到证据队列，由离线定向补采产生下一版本，避免在线一次失败污染全局atlas。

## 6. 抓取前联合选择算法

### 6.1 目标不变，先反推有用抓取域

1. 从实测地图生成当前箱体的合法箱心/朝向候选及目标优先级，包含期望深位区域。严格几何可放集合记为P_box。
2. 感知提供顶面有效域和完整包含域U_phi的只读身份；planner提出希望验证的抓取位置，仍由唯一SuctionPatchEvaluator完成完整包含和原有质量判门，返回新的候选ID。
3. 对期望箱心c*与合法箱体朝向R，检索附近具有通道连接的atlas TCP位姿t，计算r_B=R^T(t-c*)。只把落在合法顶面高度和U_phi内的抓取点送去精确评估。姿态需同时匹配完整T_BG，不能只算XY平移。
4. 同时保留原高质量候选与合法域方向极值候选，避免atlas稀疏导致只有一条反推路线。每个候选点只评估一次；不把附近已接受点平移后冒充原候选。
5. 每个已接受grasp生成目标TCP，再从所在/相邻细格取得多分支、paired transit及portal连接。局部样本是seed，精确refine后重新检查，不能把箱心吸附到格中心。

候选盒中心可达先验：对固定抓取r和朝向R，P_hint(r,R)=P_box intersection (R_tcp-R*r)。抓取前可对已验证grasp取并集；抓取后只保留实际抓取对应的一个集合。R_tcp是分支和连通性条件化的采样先验，不是可执行集合；unknown后备单独保存。

### 6.2 有界候选供应与排序

改成task-aware分页/会话：保留少量当前候选消息，支持按期望深度方向请求新增已验证patch，不直接放大全部消息。候选预算与实际pick尝试次数分开；本计划不自动增加现有最多3次pick尝试规则。

初次廉价抓取提案建议覆盖：原高质量点、向最终期望深位的有利偏置、反向偏置、两个横向方向；用U_phi真实边界限制，避免一律极限靠边。方向提案须在扫描提前停止前取得评估机会。

第一轮对最多8个目标—抓取组合做廉价精确IK/端点检查，保留最多3条不同分支/通道的路径提案；失败后按同一deadline续取。8/3是讨论默认值，不能用上限耗尽宣称空间不可达。若超过预算，显式SEARCH_INCOMPLETE。

排序采用分层而非将危险余量和距离混成一个任意权重：硬门先行；目标区域/用户允许误差层级优先；同层优先完整接近和退出链、载箱有效性及与当前状态的衔接；再比较预期规划/执行成本、IK余量、接触质量和偏心大小作为同分偏好。用户本轮没有机械偏心上限，不能另加经验阈值。

### 6.3 Pick—place绑定与冻结

预选项绑定 box observation、candidate_id、T_BG、box目标、模型/场景身份和提案路线。先确认pick接近/吸附/抬起能够实现该抓取，不因放置侧好而忽略pickup侧。

抓取前所有载箱计划是预测附件下的provisional计划，不是已测附件认证。抓取成功后以控制器确认的实际T_GB重算目标TCP、节点和路线；若偏差导致原方案不再可行，在实际抓取下搜索其他路线/目标，而非改写附件。重新抓取必须进入显式新抓取周期。

## 7. 抓取后路由消费闭环

1. 按最终箱心、实际attachment和release策略生成准确TCP目标。分别记录settled_box_target、release_box_target和对应TCP，避免contact/release查询错位。
2. 查询返回有结构的AtlasProposalSet：精确样本pose、多IK分支、paired transit、实际transit_target、连接图、样本身份、置信/覆盖信息和unknown后备。不能继续只返回status+扁平seeds。
3. 优先精确refine目标contact分支，沿已存配对/边反向构造approach/portal；每个锚点用与最终认证一致的当前载荷场景检查。不得重新把所有锚点退回固定tool-down种子。
4. 分支被拒只淘汰该候选；尝试同位姿其他分支，再尝试允许区域其他transit/portal。修改锚点不改变指定箱心。保留失败位置和碰撞对用于定向补采。
5. 原语选择：有效经验轨迹验证→静态atlas原语加动态载荷检查→短段连接/修复→现有MoveIt搜索fallback。保持共享deadline和失败memo。
6. 载箱进入、下降、释放后退出、回位整体认证；释放后的箱体作为世界障碍。不能用无载荷atlas清空它。
7. 成功结果进入有界经验库，失败按阶段写证据。未知/超时/被预算截断不写成永久不可达。

当前release gap会使“释放位置正确”和“落稳箱心正确”成为不同验收项。先保证放置目标与TCP变换精确；物理落稳位置仍用现有观测/commit确认。本计划不为了数学闭环偷偷取消release gap，也不声称仅靠刚体变换保证自由落体后的精度。

## 8. 模块与接口边界

| 模块职责 | 输入 | 输出 |
|---|---|---|
| 顶面/吸附候选提供器 | 同观测顶面、完整接触轮廓、待测点 | 真实accepted grasp及完整包含/质量证据 |
| atlas artifact reader | 静态身份、工具目标 | 结构化分支与图邻域 |
| grasp-placement proposer | 合法箱心域、accepted grasps、atlas | 目标—抓取—通道提案，全部candidate-only |
| atlas route adapter | 精确目标、实际附件、proposal | 候选graph节点/原语及provenance |
| MoveIt checker/certifier | 当前场景、完整机器人/附件、轨迹 | 统一有效性结果及失败位置 |
| experience store | 认证/执行结果、完整身份 | 有界历史提案；不拥有当前请求图 |

RouteProducer保留编排，不能继续把采样、吸附几何、缓存键和图搜索全部塞进同一文件。复用现有frame、attachment、envelope、solver和certifier单一实现。

## 9. 验证设计与可计算指标

本节是讨论版验证框架，不是已批准验收派单。实现派单前固定fixture路径/哈希、owner、plan revision、环境、deadline及最终阈值。不得沿用“有函数/源代码含文本”作为接线验证。

### 9.1 确定性功能门

- 变换矩阵：4个箱体有向yaw × 中心/+u/-u/+v/-v五个抓取 × 两个箱高，共40个理想算例；正反变换误差<=1e-8m/rad。另加非单位container姿态、偏心π分支、名义/保守中心不等三个类型。数值门是代数正确性，不是物理放置精度。
- 完整包含：矩形内部、越界1mm、边界恰好满足配置余量、吸盘大于箱面、旋转45度、顶面凹口、内部未知孔、95%覆盖但局部越界、跨两个箱面等固定用例。只有完整包含且原质量门通过的候选被接受。
- 分支消费：首分支载箱拒绝、第二分支通过；contact可解而原transit失败、另一paired transit通过；不同抓取同尺寸不可共享validity缓存；每个预期分支必须出现在实际checker输入，而非仅写在诊断里。
- 阶段正确性：抓取前候选不可冒充实测attachment；抓取后不再取不同grasp集合的并集；release与settled目标分开；卸载退出保留刚放箱障碍；未知atlas不被永久拒绝。
- 缓存：当前场景验证通过时不重新调用全局建边；新障碍、附件偏移、工具hash、ACM/padding变化触发重验或失效；无有效缓存时走原MoveIt路径；有界淘汰与请求清理不互相破坏。
- 所有必要确定性用例一次、全部通过；这是有限不变式覆盖，不表示随机运行的每单位成功率。

### 9.2 采样有效性和延迟对比

采样效率分别报告：每CPU秒新增有效独立分支数、每MiB可重放连通边数、精确目标refine命中率、载箱锚点通过率、完整路线成功比例。记录各分母与未评估数，不能用reachable比例代理全部指标。

回放应分两组：A组固定已抓附件比较纯atlas消费/缓存；B组抓取前联合方案允许不同grasp，必须另外记录pick路径和预测/实测附件，不能把A组固定附件回放冒称偏心抓取收益。

建议冻结24个输入：3个深度带 × 2个占用等级 × 2个实测尺寸档 × 2类顶面可用域。尺寸/几何来自固定真实或合成fixture并注明来源，不用目录标签代替测量。缺少某类输入则该类not_evaluated。每项同一硬件/profile、相同预算跑5次用于延迟，所有失败保留。每个arm/策略最多120请求；不是连续120次全成功门。

比较baseline、闭环分支消费、消费+经验缓存三种策略；冷/热缓存分别统计。先功能后性能；不在一轮冻结对比中修改artifact。单次请求为可靠性单位，报告成功率和区间；5次重复不能被当成120个不同场景的覆盖。

讨论目标：共同成功样本不少于20时P95首条完整认证轨迹时间较baseline下降至少20%；否则该延迟对比not_evaluated。另报全部请求含失败/超时的分布，防止只保留容易成功样本。任何基线已成功输入的新增失败单列阻止宣称全面改善。20%仅为待确认工程目标，不是已测收益；最终单请求成功率门需用户认可工作负载后冻结。

### 9.3 保留证据

T0记录revision/dirty状态、源和artifact哈希、环境/profile、场景和附件身份、工作负载和deadline。T1记录枚举/筛选/命中/尝试计数、stage timings、first-certified时间、超时、缺样与缓存cold/warm。

失败T2保存顶面与吸盘投影/完整包含掩码、候选ID和质量结果、目标箱心与TCP、实际附件、atlas节点/边/seed、FK/refine结果、碰撞对/位置、完整MoveIt场景与失败边/认证请求。仅局部疑难路径加T3。路径归档在docs/status/evidence/，不在角色笔记堆raw dump。

## 10. 分期实施建议与讨论项

1. 先确定完整包含门和抓取变换闭环；用当前atlas验证偏心目标算例、候选覆盖和真实消费，不急于全量重烤。
2. 增加结构化查询返回，使用现有paired transit及精确目标refine，修正锚点消费断层；确定性回放观察收益。
3. 扩展artifact为可重放分支图，定向补采portal、边界和实际请求薄弱区；独立记录采样成本。
4. 将目标引导的patch评估与抓取前联合选择接入真实typed入口，保持抓取后附件冻结；实现完整失败续取。
5. 接入成功经验的验证与连接路径，复用完整MoveIt认证，并限制缓存规模。

仍需讨论：指定箱心允许多大误差/区域；更深的目标与更短周期如何排序；默认分支数和候选预算；是否第一阶段只允许现有4个工具yaw（推荐保持）；最终性能门。当前用户已确定空箱与整个吸盘完整包含，不再将质量/力矩标定作为本轮前置条件。

## 11. 讨论补充：由放置目标反推pick工具朝向和完整关节分支

用户进一步指出pick的J6角度、搬运旋转与最终可达性耦合。本设计将联合决策变量显式扩展为：箱心/放置朝向、吸附点(u,v)、工具相对箱面的yaw phi、pick完整关节分支及winding、place分支、连接通道。不直接把末端yaw当成J6角度；需要完整IK、关节限位与全臂路径检查。

现有 `build_candidate_pick_segments` 的工具yaw由 `pick_tool_yaw(detection_yaw, yaw_valid, fallback_yaw)` 决定，不是目标驱动的自由搜索。后续接口需接收经完整footprint/质量评估的有向grasp候选，不能随意覆盖旧candidate朝向。非方形吸盘换角后包含域改变；当前方形接触轮廓也不意味着工具、相机和雷达π/2对称。

从最终箱体姿态T_WB_place和atlas建议的最终工具姿态T_WG_place反推T_BG=inverse(T_WB_place)*T_WG_place，再用当前实测箱体姿态计算T_WG_pick=T_WB_pick*T_BG。反推出的候选必须确实位于顶面、完整吸盘包含且可通过原质量门。求pick多个完整q解，再逐一连接吸附后实际lift状态到atlas入口/通道。终端姿态可解不能代替这条连接。

固定当前和最终箱体姿态时，改变抓取工具yaw不能消除箱体必须发生的净旋转：R_WG_place*inverse(R_WG_pick)=R_WB_place*inverse(R_WB_pick)。它改变的是夹具相对箱体方向、可用腕/肘分支、关节绕圈及旋转发生的可行路径。只有选择不同的合法最终箱体朝向（例如允许的几何对称目标）才可能减少必要净旋转；不得以预转J6声称取消固定箱体的90度转向。

规划阶段链：当前机器人→空载pregrasp→接近/吸附→携带lift→开口外转运或转向→portal→insert/descend→release→卸载retreat/return。优先在开口外允许区域完成必要转向，仍需验证全程箱体扫掠与全臂碰撞；不得把入箱末端目标的成功误报为完整链成功。

用目标端atlas多分支反向给出候选，用实际当前机器人状态向前验证pick/lift；在lift至portal的携带连接处汇合。采用共享预算的分层懒验证：完整吸盘包含→精确双端IK→节点载荷检查→atlas原语/连接→完整路径认证。首次失败可回溯到pick branch或phi，而不是只换place seed。规划诊断记录完整q、J6位置/离限位余量/实际连续行程、分支来源及失败阶段。J6只是排序/诊断之一；不能用wrap后的最短角差替代关节合法连续路径。

抓取前整链仍基于预测附件，标记provisional；抓取后实际附件确认重新验证，不能把事前证书用于不同附件。π和-π是同一工具旋转表示，但不同关节winding和到达路径仍需按实际限位处理。

增加确定性用例：同一吸附位置下phi-A可pick但无法携带连接，phi-B完整链通过；同一工具姿态两个pick分支中只有一个可接portal；J6接近限位导致退回另一分支；旋转后整个吸盘越界必须拒绝；固定箱体初末姿态下改变phi不得虚构减少箱体净旋转。全部记录实际IK/checker/edge输入，不能以源码文本断言代替行为。

## 12. 用户认可的实际链路：观测后整链规划，再执行抓取调整

用户认可“两端候选、中间连接验证”，并要求机械臂先在pick观测位测量箱体，与ULD内部OCC共同生成候选，完整合法轨迹形成后才调整J6并给出pick pose、place目标及trajectory。此处pick观测位与最终接触pick pose明确分开，避免为了测量就提前固定抓取角度/位置。

状态流程：OBSERVE→SNAPSHOT_READY→JOINT_PLAN→PLAN_READY→PREGRASP/ALIGN→APPROACH/ATTACH→ATTACHMENT_CHECK→CARRY/PLACE→RELEASE/RETREAT→OBSERVATION_COMMIT。J6调整属于从当前实测q到所选pregrasp的空载轨迹；不得在轨迹生成后单独写入J6或跳过全臂碰撞检查。需要调整其他关节时由完整IK和规划产生，不强制仅J6运动。

SNAPSHOT_READY需要箱体尺寸/有向姿态/顶面patch、当前关节状态、完整工具模型、ULD占用和静态场景。观测可来自不同时间，但各自采集时间和身份保持真实，检查新鲜度/一致性，不伪造同步时间。箱体/地图/模型变化使依赖它们的提案失效；单纯机械臂从观测位移开不应把世界系观测重解释到新的相机姿态。

JOINT_PLAN为同一方案逐阶段构造场景：空载pick接近，吸附后的预测附件载箱运动，释放后的新箱体世界障碍与卸载退出。只在整链有可执行提案时进入PLAN_READY；没有整链不先吸附尝试运气。抓取前完整路线检查基于预测附件，不能称为已确认实测附件的最终认证。

建议返回一个不可拆分的操作计划：plan_id、snapshot/model/observation身份、当前start_q、chosen suction candidate+有向工具角度、predicted T_BG、pick pregrasp/contact/lift poses及q分支、settled和release box targets、对应place TCP、按阶段分段的timed joint trajectories、attach/release事件屏障、各阶段认证及附件依赖。Box pose与TCP pose分别命名，不能用一个含糊place_pose字段代替。J6值可供诊断展示，不是独立的执行命令。

APPROACH/ATTACH后确认实际attachment与执行结束q，以其验证已计划的携带路线及衔接。条件一致且验证通过时直接复用，不默认重新做一次全局搜索。若不一致，进入有界重验证/局部连接/重规划；仅在成功后继续携带。抓取前计划无法消除这条物理事件屏障，也不能将预测变换覆盖实际测量。释放后按实际阶段确认移除附件、纳入箱体世界障碍，再执行退出。

新验收项：无完整提案时不发align/pick执行；J6调整以已检查全关节轨迹交付；接近时箱子在world、携带时为attachment、释放后为world且无错误双计/丢失；实测附件匹配时复用原carry段而不调用全局规划，不匹配时不得沿用旧认证；pick观测pose与contact pose不可混用；动作事件前后可按plan_id和阶段追溯。当前仍是设计讨论，不授权自动执行或实现。

## References

- `docs/agents/reviews/2026-11-11_0943_moveit-atlas-route-retrospective.md`
- `docs/architecture/frame_contracts.md`
- `docs/architecture/placement_prior.md`
- `src/luggage_description/config/suction_contact_model.yaml`
- `src/luggage_perception/luggage_perception/suction_patch_evaluator.py`
- `src/luggage_planning/luggage_planning/placement_prior_pick.py`
- `src/luggage_planning/luggage_planning/place_targets.py`
- `st09_contracts/attachment_geometry.py`
- `src/luggage_planning/luggage_planning/atlas_prior.py`
- `src/luggage_planning/luggage_planning/route_producer.py`
- `src/luggage_planning/luggage_planning/reachable_graph.py`
- `src/luggage_planning/luggage_planning/moveit_edge_adapter.py`
- https://moveit.picknik.ai/main/doc/concepts/motion_planning.html
- https://moveit.picknik.ai/main/api/html/classplanning__scene_1_1PlanningScene.html
