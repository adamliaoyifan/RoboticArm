# 结构候选优先选位与倒角硬门统一：G3 实施计划

- parent: PLACEMENT-GRID-LAYOUT-20261023
- generation: 3
- status: approved-design
- authorization: 用户同意结构候选优先、廉价排序、EMS仅离线，并要求落盘高效筛选与倒角修复计划；本轮仅文档，不授权运动或自动派发实现。
- base_revision: fd8f57a1a987ec0fcdae9c27a271306d7fda2636
- previous_plan: b976fe53:docs/plans/PLACEMENT_COST_PARALLEL_G2_20261023.md
- previous_implementation: 92455834（性能未通过；只作离线参考，不整包启用）
- owner: reviews/codex/gpt-5
- implementation_owner: eng/claude/glm-5.3
- integration_owner: eng/codex/gpt-6
- coordination_root: /home/adamliao/work/elfin_humble_ws/docs/agents
- dispatch_ready: no

## 1. 产品目标和不变项

目标是更快找到满足当前箱体几何与支撑条件、且可供后续轨迹认证的放置点。不是计算每个候选对下一箱的完整布局影响。

- 输入维持1cm OCC投影、同版本测量账本、权威倒角hull、完整冻结Attachment；OCC只有原store修改。
- 在线不使用EMS，不执行旧F/P未来窗口/碎片预测，不新增持久自由空间账本。EMS和旧F/P只允许独立离线评价，不成为请求依赖。
- 支撑比例、倾角、unknown规则、顶部净空、间距、包络、运动认证全部不放松。新策略间距显式0.02m；旧baseline显式0.05m，A/B隔离时必须统一成0.02m。
- 地图分辨率/包络不变；支持平放候选yaw0/90度，Attachment已有yaw不确定性已包含在包络内，不二次膨胀。邻箱测量yaw可非轴对齐。
- 前述整请求p95<=5s、max<=20s仍为性能门；后续实测未达不得在线启用。5s不是假定所有请求必达的已知事实。
- 不把稀疏候选无解、预算耗尽、未知支撑、路线不可达等价为BIN_FULL。

## 2. 新在线流程

```text
一次原子PlacementSceneSnapshot + Attachment + policy
 -> 请求内支撑/窗口索引（不构造全部候选dict）
 -> 结构候选生成、去重、廉价必要条件筛选
 -> 共用精确硬门
 -> 低层/atlas分组/邻接/深位排序 -> 一批候选 -> 现有路线搜索和认证
 -> 无硬合法候选或该批路线全部失败：同身份按需网格回退
 -> 地图变化：旧请求失效，不重基、不中途拼新图
```

结构优先是搜索顺序改变，不是永久删除其余网格位置。首次不执行旧solver全网格枚举后再挑结构点。全网格作为小样本正确性oracle和必要时惰性fallback保留。所有分支共享一次上游请求预算；续取批次不重新获得预算。

## 3. 支撑索引：一次计算，重复查询

新增纯模块 `placement_support_index.py`。输入只读snapshot/payload/硬门policy；输出请求内只读 `SupportIndex`，包含原点、floor基准、两个脚印格尺寸、窗口peak/known/support/tilt可查询统计和身份。不发布成新OCC，不以层量化篡改原始高程。

1. 高程按原solver的floor-relative语义处理；进入container几何时统一加实际floor_container_z，禁止两次加floor或当世界z。
2. 两个脚印方向的窗口max/min使用可验证的滑窗算法。known计数、指定高度带内计数可用积分图；阈值随peak变化的支撑/倾角不能直接声称一个积分图即可解决。允许候选局部精确查询或按重复高度带共享，必须与oracle相同。
3. 层标签仅用于发现平台结构/排序。支撑判门使用原始peak和原有容差，不能用量化层均值取代；不能要求整个脚印100%同高而误删原0.6比例可行窗口。
4. 对unknown只执行原floor_prior例外，绝不将未知箱间缝填平。从OCC中提取已知顶面平台边界，不能只依赖有ID的箱体账本。
5. 初始索引可扫描约29k高程格，禁止为每个候选重新建立整图统计。索引不得包含旧F/P预测未来箱循环。所有耗时单列。

## 4. 如何高效产生结构候选

新增 `placement_structural_candidates.py`，所有候选先以保守包络中心表达。输入支撑索引、账本、hull、payload、生成policy；输出 `CandidateProposal` 流及来源记录。

### 4.1 四类结构来源

| 来源 | 生成方法 | 防漏/安全边界 |
|---|---|---|
| 空容器/容器边界 | 在floor和已知支撑层，计算当前包络在整个z跨度内的可行中心域；取深/浅端、左右界角点与边中点 | 必须包含倒角高度相关边界，不用inner_size矩形角点冒充 |
| 已放箱边缘 | 按候选脚印与邻箱轮廓构造合法间距外偏移边界；取线段端点、投影、与hull中心域/其他邻箱偏移线交点 | 非轴对齐邻箱不能简单长宽换轴；交点仅是建议位置，精确间距门复核 |
| OCC已知支撑平台 | 按固定4邻接提取边界、转折点；取包络可覆盖平台的位置、中心及边界对齐点 | 平台标签不能代替支撑率门；跨箱联合平台不因ledger分块而拆掉 |
| 多结构交汇 | 只对包围范围相交的边段做投影/交点组合，按yaw/层分桶 | 不做全局所有X与所有Y的笛卡尔积；不承诺O(箱数)完整性 |

### 4.2 格点映射与去重

G3仍采用原solver脚印窗口原点格点，确保回退与离线全集可比。连续结构点映射到相邻floor/ceil窗口原点组合（最多4个），越界剔除；不只round一个点。旋转/偏心通过Attachment和窗口脚印语义处理，不移动测量箱心冒充保守箱心。

稳定ID采用map/payload身份绑定的窗口索引、footprint class、heading；同一点不同来源合并provenance，不重复算硬门。候选z由该窗口实际peak决定，不信任提议层；若实际z与提议层不同，以实际z重新做hull和所有硬门，不迭代“挪到合法”。

结构提案上限默认512个唯一格点候选/请求，批次默认64；来源固定轮转、层低优先、深位优先、stable ID兜底。上限属于新搜索policy而非容量证据，记录 generated/emitted/capped 与各来源计数；超过上限时剩余位置仍可被fallback覆盖，不能静默宣称穷尽。两参数启动只读并进入policy digest。

### 4.3 判门顺序

先用尺寸范围/窗口边界/顶部空间等便宜且可靠的必要条件拒绝，再查询unknown、支撑率和倾角，最后做完整hull/邻箱重叠/同层间距/既有aperture与corridor规则。允许调整计算顺序但失败记录保存全规范原因；与既有门优先级不同须有adapter恢复规范结果。预筛只能拒绝明确非法，不以近似证明合法。

所有有效结构候选和fallback候选共享 `PlacementHardGate.evaluate(...)`，从现有solver抽取无状态纯逻辑；旧solver委托同一内核。保留独立小型几何/支撑oracle，不能只让两条调用路径共享同一个错误再称等价。

## 5. 在线廉价排序和路线回退

新策略名 `structural_snug_depth_v1`，独立digest，不能沿用layout_cost旧名称。

- 结构与fallback每个批次内：原atlas band -> 支撑层升序 -> 同高邻接A降序 -> 深度降序 -> 原baseline score降序 -> stable ID。硬不可达直接排除，unknown不伪装可达。
- A沿用邻边投影并集、间距衰减与候选周长归一化；同顶比较候选**预测名义顶面**与邻箱测量顶面。名义位姿唯一由Attachment.box_pose_at_conservative转换。
- 不生成F/P，不模拟下一箱，不增加第二套可达性预测。本代是批次优先策略，不声称结果为所有未评估候选的全局cost最优；只有批次内prefix=排序前缀。
- fallback按层/深度/heading/索引稳定顺序分块枚举，跳过已经评估ID，复用SupportIndex和硬门。不能先遍历全图才能返回第一批。完整候选仍可在充足预算下逐批访问。
- 初批结构硬合法为空，ComputePlacement内部立即进入fallback；初批有位置但路线认证全失败，selector/orchestrator明确请求下一批，不能直接宣布箱满。
- 续取使用有界token，绑定snapshot、Attachment、policy、请求ID、cursor及已返回ID摘要。服务端同一session维护开始时间/截止时间，续取不能延长；只允许一条活动会话，完成/epoch变更/超时即释放。缓存丢失返回SESSION_EXPIRED，不重建成新预算。
- 每批响应含 stage、coverage_complete、generated/evaluated/rejected/returned、remaining_budget、termination_reason；超时返回SEARCH_BUDGET_EXHAUSTED。只有完整离散覆盖且容量硬门证据充分时才沿用原BIN_FULL条件；其他情况返回CANDIDATES_EXHAUSTED/OBSERVATION_REQUIRED等原规范码或新具名码。

## 6. 倒角修复详细步骤

### H1 权威描述与身份，禁止静默矩形退化

复用 `luggage_description.container_geometry.normalize_descriptor` / `descriptor_from_scene_config` 和 geometry_hash。DTO必须携带同snapshot绑定的descriptor，不从ledger或尺寸猜倒角。真实ULD配置声明倒角而descriptor缺失/哈希不符，返回HULL_DESCRIPTOR_MISSING/HULL_IDENTITY_MISMATCH；仅明确cuboid fixture可用无倒角合同。

已有 `_surface_hull` 对坏descriptor返回None；本代生产adapter必须先严格验证并拒绝，不能落入该静默兼容分支。不改容器视觉/物理资产，不重新生成A-prime。

### H2 高度相关可行中心域，复用现有几何核

本ULD为+Y底倒角、窄底宽顶。使用descriptor中的floor_y/wall_y/wall_z/floor_z，不硬编码历史数值。当前核已有 `y_bounds_for_z_interval`、`payload_center_y_interval`、`payload_footprint_half_extents`。

对平放包络尺寸(w,d,h)、yaw θ、底高z_b，令 r_y=|sinθ|w/2+|cosθ|d/2。中心上界为整个[z_b,z_b+h]区间内的最小 y_max(z,margin) 减 r_y；X及-Y按对应平面收缩。margin沿原法向距离定义，斜面不可简单在Y减同数值替代。向量化实现必须逐例与几何核及独立半空间oracle一致。

层索引只生成建议中心域；真实peak改变z_b后重新检查。不得只在箱中心高度取倒角宽度，亦不得只检查箱底中心或使用内腔AABB。

### H3 完整体积和开口语义

最终placement必须把完整保守包络放在指定支撑位置后检查全部8角点对闭合凸内腔半空间的包含、顶部及边界margin。平放假设不满足时具名拒绝，不用yaw-only公式强行处理roll/pitch。

现有solver `_hull_legal`调用 `contains_oriented_box_through_aperture`，允许从开口穿越；这是路线过境语义，不能单独证明最终箱体完整收纳。新增明确mode：最终placement要求完整containment（复用 `contains_oriented_box`），路线过开口仍使用through_aperture，不改路线允许过境语义。增加“箱体一部分留在开口外”负例，防止共享helper混用。

### H4 支撑与倒角是两道门

斜面是边界，不是默认水平支撑平台。floor_prior的可支撑范围必须落在实际地板内，不能把倒角底外区域补成平地。通过hull包含也不代表支撑率/倾角合格；由同一硬门分别判定。测试corner containment与floor-support clipping一致，未知顶面不变已知。

### H5 原几何oracle疑点单列修复

交叉顺序角点的SAT兼容性不能当真值。用独立OBB投影区间（两矩形各自轴，含间距语义）构造0/5/45/90度测试，与现有polygon方法对照。若发现实际不等价，保存最小反例，单独修共用几何内核/回归，不在新cost里复制已知错误。冻结参考结果前关闭此门；不得只改expected使兼容测试绿。

## 7. 接口、私有状态与修改范围

纯接口：

```python
SupportIndex.build(snapshot, payload, hard_gate_policy) -> SupportIndex
StructuralCandidateGenerator.generate(index, scene, payload, policy) -> ProposalBatch
PlacementHardGate.evaluate(index, scene, payload, proposals, policy) -> ValidityBatch
PlacementSnugRanking.rank(candidates, validity, reachability, policy) -> RankedBatch
GridFallback.next_batch(index, cursor, seen_ids, policy) -> ProposalBatch
```

输入/输出DTO深度不可变；不开放可写公有变量、可写数组别名或size/pose setter。算法无跨请求业务状态。`PlacementSearchSession`为唯一有状态协调class：`start()/next_batch()/close()`管理私有cursor/seen/deadline；只有这些方法能更新，snapshot及Attachment只读，不允许rebase/set_deadline。诊断副本经`diagnostics()`导出，不泄漏内部引用。

复用G2 DTO与邻接经过测试部分时逐文件选择，不默认合入旧F/P执行路径。新纯模块放luggage_packing；会话/消息适配位于planner节点外的独立模块。允许集成修改placement_solver、placement_planner_node、typed driver/selector批次调用及ComputePlacementV2相关合同；所有srv变化协调重建，不静默兼容旧binary。

hull核仍归luggage_description；不建第二套容器规范。同步 docs/architecture/placement.md、frame_contracts.md及对应规则。OCC-CROP、地图存储、感知、Attachment参数、物理模型和控制器均不在改动范围。

## 8. 实施分段

| 单元 | 负责人 | 前提 | 交付 |
|---|---|---|---|
| HULL-GATES | Claude | 精确base和本plan commit派发后 | descriptor严格准入、完整placement/过境区分、SAT独立对照、统一硬门及倒角fixtures |
| STRUCTURAL-CORE | Claude | HULL-GATES通过commit | 支撑索引、结构生成、廉价rank、惰性fallback、离线对照 |
| STRUCTURAL-INTEGRATION | Codex | 前两项通过commit | DTO/消息/session/selector接线、全请求性能、部署身份/rehearsal |
| PRODUCT-WITNESS | Codex | 集成通过且另行运动授权 | 冻结场景完整装箱见证及报告，不在本轮自动运行 |

每个子任务实际派发时填写通过依赖commit、具体worktree、generation和本计划提交hash。G2 COST-CORE性能失败状态不能当作通过依赖；本代改的是目标函数与搜索流程，不改写旧验收结果。

## 9. 离线验收矩阵和门槛

新增 `scripts/g3/placement_structural_eval.py` 与focused tests，所有oracle结果及失败输入持久化。

### 正确性门（全部确定性测试必须通过）

- 矩阵：resolution0.01/0.02/0.05；真实placed0/5/20（层数明确、assert actual==requested）；yaw候选0/90、邻箱0/5/45/90；grid相位0/0.25r/0.5r/0.75r；空地/等高并排/跨箱平台/支撑率门上下/unknown缝/偏心包络/窄通道。fixture实际几何必须可容纳预置箱，不静默截断。
- 倒角测试使用冻结部署descriptor：箱底位于floor、wall_z上下1mm、区间跨wall_z；到各平面有效margin的距离为-1mm/0/+1mm；nominal合法但包络越界、中心合法但角点越界、hull合法但支撑不足、开口外突出、错误descriptor/hash、非零world/container变换。EPS固定1e-9m只用于数值边界，不增通用碰撞容差。
- 所有返回候选硬门假合法=0；优化硬门相对独立oracle误差0（浮点量绝对误差<=1e-9；阈值两侧具名判定一致）。完整fallback与同grid全集oracle合法ID集合一致，且每ID最多评估一次；结构阶段不要求完备但必须报告漏掉的合法位置。
- 10次枚举顺序打乱，新策略结果一致；单批prefix/preferred一致；100次调用输入hash不变；同实例A/B/A与新实例A一致；每条响应绑定同身份。
- 强制“所有结构点不可达、存在一个非结构网格可达点”的受控route oracle，必须触发fallback并返回该点，不能报BIN_FULL；路线oracle为桩的结果不得声称真实MoveIt可达。
- stale snapshot/Attachment/policy、错误续取token、客户端重复next_batch、session过期均具名处理；同token重试返回同批，不重复推进，不重发预算；完整所有候选失败与时间截断分开记录。
- 批次COST与旧F/P可通过调用计数断言在线路径零调用；EMS模块不加载。GT/catalog不得进入在线候选生成。

### 性能门（先单次筛查，再重复验收）

本工作站CPU离线单进程，记录CPU/RAM/OS/Python/NumPy/依赖和线程；固定同输入对照旧全网格+廉价排序、新结构优先+廉价排序，统一间距/包络/硬门。旧F/P仅引用历史独立数据，不反复运行分钟级已知失败输入。

1. 每个9组负载先单次运行；任一请求>20s立即记录max失败，该组p95为not_evaluated，停止重复，不自动降负载。
2. 筛查通过后每组3预热+30测量；单组全链ComputePlacement（snapshot获取/解码、索引、提案、硬门、atlas、rank、序列化）p95<=5s、max<=20s、peak RSS<=2GiB。p95 nearest-rank=第29/30。未接真实服务的组件结果不得冒称全请求通过。
3. 首批与结构无解触发fallback分别测量；显式续取每批同门，同时报告整会话总时长/返回批数/累计预算，不能靠多批拆分隐藏总体开销。
4. 同输入旧/新对照记录结构命中率、首硬合法候选耗时、完整覆盖耗时、重复查询率、候选数量、各阶段时长及差额。无固定“亚秒/100倍”承诺。
5. startup heartbeat相邻接收max<1s；capture开关配对30次p95比<=1.10。分段互斥，差额<=max(100ms,总耗时5%)；other若为差额必须明示。

## 10. 执行入口与证据合同

以下为实现需交付的CLI合同，本轮不执行：

```bash
python3 -m pytest -q src/luggage_packing/test/test_placement_hard_gates.py src/luggage_packing/test/test_placement_support_index.py src/luggage_packing/test/test_placement_structural_candidates.py src/luggage_packing/test/test_placement_snug_ranking.py src/luggage_packing/test/test_placement_search_session.py
python3 -m pytest -q src/luggage_packing/test/test_placement_solver.py src/luggage_packing/test/test_placement_atlas_hook.py
python3 scripts/g3/placement_structural_eval.py fixtures --out "$EVIDENCE/fixtures"
python3 scripts/g3/placement_structural_eval.py compare --oracle exhaustive --out "$EVIDENCE/compare"
python3 scripts/g3/placement_structural_eval.py benchmark --resolutions 0.01,0.02,0.05 --boxes 0,5,20 --warmup 3 --repeat 30 --screen-first --out "$EVIDENCE/benchmark"
python3 scripts/g3/placement_structural_eval.py replay --bundle "$FAILED_CASE"
```

`EVIDENCE=docs/status/evidence/occ_owner_b1/placement_structural_g3_20261024`。pytest使用pipefail并tee完整日志；测试前记录git HEAD/dirty，产品源冻结提交后跑验收，报告单列随后证据commit。共用worktree/install不能混编；接口改动重建所有依赖并验证运行时文件hash。

每case保存snapshot/ledger/descriptor、Attachment完整变换/包络、policy与身份、候选来源/ID/索引、门结果、rank、枚举计数/覆盖状态、会话budget、分段计时、资源指标。失败输入必须可离线重放，旧失败不覆盖。记录容量与路线失败分类，不能把structure_empty变成箱满。报告单列cold样本、warm样本、not_evaluated和提前终止。

## 11. 后续产品见证准入

全请求离线门通过后，先非运动rehearsal验证消息/安装/地图身份/续取/捕获；运动另外授权。建议届时冻结A-prime、kinematic、seed0、inch20、prior关闭、原Attachment hash、原XY<=5mm/Z观察、180s总规划预算；不得自动增加预算。

正式运动计划另立准确命令、run额度和结束规则：最低本场前三箱完整release/verify/HOME/commit为AND3，仅证明这三箱序列，不推断人群可靠性；同时报告结构vsfallback来源、真实路线尝试数、首条认证路线时长和最终停止原因。是否继续容量测量由用户另定，本计划不默认max999。

本代完成必须有倒角正确性、全请求时延、fallback和身份闭环证据；结构候选减少本身不是容量提升证明。任何修改评分/候选上限/包络/间距需新policy身份与明确验收，不在运行中调参。
