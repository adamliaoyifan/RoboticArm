# 1 cm OCC/高程图与保守包络布局评分：实施计划 G1

- parent: PLACEMENT-GRID-LAYOUT-20261023
- generation: 1
- status: planned
- dispatch_ready: no
- owner: eng/codex/gpt-6（各实现单元及最终集成同一责任人）
- base_revision: accc0f403820f28d0ecaf3bb0dc4f750536dca74
- workspace: /tmp/elfin-master-pack-baseline-g3
- coordination_root: /home/adamliao/work/elfin_humble_ws/docs/agents
- authorization: 本轮仅交付完整计划；实施任务尚未派发。用户后续要求实施时按此计划的提交版本派发，无须重复确认已冻结的要求。

## 1. 本轮目标与语义冻结

1. Cargo OCC 的 X/Y/Z 体素与其派生高程图 XY 共用一个配置值，默认 **0.01 m**。本轮不引入多分辨率，不把5cm图插值当成1cm观测。
2. 参数可在启动配置/launch覆盖；地图创建后分辨率只读。改变分辨率须停止动作、结束当前地图实例，重启并重新建立地图；不支持在非空运行地图上热改或原地缩放。新实例使用新epoch，旧证书/缓存失效。
3. Attachment是保守包络唯一生产者。冻结当前yaw±5°、WDH增长10%、工具XY位置误差0、远离接触面位置误差5mm，配置sha256=3fa5139862ff950eb84ed5e139752e4d9776ded7d75007ba8ee9209cecae9956。保留偏心T_tool_box及已有转换；本轮不调裕量。
4. 实际选位继续按保守包络检查容纳、重叠、支撑与路线。此次不改为名义底面支撑，不放开unknown_above_floor硬门，不填补箱间空隙。因此评分改善不等于叠放一定恢复。
5. 增加同高贴边、未来支撑窗口、碎片空间cost；在硬门通过的候选中影响最终排序。预测地图仅为请求内评分数据，不写真实OCC/高程图。
6. 延续此前2cm起的箱间距意图：同层clearance默认从0.05改为0.02m，仍可启动配置。本代明确口径为**候选保守轮廓与已验证入图箱体轮廓间的规划间隙**；这是额外操作间距，不是新增尺寸膨胀，也不承诺实际箱体恰好相隔2cm。已入图箱体测量不确定性仍沿用现有地图合同，不新增隐式膨胀。所有gap评分采用同一轮廓/口径。
7. 20cm是当前箱体厚度量级；WDH按placement语义为footprint_x/footprint_y/vertical_extent，禁止当catalog长宽高重排。
8. 本轮1cm Z仍来自现有占用体素顶面，存在最多一层体素量级的量化；不宣称传感器达到1cm精度。

## 2. 已核实基线与限制

- 上一整轮 `47e8560c` 产品等价于ddc35211，5箱完整闭环，第6箱附着后重算无候选；无携带轨迹生成。证据：`docs/status/evidence/occ_owner_b1/pack_to_stop_20261023/REPORT.md`。
- 第6箱返回outside_hull=141、unknown_above_floor=59；完整后附着请求/逐候选未采到。禁止将旧5cm终态上采样作为1cm线上输入或声称精确重放历史失败。
- mapper已有`OccupancyMapStore.apply_update(MapUpdate)`、snapshot、epoch/revision/digest；扩展现有权威，不另建地图manager。
- `resolution`现有节点默认0.05；solver还有独立`floor_prior_resolution=0.05`，必须消除这处潜在分叉。
- 当前generate_candidates先按旧key裁剪top_n；solve、atlas后的候选列表另有排序。新cost必须在截断之前，且所有出口使用同一ranking policy。
- 当前ComputePlacementV2只有box_wdh，客户端把它替换成保守尺寸；planner没有完整Attachment身份。需显式传递几何，禁止从保守尺寸反推测量尺寸。

## 3. 单一所有者、统一写入接口与权限

| 状态 | 唯一所有者/入口 | 可读者与禁止事项 |
|---|---|---|
| 地图分辨率/布局参数 | 启动配置解析器生成冻结GridPolicy / LayoutPolicy | launch只转发显式覆盖；构造后只读。禁止回调直接改字段/多个模块分别设默认值 |
| OCC、箱体账本、pending、epoch/revision/digest | OccupancyMapStore.apply_update(MapUpdate)，其锁内调用mapper | node仅做ROS转换/权限检查；planner、cost、driver只能读snapshot；禁止外部`_mapper`写入 |
| 观测/提交/移除/重置授权 | 现有runtime admission + store事务校验 | 保留occupancy_commit仅mapper/driver范围及其他动作原有门；使用同一expected identity、operation_id、verify anchor，禁止引入绕过门的cost/regrid服务 |
| Attachment测量值、T_tool_box、保守包络 | MeasuredAttachmentProvider生成冻结AttachmentGeometry | planner只校验/读取；禁止在driver、solver、cost、碰撞检查器再次改尺寸 |
| 请求内候选 | 枚举器创建不可变CandidateGeometry；硬门创建CandidateValidity | 评分、atlas和排序分别返回附加记录；禁止共享dict被多个阶段原位修改score/size/pose |
| 布局评分 | PlacementLayoutCost.evaluate返回不可变CostBreakdown | ranker消费，diagnostics序列化；不写候选几何或地图 |
| 最终顺序 | PlacementRankingPolicy.rank唯一入口 | preferred、top_n、返回slot/candidates、driver消费顺序一致；运动可否执行仍由后续认证决定 |
| 预测高程图 | 请求内只读基线+局部预测patch | 禁止发布成真实surface或提交预测空隙/支撑 |

“统一权限”指已有进程内所有权、动作准入和事务CAS约束，不声称实现ROS网络调用者身份认证。本轮不新增SROS2/安全基础设施。读出的数组/列表不暴露可写内部引用；对外给只读视图或隔离副本。所有setter或可变别名检查采用行为测试。

## 4. 参数与配置接口

新建 `src/luggage_perception/config/cargo_map.yaml`，唯一默认：节点参数`resolution: 0.01`；新增纯模块`cargo_grid_policy.py`解析/校验为`GridPolicy(resolution_m, max_voxel_count, policy_digest)`。兼容既有ROS参数名resolution，禁止同时另设一个可独立覆盖的height_resolution。

- launch新增`cargo_map_resolution_m`，默认空表示用配置；显式值仅覆盖mapper的resolution及其同源floor-prior配置输入，启动时输出最终值、来源、GridPolicy digest。
- 允许有限正数且0.005<=resolution<=0.05m；部署验收覆盖0.01/0.02/0.05，0.005仅在资源预检通过时允许。max_voxel_count默认8,000,000，超过则分配前拒绝，不静默改粗；此容器0.005约34.75M会明确拒绝。
- surface携带实际resolution；planner收到地图时只使用该值。启动无地图的floor prior也从同一个解析结果获取值；旧floor_prior_resolution仅作迁移兼容校验，冲突明确拒绝，随后移除重复默认。
- 配置、命令行冲突优先级固定：显式launch覆盖 > config；不另加环境变量或driver硬编码地图分辨率。测试可构造GridPolicy，产品模块不能自选默认。
- 运行期`ros2 param set ... resolution`、布局权重/间距变化均返回只读错误，参数值与实际状态不变；通过修改启动参数重启生效。该边界在runbook中写明。

新建 `src/luggage_packing/config/placement_layout.yaml`：

```yaml
placement_planner:
  ros__parameters:
    same_layer_clearance_m: 0.02
    layout_cost_enabled: true
    layout_weight_adjacency: 0.40
    layout_weight_future_support: 0.40
    layout_weight_fragmentation: 0.20
    layout_neighbor_max_gap_m: 0.08
    layout_same_top_tolerance_m: 0.01
    layout_future_footprint_mode: current_conservative
    layout_rank_policy: layer_layout_depth_v1
```

这些权重是G1显式诊断起点，不是已测最佳值；冻结后不可在scored run中调节。未来参考脚印先只用当前Attachment保守WD及0/90°两个朝向，不用未知未来箱子的GT/catalog。所有长度单位m，score项归一化且记录原始值。`layout_cost_enabled=false`走有名的旧排序策略，用于离线A/B，不执行半套混合策略。

## 5. 新增/扩展的数据接口

### 5.1 地图快照

复用store的snapshot，新增只读 `PlacementSceneSnapshot`适配层，字段为：snapshot_identity(epoch,revision,digest)、scene_revision、geometry_hash、resolution、height/state/valid/confidence、已提交箱体几何及来源。适配器须在相同identity下读取surface和ledger，失配返回`PLACEMENT_SNAPSHOT_MISMATCH`，不得把新图与旧账本拼接。现有MapSnapshot已包含surface和ledger，但GetCargoVolumeV2只导出3D数组，没有原子surface+ledger组合。新增只读`/cargo_map/get_placement_scene_v2`服务，从store同一个MapSnapshot一次序列化两者；不让评分拉取全量3D图，不引入第二个写入口。

OCC导出、surface、commit ledger和缓存必须来自同一次store snapshot。resolution及GridPolicy digest进入地图content digest/缓存键；容器geometry_hash仍只描述容器，不混入分辨率。

新增 `luggage_msgs/srv/GetPlacementSceneV2.srv` 的明确字段：

```text
string request_id
string scene_revision
string geometry_hash
string expected_epoch
uint64 expected_map_revision
string expected_content_digest
---
bool success
string reason_code
luggage_msgs/SurfaceContractV2 surface
luggage_msgs/PoseContractV2[] placed
float64[] placed_wdh_flat
string[] placed_ids
```

响应所有数组来自同一snapshot；surface已有epoch/revision/digest作为唯一响应身份，不重复维护第二组。只读服务不调用apply_update、不确认pending，expected identity不匹配立即拒绝；新请求pending地图沿用原有确认门。新增`placement_scene_adapter.py`负责snapshot→message和message→纯DTO的结构转换，map node保持薄层。snapshot服务时限2s，计入placement总时限与性能门；不能在节点的同一互斥callback中同步等待自身服务，客户端使用已有异步请求结构。

### 5.2 ComputePlacementV2扩展（协调升级，全依赖重建）

保留box_wdh用于**测量/名义尺寸**，不得原位覆盖；追加：

```text
string geometry_stage              # nominal_preview | attached_conservative
bool has_attachment_geometry
luggage_msgs/AttachmentGeometryV2 attachment_geometry
string expected_epoch
string expected_content_digest
```

- nominal_preview：仅用于预抓取诊断，不产生可执行放置认证；无Attachment时不得偷偷生成中心假设。
- attached_conservative：必须有完整AttachmentGeometry，校验digest、测量绑定、场景/附着身份；缺失或不一致具名拒绝。另建不可变`PlacementPayload`，包含measured_geometry、conservative_geometry、名义/保守位姿变换。
- solver的placement_dimensions只引用conservative.dimensions_wdh；名义尺寸仅用于解释、预测物理箱顶与之后的测量提交。对真实occupancy提交仍是verify锚定测量几何，不能用包络填充虚假实体。
- response增加`candidate_ids[]`、`layout_policy_digest`、`candidate_layout_costs[]`、`slot_geometry_semantics`和`placement_geometry_digest`。candidate_scores统一为`-layout_cost`（越大越好）；旧baseline score进诊断记录，避免正负混乱。
- response的附着后候选为conservative_center语义；`attachment_placement_adapter.nominal_placement_response`是唯一转换入口，返回新的nominal_center结果，原response不变。driver消费名义目标，但轨迹构建/认证继续绑定原Attachment digest与保守包络。
- AttachmentGeometryV2消息及纯st09_contracts已存在，复用其字段/合同。packing纯算法不能import planning形成循环；消息adapter可放description的ROS bindings，或packing节点层薄adapter，纯类依赖st09_contracts。
- 此次扩展V2是同工作区协调部署，不承诺旧二进制wire兼容。更新接口身份清单/类型指纹并重建所有客户端、服务端；旧overlay必须被启动身份检查拒绝。

### 5.3 纯算法接口

```python
PlacementLayoutCost.evaluate(
    snapshot: PlacementSceneSnapshot,
    payload: PlacementPayload,
    candidates: tuple[PlacementCandidate, ...],
    policy: PlacementLayoutPolicy,
) -> tuple[PlacementCostBreakdown, ...]

PlacementRankingPolicy.rank(
    candidates, validity, reachability, layout_costs, policy
) -> tuple[RankedPlacementCandidate, ...]
```

CostBreakdown字段：candidate_id、map_identity、geometry_digest、policy_digest、adjacency_raw/normalized、future_support_before/after/delta、fragmentation_before/after/delta、total_cost、validity/status、elapsed_ms。不允许NaN或用0冒充未评估；cost缺少必需输入则整次请求具名失败，不默默回退。

## 6. 评分与排序的具体行为

1. 同高邻接A：只计算平行边投影重叠长度；顶面差<=1cm，轮廓距离g在[g_min,0.08]内。单边收益为重叠长度×clip((0.08-g)/(0.08-g_min),0,1)；多邻居重叠边段先求并集避免重复奖励，除以候选周长，得到A∈[0,1]。g_min为同一个same_layer_clearance_m。
2. 未来窗口F：在预测**测量箱顶**与现有已验证顶面组成的临时图上，按当前保守WD枚举未来中心。只有完整窗口满足现有支撑已知性、support_ratio、顶面一致性及容器顶部/几何硬条件才算可用；不把未知格改成free或support。0/90°可用中心区域取并集，面积按resolution²计算；F=(after-before)/hull_floor_area，可为负。此项只声明支撑潜力，不声明未来机械臂可达。
3. 碎片项P：在该候选所影响的同高层，统计已知可用区域中无法容纳参考脚印的孤立窄区域面积；保持未知区域独立、不给推定free。P=(after-before)/hull_floor_area，减少碎片为负。不用全图高度TV，避免平滑噪声/填缝制造假平台。
4. 总cost `C=-0.40*A-0.40*F+0.20*P`。输出各项原始m/m²及归一化值，便于追溯。
5. 所有硬门（包含当前保守支撑门）先检查；cost不改变feasible。每个请求共享只读地图基线和预计算窗口统计，候选仅更新受影响patch。缓存键含map、payload、policy全部digest。
6. `layer_layout_depth_v1`顺序：合法性 → 原atlas可达性分组 → 支撑层低到高 → 总cost低到高 → 深度深到浅 → 旧score高到低 → stable_id。硬不可达仍拒绝；实际建路/认证另行进行，不能将尚未建路的route_clearance=0当已知空间证明。
7. 评分和上述排序必须发生在top_n截断前；atlas标注若会影响合法性/分组也必须在截断前。generate、solve、preferred、ranked_feasible_candidates全走一个rank函数。后续路线selector若因可执行性/既有路线代价改选，必须记录placement rank、route rank和改选原因，不能归因于本cost失效而不取证。
8. 本代不使用粗网格预筛导致合法候选丢失；向量化窗口统计/积分图/局部patch只优化计算，不改变枚举结果或偷偷减少top_n。

## 7. 明确修改文件

| 文件（src/相对路径，标注新增） | 修改职责 |
|---|---|
| luggage_perception/config/cargo_map.yaml（新增）、luggage_perception/luggage_perception/cargo_grid_policy.py（新增） | 唯一1cm默认、验证、只读策略及digest |
| luggage_perception/scripts/cargo_volume_mapper_node.py | 参数适配/只读拒绝；所有写均进入store；同源floor prior配置输出 |
| luggage_perception/luggage_perception/occupancy_map_store.py | 复用apply_update、snapshot原子性；分辨率身份、只读快照访问，禁止新平行ledger |
| luggage_perception/luggage_perception/cargo_volume_mapper.py、voxel_log_odds.py、cargo_volume_snapshot.py | 1cm容量预检；若基准超限则紧凑数组/向量化/缓存，保持geometry lock、source和sensor shadow恢复语义；不改阈值 |
| luggage_description/luggage_description/ros_frame_bindings/surfaces.py | resolution/shape/identity严格校验；禁止改写来凑齐输入 |
| luggage_gazebo/launch/sim_world.launch.py、luggage_perception相关launch、luggage_planning/luggage_planning/hardware_placement_launch.py | 两类部署路径转发同源配置、覆盖日志；消除旁路5cm默认 |
| luggage_msgs/srv/ComputePlacementV2.srv、GetPlacementSceneV2.srv（新增）、luggage_msgs/CMakeLists.txt | 显式几何/cost与原子只读快照接口；复用AttachmentGeometryV2 |
| luggage_perception/luggage_perception/placement_scene_adapter.py（新增）、packing节点层对应decoder | store snapshot完整序列化；纯DTO解码/身份校验，禁止新增状态所有者 |
| luggage_planning/luggage_planning/typed_driver_adapter.py、attachment_geometry_transport.py、attachment_placement_adapter.py | 名义与保守尺寸分开字段；只传输、校验、单次转换，禁止重膨胀 |
| luggage_packing/luggage_packing/placement_layout_contracts.py（新增） | 冻结Snapshot/Payload/Candidate/Policy/Cost DTO与校验；纯Python无ROS |
| luggage_packing/luggage_packing/placement_layout_cost.py（新增） | 预测patch、三项cost、只读计算 |
| luggage_packing/luggage_packing/placement_ranking.py（新增） | 唯一新/旧ranking策略与deterministic tie break |
| luggage_packing/config/placement_layout.yaml（新增）、scripts/placement_planner_node.py | 配置解析、快照捕获、服务适配和诊断薄层；不堆评分算法 |
| luggage_packing/luggage_packing/placement_solver.py、placement_reachability.py | 不可变阶段结果、cost-before-truncation、atlas顺序统一；保守硬门不放松 |
| luggage_gazebo/scripts/place_smoke_driver.py、pack_eval_driver.py | 完整前/后附着请求关联；消费具名几何语义；独立记录attempt_started以修正终止箱少计，禁止改变stop-on-first-failure |
| scripts/g3/n3_functional_run.sh | 参数转发、冻结配置/安装身份、capture-health gate；保留单栈与owned teardown |
| scripts/g3/placement_layout_eval.py（新增） | fixtures/replay/benchmark/compare/audit CLI；不上线发布地图或使用GT选点 |
| docs/architecture/placement.md、frame_contracts.md、size_semantics.md及对应.cursor规则 | 统一几何/排名/默认间距/消息合同和单写权限；实现同commit更新 |

attachment_geometry.yaml及Attachment产生算法保持冻结；这里只补接口可追溯性。涉及现有巨型node的新增逻辑必须下沉到上述专职模块。

## 8. 分阶段交付（全部owner=eng/codex/gpt-6）

- S1 GRID：依赖无；从base开始，实现统一参数、1cm存储/导出/配置身份与权限行为测试；离线mapper基准达标。
- S2 CONTRACT：依赖S1；ComputePlacement几何与snapshot合同、overlay协调升级、完整失败边界采集、计数修复；契约往返/偏心转换/拒绝测试通过。
- S3 COST：依赖S2；纯cost/ranking及top_n前集成，2cm操作间距配置、数据不可变性、确定性fixture与性能通过。禁止单独扩展planning膨胀。
- S4 INTEGRATION：依赖S3；构建/身份哈希、只读启动参数检查、无动作rehearsal，冻结产品后一次持续装箱到终止，审计/证据/清理/报告。

每阶段以自己的实际实现commit和测试证据关闭；下一阶段base取其通过commit，派发时填写实际hash，不预填虚构hash。本计划提交的hash作为每个派发的exact plan_revision；目前dispatch_ready=no，不创建可运行mailbox项。

## 9. 验收：输入矩阵、门槛与重复规则

### A. 行为正确性（全部确定性用例必须通过）

新增测试 `test_cargo_grid_policy.py`、`test_placement_layout_cost.py`、`test_placement_layout_contracts.py`、`test_placement_ranking.py`；维护现有mapper/store/attachment/placement测试。

- 分辨率0.01/0.02/0.05；拒绝NaN/Inf/<=0/范围外/超过voxel cap；无显式覆盖时两图均0.01；运行set拒绝并保持原值/digest；两图/ledger版本一致。
- 同样小网格旧实现与存储优化逐cell states/logodds/source/labels一致；add/remove/reset/sensor shadow恢复、重复operation幂等、stale epoch/revision/digest拒绝。拒绝操作地图及账本全量hash不变。
- 同一Attachment encode/decode digest相同；偏心位姿往返位置误差<=1e-9m、旋转矩阵误差<=1e-9；box_wdh保持名义值，planned尺寸只等于Attachment保守值；无二次增长。已完成接口转换不得再转换。
- 手工fixture：空地、同高两箱、异高两箱（顶面差0/0.01/0.02m）、已有5箱重建场景、unknown边缘、狭缝和边界相位。间隙0.019/0.020/0.030/0.050/0.080m；yaw0/90及5°测量轮廓；平移相位0/0.25r/0.5r/0.75r；resolution三值。0.019拒绝，0.020允许等于阈值；评分不使任何硬拒绝变合法。
- 在无其他门差异的同层fixture，g=0.020比0.050的A严格更高；平行边对齐比角贴近更高；顶面差0.02无同高奖励；宽平台比相同面积细长平台的可用完整窗口面积大；未知缝永不新增支撑。
- cost-disabled与旧策略候选ID序列一致；cost-enabled夹具中改变depth但使cost明显优时选新cost优者；改变输入枚举顺序10次，输出ID顺序完全相同。top_n=1/5/2000与全候选排序前缀一致；preferred=returned[0]。
- rank/atlas/cost调用前后输入snapshot、geometry、candidate字节hash完全一致；重复100次不得累计膨胀或分数漂移。
- 第6箱场景用**已验证测量ledger**重新栅格化为各分辨率，并明确标为derived fixture；旧5cm unknown不能变成1cm已观测free。不得声称精确在线失败重放。新的在线capture必须能够离线重放新的附着后实际请求。

### B. 性能门（部署同一工作站，ROS Humble，记录CPU/RAM/GPU/线程/依赖）

当前容器包围尺寸1.49×1.97×1.48m，1cm为4,344,244体素、29,353高程格；使用0/5/20箱已验证测量几何夹具，每组合3次预热+30次测量，不能合并掩盖单组失败。

- mapper构建/首次完整snapshot<=30s；热态add+surface+digest p95<=1s、max<=2s；mapper进程peak RSS<=2GiB。
- 单次ComputePlacement（全候选、全部hard gates/cost/atlas和序列化）p95<=5s、max<=20s；planner peak RSS<=2GiB；维持现有30s服务deadline。
- required peer heartbeat相邻接收间隔max<1s（现有约2s租约不加长）；启动与计算期间都捕获，重计算不得占住心跳executor。
- capture开启相对关闭的30次配对离线benchmark p95耗时比<=1.10；超过则先优化采集，不能以降低capture完整性通过。
- 样本不足30/组 => not_evaluated；性能不达标则留在离线优化，禁止带着已知阻塞进入整场仿真。以上是拟定工程门槛，不是已达到的测量结论。

### C. 离线A/B与最终实时验证

离线相同原始/测量输入比较：A=5cm/旧cost/5cm间距；B=1cm/旧cost/2cm间距；C=1cm/新cost/2cm间距。B/C隔离cost影响；A/C存在网格与间距两项混杂，只描述总体差异。另固定间距2cm比较5cm/1cm，以隔离网格效应。输出每例分项、ranking key、选中ID、拒绝原因和耗时。

实时：本场景A-prime、seed0、inch20、prior关闭、预算180s、包络hash冻结、XY<=5mm/Z观察、直接portal优先。先无动作rehearsal通过gain/URDF/唯一clock/camera/地图/捕获健康；启动失败零case不计装箱样本，最多1800s墙钟内重启且逐次保留证据。启动后冻结版本运行C **一场** max999、driver43200s，到首个终止事件，禁止跳箱/人工放置/调整包络/失败后盲重跑。

- 功能回归门为同一场最初3箱全部release/verify/HOME/commit通过（AND3，要求本场3/3，无人口成功率推断）；不足3箱则功能门失败或启动not_evaluated，不能当通过。
- 容量观测不规定必须>5：报告真实成功数、首个失败阶段、各层箱数/间隙/顶面窗口及cost。仅BIN_FULL且证据支持时讨论此输入条件容量；candidate exhausted/预算/infra不能称满。
- 结果必须将已物理提交但HOME失败、完整闭环、尝试开始计数分别列明，suite和ledger逐条一致。每个成功箱释放后验证通过、地图与回程冻结一致。
- 一场只证明该序列行为，不统计可靠性提升；布局fixture改进与实时箱数分别报告。

## 10. 精确执行入口（以下新增脚本/子命令为实施交付合同，当前尚不存在）

从实施worktree运行，先source /opt/ros/humble/setup.bash和主workspace install，再source当前worktree install；测试不得用src路径冒充安装身份。新增文件安装规则与包依赖一起更新。

```bash
colcon build --packages-select luggage_msgs luggage_description luggage_perception luggage_packing luggage_planning luggage_gazebo
python3 -m pytest -q src/luggage_perception/test/test_cargo_grid_policy.py src/luggage_perception/test/test_voxel_log_odds.py src/luggage_perception/test/test_cargo_volume_snapshot.py src/luggage_perception/test/test_cargo_map_commit_contract.py src/luggage_perception/test/test_occupancy_map_store.py src/luggage_perception/test/test_occupancy_map_store_anchor.py src/luggage_perception/test/test_occupancy_map_store_confirmation.py src/luggage_perception/test/test_occupancy_map_store_projection.py
python3 -m pytest -q src/luggage_packing/test/test_placement_layout_contracts.py src/luggage_packing/test/test_placement_layout_cost.py src/luggage_packing/test/test_placement_ranking.py src/luggage_packing/test/test_placement_solver.py src/luggage_packing/test/test_placement_atlas_hook.py
python3 -m pytest -q src/luggage_planning/test/test_attachment_geometry_transport.py
python3 -m pytest -q src/luggage_gazebo/test/test_release_pose_stream.py src/luggage_gazebo/test/test_pack_commit_source.py src/luggage_gazebo/test/test_commit_to_cargo_map.py
python3 scripts/g3/placement_layout_eval.py fixtures --resolutions 0.05,0.02,0.01 --out "$EVIDENCE/fixtures"
python3 scripts/g3/placement_layout_eval.py benchmark --warmup 3 --repeat 30 --out "$EVIDENCE/benchmark"
python3 scripts/g3/placement_layout_eval.py compare --matrix grid_layout_g1 --out "$EVIDENCE/compare"
PHASE2_STARTUP_ONLY=1 PHASE2_RETURN_REHEARSAL=1 PHASE2_RUN_OUT="$EVIDENCE/rehearsal" bash scripts/g3/n3_functional_run.sh
PHASE2_MAX_BOXES=999 PHASE2_DRIVER_TIMEOUT_S=43200 PHASE2_RUN_OUT="$EVIDENCE/live_01" bash scripts/g3/n3_functional_run.sh
python3 scripts/g3/placement_layout_eval.py audit --run "$EVIDENCE/live_01"
python3 scripts/g3/placement_layout_eval.py replay --bundle "$EVIDENCE/live_01/failed_placement"
```

`EVIDENCE`统一指向实施worktree的`docs/status/evidence/occ_owner_b1/grid_layout_cost_g1_20261023/`；shell变量仅作路径。replay仅在有失败bundle时运行，否则以每箱已捕获请求离线抽查；所有新增CLI必须对上述门槛自动返回成功/失败/not_evaluated，不依赖阅读源码字符串判定。

## 11. 证据矩阵与资源预算

| 边界 | T1始终记录 | T2触发与payload | 窗口/格式 | 上限/复核 |
|---|---|---|---|---|
| config/startup | revision、dirty、安装hash、实际参数及来源、grid/policy/envelope digest、环境和耗时 | 启动失败时URDF/参数/clock/camera/readiness | 启动至终止；JSON/text | 128MiB；preflight健康检查 |
| perception→Attachment | 请求/箱ID、测量WDH、T_tool_box、附着generation/digest | 失败箱检测及原始对应测量帧、完整AttachmentGeometry | 失败前10s后5s；CDR/NPZ/JSON，GT独立目录 | 512MiB/箱；几何往返和来源审计 |
| map→placement | 原子snapshot身份、resolution、shape、ledger与state统计 | 请求开始即保存完整snapshot（或指向已durable内容寻址文件），失败时保留 | exact一次快照；压缩二进制数组+JSON索引，禁止每候选拷贝全3D图 | 1GiB/版本；hash/shape/epoch验证 |
| ComputePlacement | 完整原始request/response，所有候选ID/pose/WDH/reason/rank/cost、截断前计数 | 任一失败冻结实际后附着请求、snapshot、策略及全部硬门/评分记录 | 本次请求起止；CDR+压缩列式记录 | 512MiB/请求；离线replay ID/结果一致 |
| route→execution→release | 几何digest、map identity、轨迹/证书、状态边界、verify/commit结果 | 失败轨迹和现场、joint/controller、release原始流、heartbeat环 | 心跳60s，关节失败前10s后5s，release90s/8MiB | 1GiB/箱；独立时序关联 |
| suite/stop | attempted-start ledger、完整闭环/物理提交、终止原因、阶段计时 | 首个失败及最终布局、teardown | 全场轻量JSONL，失败manifest | 整场16GiB，保留总32GiB |

捕获门必须在动作前验证写盘/空间/解码/请求关联，不能只验证目录存在。复用内容寻址snapshot，压缩完整候选记录；不因512MiB限制丢候选而仍称完整。超预算在下一次动作前受控停止、记resource_censored并保存证据。失败manifest明确case/stage、原始输入、missing、capture_complete/replay_possible；缺失先补采集再进行下一次该门运行。

失败必要窗口和索引持久化后执行owned teardown；正常driver退出立即清理。domain7/headless/独占Gazebo；检查residual bridge及恰好一个clock。原始产物复制主workspace并逐文件sha256核对，提交报告/manifest/有界关键JSON，完整二进制本地保留。不得覆盖旧失败。

## 12. 完成条件与后续决策

实施完成必须同时有：统一参数/写权限行为通过、接口/几何往返通过、离线排名/不可变性/性能通过、安装身份与rehearsal通过、冻结整场观测报告及完整终止证据。实时功能门失败应明确失败，不得把“已经跑完”当产品通过。

首轮结果专门回答：1cm能否改善支撑边界表达；新cost是否改变选点并形成更好的平台；实际停止原因是否仍为保守包络支撑/容纳；资源代价是否满足门槛。看完这些证据再另开一代讨论包络大小/接触支撑语义，本轮不自动调小包络以追求箱数。
