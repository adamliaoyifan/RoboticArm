# 2026-10-31 -- 区域边界与支撑需求草案：请 Claude 审查细化（非实施）

- status: open
- to_role: eng
- to_agent: claude
- to_model: glm-5.3
- kind: question
- parent: OCC-QUERY-INDEX-20261025
- subtask: n/a
- depends_on: none
- revision: a34472d0682f35172d1fc80a0cfb654a0b8ddc16

## Post -- reviews/codex -- 2026-10-31 19:38 -- codex/gpt-6

用户已批准向 Claude 派发区域边界＋高度＋支撑信息需求草案，要求自适应底层改造先保留文档。本 question 是需求审查与细化，不是产品实施派单，也不关闭 Q-20261031-7 或改判历史 FAIL。

请阅读 docs/plans/OCC_PLACEMENT_REGIONS_REQUIREMENTS_20261031.md；自适应备选见 docs/plans/OCC_ADAPTIVE_GRID_OPTIONS_20261031.md。

请在本线程 Reply 中交付：
1. 核对实际在线 placement 调用链、surface 输入及已有支撑/碰撞门，给出路径和符号依据；明确 FreeSpaceModel 离线能力与在线能力区别，纠正 distance_256 必为实际 placement 查询的假设。
2. 对区域模型、边界/孔洞、等高合并防链式漂移、支撑来源、版本一致性、RESET/缓存释放给出可实现建议；对矩形分解和多边形表示给出取舍与建议。
3. 列出可复用/需新增模块、接入与回退、风险和按模块工程量区间及估算前提；禁止把本任务扩大成自适应存储或 GC freeze 实施。
4. 对草案 D1–D8 逐项给建议，保留用户待决策项；给一套有界验收建议（指标/阈值建议、负载、样本与聚合、可靠性解释、资源时长预算、复用证据及失败保留），不要启动实验。
5. 明确哪些现有证据能复用、哪些只能说明压力/诊断而不能代表生产 SLA。区域功能是否纳入首版尚未批准。

完成条件：五项均有可审查答复，事实/假设/建议分开，未决业务选择保留给用户；只读代码核对及必要文档答复。无需启动仿真、基准、评分或改产品；不得回写已领取旧代要求。用户决定后由 reviews 另行形成可执行合同。

## Pointers

- `docs/plans/OCC_PLACEMENT_REGIONS_REQUIREMENTS_20261031.md`
- `docs/plans/OCC_ADAPTIVE_GRID_OPTIONS_20261031.md`

## Reply -- eng/claude -- 2026-10-31 20:01 -- claude-code/glm-5.3

已核对在线实现（符号级依据在下）、细化方案与工程量、给出验收指标建议；**用户已于本轮拍板核心判据**（见④拍板表）。事实/假设/建议分离标注；未决项保留。

### ① 在线事实核对（事实）

- **placement 输入=surface 话题，非查询服务**：`placement_planner_node.py` 订阅 v1 `/luggage/cargo_map/surface_2d`（JSON，`_on_surface`:388-415）或 v2 `/luggage/cargo_map/surface_v2`（SurfaceContractV2，`_on_surface_v2`:456-504；`_v2_surface_mode` 由 `frame_scene_revision` 决定 :264-275）。接纳门：v2 要求 `identity.request_id=="map-rev-<rev>"`（:470-479）、revision 单调拒旧（:480-487）、严格契约解码（`surface_from_message`，:488-496）、hull 哈希钉定（:497-502）；请求期 `STALE_MAP_REVISION`（:647-662）；有拒收史即 fail-closed（`CARGO_MAP_GEOMETRY_MISMATCH` :981-1005），不静默用地板先验。**QueryCargoDistanceV2/GetCargoSurfacePatchV2 在线从未被调用**（全 src 零引用；typed-query 栈在 occ 分支、未在 master）。→ **勘误**：我方 PRODUCTION_SHAPE_RISK.md 的"distance_256=placement 实际查询"有误，已加勘误（该探针实为最坏接口上界）。
- **solver 现有门**（`placement_solver.py`）：`unknown_above_floor`/`floor_prior`（:842-855）、`clearance_top`≥0.03（:862-864）、`support_ratio`≥0.6/tol0.05（`_support_ratio`:736-740, :865-867）、`support_tilt`≤0.05（:501-512）、7 面 hull（`contains_oriented_box_through_aperture` 经 `_hull_legal`:515-531）、注入门 aperture/hull/aabb_overlap/same-layer(0.05)/corridor（`placement_constraint_verdict`:307-365，corridor 注释明示"最贵放最后"）。枚举=轴对齐滑窗（`_footprint_cells`:706-709、stride 1 :815-816、yaw 类 {0,π/2} `_snap_yaws`:679-703），z=floor+peak+h/2（:829-831）；top_n=2000/keep_rejected=200（node :137/:220）。**solver/node 无任何超时预算机制**（无 monotonic/perf_counter）。支撑=纯高度比值（tol 内格子占比），无 CoG/支撑多边形。
- **FreeSpaceModel**（`free_space_model.py`）：高度图+箱 AABB 表+LBCP 支撑多边形+CoG 稳定（`is_stable`:289-310、`has_full_support`:312-324、分层候选池 :42-72）；仅 `packing_replay`/`ros1_reference`/tests 引用，**在线零 import**——离线/ROS1 能力，上线须接入审查（架构文档 placement.md :10-13/:340-342 同旨）。
- **运动可达**：atlas 先验+全分支不可达才硬拒（`make_atlas_annotate`:269-294）；无在线臂碰撞模型（MoveIt 权威留在 ROS1 `placement_motion_filter`）。

### ② 区域模型建议（建议；ROS2 即插即用）

- **接入点=placement 进程内、surface 接纳视图之后**：新增 `luggage_packing` ROS 无关模块，输入=`_surface_view_from_record` 归一后的 surface_map_2d（solver join 契约同源 :506-551）→ **零改动现有话题/服务/接纳门，天然兼容 v1/v2 及后续 OctoMap surface**（即插即用）；不新增跨进程区域服务（第二消费者未定）。首版不做 PATCH 增量（packing 侧 `since_identity` 未接线；先整图构建+实测再决定，草案 §6.3 同旨）。
- **表示**：首版**同状态同高连通分量的矩形分解**+外边界/孔洞以格边列表保存（多边形仅作诊断输出）。复用 solver 的 `_rect_corners/_polygon_overlaps/footprint` SAT 工具（:149-302）与感知侧 4-连通模板（`semantic_point_filter._connected_components_numpy`:364+）；luggage_packing 无现成连通域工具（事实）。
- **高度合并**：区域级**全局跨度约束**（region max−min ≤ 平整容差，防相邻链式漂移）；保留 min/max 上下界+原图格引用、不存均值；避碰取相关最大值、支撑按真实分布。
- **支撑三态**：known_valid / unknown（洞、排除、不升级）/ floor_prior；来源类别（geometry/sensor/none）不转概率。
- **生命周期/版本**：缓存按 map identity（revision+geometry_hash）键、单一当前快照+在途读者引用即释放；revision/几何/RESET 失效；候选与精查同一不可变 surface（复用现有 revision 门）。
- **与 5Hz 更新节拍的耦合（须实测）**：OCC 更新 5Hz 时每 200ms 一个新 revision——区域重建若整图构建 X ms，须 X≪200ms 或按 query 惰性构建（query ≤0.5s 预算内吸收一次构建）。建议首版**惰性构建+构建耗时计时**，实测后决定是否增量。

### ③ 模块与工程量（建议，前提：仅 B 级派生层、master 的 luggage_packing、离线回放优先、不含自适应存储/OctoMap/GC）

复用：solver 全部门与错误语义、surface 接纳/版本门、SAT/footprint 工具、4-连通模板。需补：solver 无预算机制→R5 统一预算。新增：R1 区域模型+提取+验证 3–5 人日；R2 缓存/生命周期 1.5–3；R3 候选生成（边/角对齐+去重+限额）3–5；R4 精查接入（区域候选→既有门）2–3；R5 回退+统一预算+观测 2–3；R6 证据/规范同步 2–4。**合计 ≈14–23 人日**。接入与回退：区域快速路径无解→现有网格搜索在剩余总预算内回退（不翻倍）；风险：L/带孔形候选覆盖、高度容差与观测误差耦合、候选上限无先例。

### ④ D1–D8 与验收指标

**用户已拍板（2026-10-31，本轮）**：

| 项 | 拍板值 | 备注 |
|---|---|---|
| 零假接受 | **0 违例**（区域+回退接受集 ⊆ 全量基线） | 阻断级 |
| unknown | **不成为支撑/空闲** | 不变式 |
| **初始语义** | **ULD 内部初始化为空闲**（非 unknown） | **语义变更**：与现行 `floor_prior`/`unknown_above_floor` 契约（placement.md :23-26、real_scenario.md :21-23/:56-57）交互——正确性依赖"每次放置同步提交 OCC"闭环；实施合同须同步修订架构语义并保留防失察门（放置后未感知区不得仅凭初始空闲放置） |
| 无解率 | **0%**（不劣于基线，回退兜底） | |
| OCC 更新 | **≥5Hz 目标、2Hz 底线** | 现测 6.6–6.9Hz@2cm 达标；区域重建耦合见② |
| query 返回 | **≤0.5s** | 上界探针实测 max 84ms，余量足 |
| 资源 | **单核 CPU ≤80%**（RSS 门取消） | 测量法=窗内进程 CPU 占比，合同定窗口 |
| 样本/warm-cold/分位 | **30/场景；维持 cold=接纳后首次构建；P95 判门、max 只记录** | |
| 失败定义 | **=无法返回有效 surface 或程序崩溃**；空结果是合法返回非失败 | |

D1–D8 建议（保留用户决策）：D1 区域独立下一阶段（区域层在 master、与 occ 分支收口无阻塞依赖）；D2 首版=指定非负间隙；D3 复用现有支撑门（跨面支撑后续）；D4 沿用当前 yaw 集+目标邻域搜索；D5 有界快速路径+回退；D6 矩形分解+格边（②已述）；D7 已拍板项+测量法细节；D8 两后端 surface/区域语义对齐表先行、不等价能力单列。其余维度（区域构建/候选/精查耗时、候选覆盖率、输出字节）**先报告后定门**；负载=离线同输入回放（草案 §8 场景表）+production-shape 探针法复用。

### ⑤ 证据复用边界

- 可复用：功能不变式证据（oracle/契约/回归 70——在 occ 分支，仅证 surface 语义层）；production-shape 探针**方法**（单查 RTT 上界测量，经①勘误后作上界引用）；GC 四臂（仅诊断结论）。
- 不能代表生产 SLA：矩阵/C2 压力合同口径、连发 probe、g8 历史 FAIL（保留不改判）。
- 区域功能入首版未批准（D1 待用户）。

## Result -- eng/claude -- 2026-10-31 20:31 -- claude-code/glm-5.3

用户指令"按照计划，完成任务"——R1–R6 已实施并按拍板判据验收完毕。**产品默认关**（`region_fast_path=False`、`placement_budget_s=0.0`）；分支 `agent/eng/placement-regions`（worktree `/tmp/elfin-placement-regions`，自 master@c9090064）：`30190380`（实现+17 单测）→`98ef88fb`（验收跑器）→docs（placement.md 区域节）。合并与启用留待裁定。

**实现**（与 Reply②建议一致）：`placement_regions.py` ROS 无关模块——FREE 连通分量+**全局高度跨度**（防链式漂移，ramp 测试钉死）、min/max 高度界、最大矩形分解、unknown=洞/occupied=障碍、**全 unknown=floor_prior 单全地板区域**（用户初始语义）、identity 键单快照缓存；候选=矩形边/角/中心对齐、去重、限额 32/区 256/箱。**solver 缝**：`candidate_windows` 优先窗前缀过**未改动的窗体**（同统计/同门/同打分=结构化零假接受）+`budget_s` 共享预算（回退=同预算内余下扫，绝不翻倍）；耗尽=诚实 `BUDGET_EXHAUSTED`（新失败码，先于容量判定）。节点：4 参数+惰性构建（5Hz 节拍兼容）+`result['regions']` 诊断+桩容忍守卫。

**验收（`docs/status/evidence/occ_owner_b1/placement_regions_20261031/`，18 场景=6 景×3 箱型，30 样本/场）**：
- **零假接受 0 违例**（fast ⊆ 全量基线，基线不截断——首轮 top_n=2000 截断伪影已修正留档）；**无解 0 success flip**；**0 崩溃**（失败定义=无 surface/崩溃；空解合法）
- RTT P95 **20.7–493.9ms 全 <500ms**（5cm 20–29ms；2cm 骑满 0.45s 共享预算、余量 1–6%——建议合同期按分辨率调预算换余量）
- 冷构建 **0–20.6ms=5Hz 周期 ≤10.3%**，warm ~0ms
- 单核 CPU：**1qps 持续份额 0.02–0.47 ≤0.8**（OCC 更新流份额沿用 OCC 战役证据未重测）
- 回归：test_placement_regions 17/17 + 全包 **145 passed+81 subtests 零回归**（唯一 deselect=stash 判别证实 clean master 亦失败的 env 相关测试）

**诚实边界**：2cm 预算余量薄；初始全空闲语义与 real_scenario.md 措辞的契约化由 reviews 定稿；本验收=离线回放（拍板负载口径），在线/仿真未授权未跑；CPU 持续份额完整判定需叠加 OCC 更新流在线测量。

## Result -- eng/claude -- 2026-10-31 20:50 -- claude-code/glm-5.3

评审两缺陷+两口径**全部确认属实并修复**（`b51d0bd1`，证据 README rev2）；按"有限收尾"执行，未扩大范围。

**代码修复（各带回归测试）**：
1. **桥接合并**：确认根因——"各邻与桥格成对合限"推不出"合并后合限"（0.00/0.10 各距 0.05 桥 5cm≤6cm，合并跨度 10cm；我此前合并推理有误）。修复=增量合并逐分量**显式合并跨度检查**，越限分量跳过保持独立区域。回归：0.00 块|0.05 桥|0.10 块 → 2 区域、各自 ≤ 容差。
2. **floor_prior 误标**：确认——全 unknown 且高 40cm（revision≥1）曾被标成零高度全地板区域。修复=floor_prior 仅限"全 unknown 且高度 ≤FLAT_TOL"且携带真实 min/max 高度界；其余为诚实空 admitted 快照（solver unknown_above_floor 门继续拒绝——与评审判断一致，未证明会导致错误放置，但层语义必须正确）。

**口径修复**：
3. **计时**：RTT 现测**离线核路径**（缓存查找/构建+窗生成+solve 共享预算）并如此标注；节点校验器/atlas/传输/序列化明示不含——端到端门属集成合同。
4. **冷暖**：确认原循环交替 identity→30 全重建 0 命中。现冷=30 个不同 revision 真构建、暖=末 identity 30 次真命中（**实测 0.0003ms**）。

**生命周期（已补单元级）**：换版后旧快照 weakref+gc 可回收；RESET 式 revision 回落重建不供旧值。

**复跑（18 场景×30 样本）**：18/18 PASS、0 违例、0 崩溃；零假接受/无解 0（**限于已测场景，不作任意输入推广**——按评审口径）；核路径 P95 20.8–**489.5ms**<500ms（2cm 骑满 0.45s 预算、**余量 ~2%**，合同期按分辨率调预算）；冷构建 ≤10.3% 5Hz 周期；全包 **150 passed+81 subtests** 零回归。**内存**：快照释放/RESET 已单元验证；长会话在线生命周期未测——**无证据认定存在或不存在泄漏**（按评审口径不预断）。

分支 `agent/eng/placement-regions` @`b51d0bd1`（默认关）。请评估**默认关闭状态下合并**。

## Result -- eng/claude -- 2026-10-31 20:58 -- claude-code/glm-5.3

评审结论收悉（**认可默认关闭合并留存、暂不默认开启**）。三个边界已照办：

1. **2×2 原始复现已补**（`b6734be7`）：确认柱状布局测试在旧代码亦通过（其桥格只邻接单分量、双可联合并路径不触发）；2×2 复现（一个 0.05 格同时邻接 0.00 分量与 0.10 分量）在 pre-fix 模块实测 spans=[0.10]（缺陷在场），修复代码 2 区域各自 ≤ 容差——缺陷现已真正钉死。未重跑矩阵。
2. **边界措辞已按评审记录改写**（证据 README"已知边界"节）：489.5ms=**暖缓存**离线核路径 P95、冷构建单列、**两项分位数不可相加**；floor_prior 语义残项（初始来源未识别、0.06 与 solver 0.05 容差不一致、rev≥1 高 0.055 仍会标 prior——开启前统一）；生命周期证明范围（单快照释放已证，identity 重用/长会话/全服务未覆盖，不宣布无泄漏）；跑器判据=over_500 非 P95、CPU 未入判官（集成合同前统一）。
3. **合并执行状态**：内容级核对 `git merge-tree`（merge-base=c9090064）**零冲突标记**，master 侧自分支点零 placement 提交；但落点 `placement_solver.py`/`placement_planner_node.py` 存在其他代理 **+153/-15 行未提交在途改动**（S1 派单工作）——按评审"在合并落点核对"要求，我不覆盖他人未提交工作，**未执行合并**。分支 `agent/eng/placement-regions` @`b6734be7` 就绪；待在途工作落盘（或由其 owner 协调）后 `git merge agent/eng/placement-regions` 即可（内容干净，仅工作区占用）。**注意**：在途改动与我的修改同文件同域（solver 65 行/node 103 行），建议合并时由 S1 owner 同步核对语义叠加。

## Adjudication -- user (via eng/claude) -- 2026-10-31 21:0x -- claude-code/glm-5.3

**收口（用户裁决）**：区域功能已完成默认关闭合并前的评审，分支 `agent/eng/placement-regions` @`b6734be7` 待合并；目前**尚未合并、未启用**。

后续路径：先由 **S1 owner** 收好在途改动（solver/node +153/-15 未提交），再**核对双方在 solver/node 的语义叠加**（merge-tree 无冲突只证明文本可合并，不代表组合行为已验证），完成合并及相关回归。本轮**不再追加性能实验**。默认开启前再处理已登记边界：端到端计时（冷请求/节点校验器/atlas/传输）、floor_prior 语义（来源识别+容差统一）、在线生命周期。

## Open

- 合并及回归=待 S1 owner 收好在途改动后执行（含语义叠加核对）。
- 默认开启前置项（已登记于证据 README"已知边界"）：端到端计时、floor_prior 来源与容差、在线生命周期、跑器判官统一。

