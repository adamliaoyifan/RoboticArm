# ST09-I S5.7 设计文稿：运动生成统一收敛

> Driver 声明意图，Policy 决定目标，RouteProducer+MoveIt 决定轨迹，
> Certifier 决定可行性，Executor 只执行认证关节轨迹。
> 获批后落盘 `docs/architecture/motion_generation_convergence.md` 实施。

## 0. 问题与现状盘点

**问题**：评测血统 driver 把"想完成什么"与"怎么走"写在同处——临时拼
Cartesian/pose_target 段、Cartesian 探针、world-X 判断、固定槽位。执行器
按 S3 契约拒绝生成类指令（SEGMENT_GENERATION_REMOVED），这些调用点已是
死路且绕开认证。

**调用点盘点**（探索核实，file:line）：

| 调用点 | 现状 | 处置 |
|---|---|---|
| place_smoke `_exit_to_portal` :326-350 | 活（OCC 血统），ad-hoc cartesian；`_execute_segment` :709-725 与 I1 :677 的 place_exit 豁免让它能走裸 v1 PlanMotion | **删除** builder + 两处豁免；M3 的命名位覆盖上收到 CargoMap 层 |
| cargo_map `_lift_for_cargo_view` :452-523 | **死代码**（零调用者；`cargo_view_lift` 只残留在重试白名单与文档串） | 删除 + 清白名单 |
| cargo_map `_plan_suction_pose` :354-384 + `_execute_view_candidates` :386-450 | 活：aim 候选逐个试 pose_target（视角在帧内判定在 driver 侧） | **§4 观察链**接管 |
| motion_occ `_frame_probe_winner` :224-268 | 活但**实际必败**：构造 cartesian 探针 → ProbeMotionSegmentV2 对 cartesian 具名拒绝 | **§5 重验证**取代 |
| motion_occ `_place_probe_fn` :518-591 | 死代码（零调用者） | 删除 |
| `_fixed_slot` place_smoke :569-595 | M3 不可达（被 OCC 覆盖遮蔽；仅 plan_only/合成路径） | **§7 测试锁死** |
| place_only_eval_driver | 无 import 边，domain/感知自隔离 | §7 测试锁死 |
| pick 阶梯（BuildMotionSequenceV2 → pick_leg_records） | 活，服务侧生成 typed legs；**未经 ValidateRouteV2 认证**（:1179 文档自认） | 保留设计；认证升级见 §4' |

## 1. 区域分类（Zone）

替代 world-X 与固定 0.45。容器局部坐标四态：
`INSIDE_CONTAINER` / `OPENING_CORRIDOR` / `OUTSIDE_CONTAINER` / `UNKNOWN`
（2026-10-03 收名：原 `OUTSIDE_SAFE_REGION`——走廊深度推导只能分类 TCP
相对开口的位置，**不能证明整臂关节运动安全**，"SAFE" 授权语义过强）。

**分类 ≠ 授权**：`OUTSIDE_CONTAINER` 不授权裸 GoToRobotPose。即使处于该
状态，命名位运动仍必须经 MoveIt 建边 + 轨迹认证（§6 GoToRobotPoseV2）。

```
corridor_depth = opening_wall_thickness   # (outer−inner)/2，scene 数据（部署=0.04）
               + tcp_camera_envelope      # |T_tcp_camera|，URDF 静态 TF（≈0.267）
               + safety_margin            # 声明常量 0.10
```
部署代入 ≈ 0.41 m。实测锚点（已入测试）：pickup TCP 面外 1.70 → OUTSIDE；
开口嘴 +0.055 → CORRIDOR；舱内 −0.445 → INSIDE。

- 落点：`scene_tf_config_utils.container_zone(point, config, corridor_depth)`
  （纯几何）；corridor 组装在 luggage_planning（需 URDF TF）。
- **frame-runtime 下无 scene → `UNKNOWN` 具名拒绝 `SCENE_IDENTITY_MISSING`**；
  world-X fallback 仅显式 legacy 模式（§7）。
- 已有基础：`_container_zone`（place_smoke :396-434）改名为此四态即可，
  三态回归测试在 `test_home_arm_containment.py`（9/9）。

## 2. EnsureCycleStartV2 —— 周期起点状态机

**目的**：下一周期从已知、可认证状态开始。接口（新 srv，trajectory_selector
侧——它拥有 RouteProducer）：

```
Request:  current_joint_state[6] / current_tcp_pose / attachment_state
          scene_identity(scene_revision+geometry_hash) / current_map_revision
          target_observe_pose_name
Response: outcome  # ALREADY_READY | CERTIFIED_ROUTE | RECOVERY_REQUIRED
          route[]  # outcome=CERTIFIED_ROUTE：typed joint_path legs
          reason   # RECOVERY_REQUIRED 具名原因
```

状态处理（2026-10-03 边界锁定：完整 unloaded recovery 搜索**不阻塞** M3——
正常 place route 已含 release 后 retreat/return；未完成先具名
`ROUTE_INCOMPLETE` 停止即可，通用异常恢复单列后续任务；**必须完成**的是
认证化 GoToRobotPoseV2）：
- zone=OUTSIDE + 未携带 → `ALREADY_READY`（caller 走 §6 命名位——注意
  这只是"可以发起 §6 请求"，命名位运动本身仍走认证，见 §1 分类≠授权）。
- zone=INSIDE/CORRIDOR + **已释放** → producer 以 live q 为起点、released
  box 为世界障碍，搜 **unloaded exit 路线**（机制已存在：任意 live q 注册
  pick_home 模式 + `retreat_chain` exit→home 阶梯 + unloaded SceneState 组装
  `edge_scene_states` else 分支；缺的是服务面 + 非槽位通用 anchor + 认证
  接线——anchor 用开口几何而非槽位 TCP 合成，即 `_exit_to_portal` 的几何
  上收到 producer）。
- zone=INSIDE/CORRIDOR + **携带** → 保持真空；S5.7 首阶段返回
  `RECOVERY_REQUIRED(CARRY_EXIT_UNSUPPORTED)` 具名停会话（正常链不会把
  带箱机械臂留在容器内——route 未完成即 §3 的 ROUTE_INCOMPLETE 路径）；
  carry-exit 认证搜索列为后续项。
- TF/scene/attachment 不明 → `RECOVERY_REQUIRED(IDENTITY_UNKNOWN)`。

## 3. place_exit 并入认证路线（收尾）

Producer 路线已含 release twin + unloaded retreat/return（`best_full_route`
:426-483，释放后腿由 place 循环 :1361-1368 执行）。剩余：
- 删 `_exit_to_portal` builder 与 `_execute_segment`/I1 的 place_exit 豁免
  （删后 TYPED_SEGMENT_MISMATCH 全覆盖）。
- place 循环在 post-release 腿完成前终止 → 具名 `ROUTE_INCOMPLETE` →
  §2 恢复流程；不再补段。
- `STAGING_NAMES` 等死名清理。

## 4. 观察链：ObservationPolicy → PlanObservationRouteV2

**目的保留**：实测落点/姿态、真实传感器写入 cargo map、≥1 sensor-occupied
cell、下箱 revision、杜绝 GT 冒充 measured。

**ObservationPolicy**（`cargo_view_sequence.py` 升格，纯函数核已在）：
- 已有：aim 网格（raise×blend）、lookback eye、revision 链、停止规则
  （`view_stop`：integrate 成功 ∧ footprint≥1 ∧ revision 严格增）、
  `view_policy_record`+`policy_hash`（冻结记录）。
- 上移/新增：**目标选择**（今日由 driver 从 slot meta 拼箱顶目标——
  `target = slot_world + h/2` :281-286 移入 policy）、视角在帧内预判
  （`_candidate_in_frame` 投影门移入 policy，输入相机内参）、最大视角数与
  停止条件输出。输入：已放箱估计区域、地图 unknown/occupied、相机内参、
  容器几何、机器人状态、max_views；输出：0-2 个观察目标
  （相机/TCP 位姿 + 可见性预测 + 停止条件）。

**PlanObservationRouteV2**（trajectory_selector 新 srv）：

```
Request:  current_state(joints+tcp) / observation_targets[](PoseContractV2)
          scene_identity / latest_map_revision
Response: success / reason_code / route[](typed joint_path legs)
          route_digest
```

RouteProducer 把观察目标当图目标走同一 anchor→MoveIt edge→`_certify` 管线。
**真正新增（探索核实）**：certifier `_first_failure` 强制 AttachmentRecord
（None → ATTACHMENT_MISSING）——需要**显式无 attachment 准入分支**
（unloaded SceneState 组装已存在）；`MapRevalidator.revalidate(in_flight=False)`
直接复用做 admission 新鲜度（caller 供 latest_snapshot，
`latest_snapshot_fn` 模式已有）。

**§4'（本轮纳入，用户裁决 2026-10-03）**：pick 阶梯认证升级——无
attachment 准入打通后，**pick_leg_records 的边同轮接上 ValidateRouteV2
认证**（今日 :1179 自认未认证，与"pick 边也过 certifier"的目标差距）。
实现：waypoint_generator 的 pick 编码器在建边后组 unloaded 单场景 +
`MoveItStateValidityAdapter.load_edges` → `/validate_route_v2`（与 §4
同一客户端薄层）；认证失败具名拒绝（CERT_*），不降级执行。单箱链（§8.6）
前完成。

## 5. ValidateSelectedRouteV2 —— 执行前重验证（探针取代）

原探针目的保留；**新语义：不生成第二条路径**（认证 A、执行 A）。

```
Request:  route_digest(=SelectPlacePathV2 path_hash)
          legs[](即将执行的 motion_segments 原样)
          current_joint_state / attachment_identity
          scene_identity / latest_map_revision
Response: valid | 具名拒绝（DIGEST_MISMATCH / START_STATE_MISMATCH /
          IDENTITY_CHANGED / CERT_* / SCENE_*/...）
```

检查项与实现基础（探索核实）：
- **digest == 将发送轨迹**：`path_hash` 本就覆盖全部 knot 集
  （`PlannedRoute.digest` 内嵌 `TimedTrajectory.digest`）；客户端用
  `record_from_message`（motion_frame_adapters.py:88）重建逐 leg digest 比
  对（今日无人做，薄新增）。
- 起点/identity：当前 joints vs 首腿起点（≤ QP 门 0.01 rad 同标准）；
  scene/geometry/map revision、attachment（VacuumCommandV2 的
  BoundaryDiagnosticV2 回显全套 identity）vs 选择时。
- **全轨迹重认证**：独立薄客户端组 `load_state_scenes[]`（完整冻结场景，
  FrozenSceneComposer 配方已存在但私有）+ ≤10ms 采样 + 原 knots →
  `/validate_route_v2`（C++ 纯重验：不重规划、不跑 IK、echo digest）。
  已知坑：controls_states.py:307 用旧字段名（stale，勿抄）。
- 成功后执行的必须是**同一份** joint_path；失败 = 具名拒绝该 placement
  （删除 excluded 重选循环——重验失败不是换槽信号）。

**map 语义（2026-10-03 锁定：不是陈图回退）**——verify 的 map 行为四态：
1. latest map 可用且 identity/revision 合法 → 重验证（全路径）；
2. latest map **不可用**（拉取失败/服务无响应）→ 具名拒绝
   `MAP_STATUS_UNAVAILABLE`；**过期**（2s 新鲜度）→ 具名拒绝
   `MAP_STALE`；**revision 倒退** → 具名拒绝 `MAP_RESTART`；
3. latest revision 增加但不影响 route edges → 允许（记录新 revision）；
4. 影响 route corridor → 重认证或拒绝（certifier 裁决）。
**任何情况下不得在选择时旧 snapshot 上回退**。证据与测试名须写明此语义
（如 `test_map_status_unavailable_refuses_without_stale_fallback`）。

## 5b. 执行窗口闭环（admission generation）——S6 前必须

§5 证明了"认证 A、执行 A"；**尚未证明"执行 A 时认证仍有效"**。driver
调 verify → valid → 逐腿调执行服务之间，scene、map、attachment
incarnation 仍可能改变（peer restart / lease 过期 / identity change）。
闭环方案（用户 2026-10-03 锁定，选项 b+c）：

- **执行请求携带**：`route_digest` + `certification_id`（certificate
  digest）+ `admission_generation`（scene_revision / geometry_hash /
  map_revision / attachment incarnation / 授予时刻）。
- **executor 原子验证后提交**：第一条腿实际提交控制器前，向 selector
  的 admission 面核对 generation（机械比对，不做可行性判断——五层原则
  不破）；不匹配 → 具名拒绝，零提交。
- **后续每腿复核**同一 generation 未被吊销：selector 重启 →
  `ADMISSION_UNKNOWN`；identity/attachment 变 → `GENERATION_CHANGED`；
  map revision 前进且不影响 route edges（MapRevalidator affected_edges）
  → 仍 LIVE（记录新 revision）；影响 corridor → `REVOKED`。
- 语义：认证 A、执行 A、**且执行 A 时认证仍有效**；失败具名拒绝该
  placement，不降级、不重选。

`--place-probe on` 改调此面；`_frame_probe_winner` 与死代码 `_place_probe_fn`
删除。

## 6. GoToRobotPoseV2 —— 命名位姿收紧

- 只承担"从已确认安全区域进入已标定站位"；zone != OUTSIDE 拒绝（先 §2）。
- **过渡接线（2026-10-03 记录）**：`portal_observe_sim` 目前走旧 v1
  GoToRobotPose（裸 2 点 FJT + authorize_legacy_effect），仅视为 S5 调试期
  临时接线；GoToRobotPoseV2 必须在**正式单箱验收前**替换它。
- **两个真新增**（探索核实今日皆无）：(a) 执行时把位姿文件的
  `scene_revision_at_capture`（schema 已有、对冻结 cell 校验）与 live
  `RuntimeIdentityV2.scene_revision` 比对；(b) 运动走认证路径的接线
  （今日 v1 是裸 2 点 FJT + authorize_legacy_effect）。
- typed 封皮沿 `_leg_record`/`message_from_admitted` joint_path 模式。
- `portal_observe_sim` 保留为仿真专用已标定观察位（SIMULATION ONLY；
  deploy 实测前 valid:false 纪律不变）。

## 7. Legacy 隔离（测试锁死）

- `_fixed_slot`：仅显式 test mode；frame-runtime/production 具名拒绝；
  证据 `slot_source=test_fixture`；不得回退掩盖 placement 失败。
- world-X fallback：仅显式 legacy 模式。
- place_only_eval_driver：断言无 import 边（测试锁）。
- 删除项随行：`_lift_for_cargo_view`、`_place_probe_fn`、place_exit 豁免、
  `STAGING_NAMES`。

## 8. 实施顺序（用户给定）+ 运维项

1. §3 place_exit 并入 + 删共享 builder/豁免/死代码。
2. §1+§2 `_home_arm` → 四态分类状态机（分类改名先行；EnsureCycleStart 的
   unloaded-exit 路线随后；容器内携带箱 → 具名拒绝
   CARRY_EXIT_UNSUPPORTED，全量 carry 搜索为后续项——用户裁决）。
3. §5 探针 → 选中路线重验证（含 map 四态语义：无陈图回退）。
3b. **§5b 执行窗口闭环**（admission generation 携行 + 执行前/逐腿吊销
   检查）——S6 前必须，正式单箱验收前落地。
4. §4 观察链（policy 上移 + PlanObservationRouteV2 + 无 attachment 准入）
   **+ §4' pick 阶梯认证**（同一无 attachment 准能与客户端薄层，
   单箱链前完成——用户裁决纳入本轮；S6 不留未认证 pick）。
   边界（2026-10-03 锁定）：certifier 用**显式 LoadState.EMPTY**，不把
   attachment 缺失解释成 unloaded；ObservationPolicy 只选**最多两个**
   相机目标并预测可见性；TCP 转换、MoveIt 建边、认证属于
   RouteProducer；PlanObservationRouteV2 只返回完整 certified
   joint_path；每次观察后再按真实 integration/revision/occupancy 决定
   是否请求第二视角。
5. §7 legacy 锁死。
6. 单箱全链：pick→attach→placement→route→release→exit→observe→integrate→
   return（GoToRobotPoseV2 替换调试期 v1 接线后才是正式验收）。
7. 单箱过 → 三箱 M3。

**运维项（并行，解除当前 smoke 阻塞）**：semantic 卡死真因是
`import clip` 失败 → ultralytics 在线 AutoUpdate（`bbox_fill` 与
`yolo_world` 同类，都需 CLIP；vendored CLIP 在
luggage_perception/vendor/ + ~/.cache/clip 符号链接）。修法：
(a) 确保 vendor/ 进 overlay 安装（setup.py package_data 核查）；
(b) launch 传 `semantic_require_backend:=bbox_fill`（fail-fast，stub 回退
永不匹配）。
**正式结果要求（2026-10-03 锁定）**：(i) 先验证 require 的 backend 名称
确实合法（fail-fast 才有意义）；(ii) vendor 文件进 install overlay；
(iii) 禁止联网 AutoUpdate；(iv) 指定 backend 不可用 → 启动失败；
(v) 不允许静默回退 stub；(vi) 不依赖用户 cache 符号链接
（~/.cache/clip 不得是正确运行的前提）。

## 9. 认证/identity 复用备忘（实现期参考）

- `path_hash` = 所发 motion_segments 的完整轨迹摘要（逐 leg 可重建比对）。
- ValidateRouteV2 请求需完整冻结场景 per load state + load_state_index +
  knot + ≤10ms 采样；`model_sha256` 仅供记录。
- certifier 检查序：snapshot/attachment digest → attachment 资格（需新增
  无 attachment 分支）→ 边存在性 → 逐边（限位/时序/采样律/碰撞/occupancy）
  → 全局 clearance。
- MapRevalidator：2s 新鲜度 + geometry_hash + affected_edges；
  in_flight=False 即 admission 语义。
- 预算三常量在 grid_path_search 与 contract JSON 重复（JSON 仅文档）——
  已知债，不扩。
- site_pose_schema：`scene_revision_at_capture` 已是一等字段；identity 级
  srdf/urdf pin 验收器支持、deploy yaml 未携带（GoToRobotPoseV2 可加）。

## 10. 验证

- 单测：四态分类回归（扩 UNKNOWN）、digest/起点/identity 重验证断言、
  无 attachment 准入（观察腿认证正/负例）、**pick 腿认证正/负例**
  （认证失败具名拒绝、不降级执行）、legacy 拒绝锁、
  place_only 无 import 边。
- 单箱 smoke 判据 = §8.6 全链事件 + GT/传感器双路（m3 判定器）。
- 既有套件不回归（I 34/0、E、gazebo/planning 受影响面）。
