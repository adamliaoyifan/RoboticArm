# Todo 5 切片 A — place 动作详细设计

父计划：[closed_loop_place_pack.md](closed_loop_place_pack.md)。本文只管**一次 place**：
吸着箱子从 `pick_retreat` 出发，进集装箱、落位、松手、退出。

栅格更新（切片 B）、`ComputePlacement`（切片 C）、塞满循环（切片 D）不在本文。
本文的槽位是**外部给定**的 `SlotSpec`。

---

## 0. 复核父计划发现的三个硬伤

父计划把切片 A 写成「传入 `request.place_slot` + 填 `opening_info`」。实际读代码后，
这样接会直接跑错，原因如下。

> **已定**：world 与 base_link **不是**同一个系（实测差 yaw −90° + 0.86 m，见 A-1）。
> 按「pick / place 用同一个系」的要求，全链统一到 **world**。
> 这条决定同时决定了 A-1 和 A-2 的解法，见第 2 节。

### A-1 坐标系：place 是 base_link 原生，执行器只认 world（阻断）

| 事实 | 位置 |
|---|---|
| `MotionExecutor` 把 `pose_target` 和 cartesian 的 `header.frame_id` 都写死成 `world` | `motion_executor.py` `self._frame`，`_build_goal_constraints` / `_plan_cartesian` |
| ROS 1 明确把 place 段当作 base_link 位姿 | `ros1_reference/motion_planner_node.py::_place_segments_in_base_frame`（`stage_mid/stage_late/stage/transit/traverse/insert/descend`，`retreat` 带 `keep_tool_down` 时也算） |
| `SlotSpec` 生产者用 base_link | `placement_planner_node` `~base_frame` 默认 `elfin_base_link` |
| `waypoint_generator_node` 的 place marker 已经按 base_link 发 | `_publish_pick_markers`：`phase == "place"` → `elfin_base_link` |

`elfin_base_link` **不等于** world。用 `scene_tf.yaml.example` 实算
（pedestal `rotation_rpy [0,0,1.5708]`、高 0.86）：

```
container_in_base_link    t=[0, -1.5, -0.86]   rpy=[0, 0, -1.5708]
opening_center_base_link  [-0.270, -0.755, 0.440]
opening_normal_base_link  [0, +1, 0]
```

差一个 **yaw −90° + 0.86 m**。直接把 base_link 的槽位喂给写死 world 的执行器，
箱子会被送到绕 Z 转了 90° 的位置。

### A-2 staging 的数学只在 base_link 成立（阻断，且与 A-1 耦合）

`build_sequence` 里 stage 只在 **Y** 上外推：

```python
y=portal[1] + normal[1] * stage_distance
```

实算这个场景（`opening.side: negative_x`，container 相对 world 无旋转）：

| 系 | 开口法向 | `normal[1]` | staging |
|---|---|---|---|
| `elfin_base_link` | `[0, +1, 0]` | `+1` | 外推方向正确 |
| `world` | `[-1, 0, 0]` | `0` | **退化成「原地抬高」**，形同虚设 |

结论：staging 是**按 base_link 写死的**。既然 A-1 定成统一到 world，
这段外推就必须改成沿完整法向量的三轴通用写法，不能只动 Y。
这是本切片唯一一处几何逻辑改动，且可以纯 Python 单测覆盖（不需要仿真）。

### A-3 descend 是设计上的碰撞（阻断）

`contact_z = slot_center_z + box_height/2`，`descend` 把 `suction_contact_frame`
送到这个高度，等价于**箱底正好贴集装箱地板**。而：

- 箱子此刻是 `AttachedCollisionObject`，`touch_links` 只有
  `suction_panel / suction_contact_frame / elfin_link6 / elfin_link5`
  （`planning_scene_client.DEFAULT_TOUCH_LINKS`），**不含集装箱**
- `MotionExecutor` 默认 `cartesian_avoid_collisions=True`
- `insert` / `descend` / `retreat` 是 `allow_ompl_fallback=False`

→ `descend` 的 cartesian fraction 会在接触前掉下去，直接 `PLACE_FRACTION_descend`。
ROS 1 有对应开关 `_set_place_support_touch(allowed)`，Humble 侧没有。

### 其它已知、非阻断但必须记账

- `insert/descend/retreat` 带 `keep_camera_down=True`，执行器**未实现**，
  只是在 **success** 消息后面追加 `NOT_IMPLEMENTED: keep_camera_down`。
  不能因为段成功就以为相机朝下被约束住了。
- ROS 1 在 `descend` 之后、松手之前有 `_ensure_release_settled`
  （`vel_tol 0.03`、`timeout 3.0`、`hold_time 0.25`，失败会补发 hold 轨迹再判一次）。
  Humble 侧没有，直接 `enable:false` 等于「还在晃就松手」。

---

## 1. 范围与不做

**做**：`CARRY → 进箱 → 落位 → 松手 → 退出 → 回 observe` 一次，带全量 debug。

**不做**：packing 求解、占据栅格、多箱、`keep_camera_down` / `lock_wrist` 实现、
容器内 NBV、真机。

**前置**（Todo 4 已验收，直接用）：pick 四段 100%、`vac_attach`/`vac_follow` 10/10、
`GoToRobotPose` 已把 named pose 时长封顶到 4 s。

---

## 2. 已定的两个设计选择

### D-1 坐标系：pick 与 place 统一用 `world` ✅

执行器已经是 world 写死的，pick 四段在 world 下 100% 通过，gz 真值和落点校验也都在
world —— 所以统一到 world 的改动面最小，且校验链不用跨系。具体：

1. `world` 是**唯一**的运动规划系。`MotionExecutor` 不动。
2. `SlotSpec.place_pose` 消息里**没有** frame 字段，靠约定。约定写成：
   生产者（切片 C 的 `placement_planner`）产 `elfin_base_link`，
   **`waypoint_generator_node` 在边界处用 TF 转成 world**，`build_sequence` 之后
   一律是 world。这个转换点是唯一的，不允许在别处再转一次。
3. `opening_info` 也在 world 下组装。
4. **`build_sequence` 的 staging 改成三轴通用**（A-2）：沿完整 `normal` 向量外推，
   不再只加 `normal[1]`。
5. `waypoint_generator_node._publish_pick_markers` 的 place 分支从
   `elfin_base_link` 改成 `world_frame`，否则 RViz 里的 marker 会和实际目标差 90°。
6. 切片 A 的固定槽位直接在 world 下算（`container_link` 相对 world 无旋转，
   内底板中心 world = `[1.5, 0, 0.53]`，槽位中心 `z = 0.53 + box_h/2`，yaw = 0），
   先不经过 base_link。

debug 里**仍然两个系都记**（第 5 节 `pose_world` / `pose_base_link`）：
统一之后 base_link 那份是纯诊断用，一旦两者关系不是 yaw −90° + 0.86 m，
说明 TF 或 scene_tf 被改过。

### D-2 带着箱子失败时：`stop` ✅

`insert` / `descend` 失败时箱子还吸在臂上。默认行为：**就地停住，不松手、不重试**，
dump 现场后整个 run 结束。切片 A 的目的就是把第一次进箱的失败现场原样留住。

`park`（退回 transit → 飞回 pickup 平台放回 → 继续下一箱）留成
`--on-place-fail=park`，等切片 D 跑循环时再实现。**任何情况下都不做 `drop`。**

---

## 3. 状态机

一次 place 是一条线性链 + 两条逃逸边。每个状态有：进入条件、超时、失败码、
出状态时必须落盘的 debug。

### 3.1 状态表

| # | State | 进入条件 | 正常出口 | 超时 | 失败码 |
|---|---|---|---|---|---|
| P0 | `CARRY_READY` | pick 4/4 且 `vac_attach` 且 `retreat_ok` | P1 | — | `PLACE_PRECOND`（真空没吸住 / retreat 高度不对） |
| P1 | `SLOT_RESOLVED` | 拿到 `SlotSpec`（切片 A 由参数给定） | P2 | 5 s | `PLACE_SLOT_INVALID`（不在内腔 AABB 内）、`PLACE_SLOT_UNREACHABLE`（`/compute_ik` 无解） |
| P2 | `SEQUENCE_BUILT` | `BuildMotionSequence(phase=place)` 成功且段数 ≥ 5 | P3 | 15 s | `PLACE_BUILD_FAILED` |
| P3 | `STAGING` | 有 `stage_mid/stage_late/stage` 段时才进 | P4 | 每段 60 s | `PLACE_PLAN_<seg>` |
| P4 | `TRANSIT` | — | P5 | 60 s | `PLACE_PLAN_transit` |
| P5 | `TRAVERSE` | — | P6 | 60 s | `PLACE_PLAN_traverse` |
| P6 | `INSERT` | 开口通过判据成立（第 4 节 I-2） | P7 | 60 s | `PLACE_PLAN_insert` / `PLACE_FRACTION_insert` |
| P7 | `DESCEND` | 支撑面 ACM 已放开（A-3） | P8 | 60 s | `PLACE_PLAN_descend` / `PLACE_FRACTION_descend` |
| P8 | `RELEASE_SETTLE` | descend 成功 | P9 | 3 s + 一次 hold 重试 | `RELEASE_SETTLE_FAILED` |
| P9 | `RELEASED` | `VacuumCommand(false)` 成功且场景 detach+remove | P10 | 10 s | `VACUUM_DETACH` |
| P10 | `PLACE_RETREAT` | 已松手 | P11 | 60 s | `PLACE_PLAN_retreat` |
| P11 | `VERIFIED` | 采样箱子实际位姿 | P12 | 3 s | `PLACE_VERIFY_XY` / `_Z` / `_YAW` / `_TIP` / `_DRIFT` |
| P12 | `COMMITTED` | 切片 A 为空操作（切片 B 接 `AddPlacedBox`） | P13 | 10 s | `COMMIT_FAILED` |
| P13 | `HOME` | `GoToRobotPose(pickup_observe)` | 结束 | 30 s | `GOTO_FAILED`（**非阻断**，不进 place 分母） |

### 3.2 逃逸边

```
P3..P7 任一失败  ──► ABORT_CARRYING   (箱子仍在臂上，真空仍开)
P8 失败          ──► ABORT_CARRYING   (宁可不松手)
P9..P12 失败     ──► ABORT_RELEASED   (箱子已在容器里，臂是空的)
```

- `ABORT_CARRYING`：按 D-2 = `stop`：**不** `enable:false`、
  **不** `ClearCurrentBox`、**不**回 observe，dump 后整个 run 结束。
- `ABORT_RELEASED`：箱子已落地，臂必须先退开再收工。执行 `retreat` → 回 observe
  → dump。切片 B 之后这里还要保证**不 commit**，否则栅格会比现实多一个箱子。

### 3.3 全程不变量（每次状态切换都校验并记录）

| ID | 不变量 | 检查窗口 | 违反 |
|---|---|---|---|
| I-1 | `vacuum.attached == true` 且 `fail_reason == ""` | P0 → P8 每段前后 | `PLACE_LOST_PAYLOAD` |
| I-2 | 箱子 AABB 在开口孔径内（`aperture.corners`，含 margin） | P6 进入前 | `PLACE_APERTURE` |
| I-3 | gz 箱子中心与吸盘的相对位姿漂移 < 5 cm | P4/P5 各采样一次 | `PLACE_FOLLOW_DRIFT` |
| I-4 | `follow_skipped` 增量为 0 | 全程 | 记录，不阻断 |
| I-5 | 松手后箱子位移 < 2 cm | P11（隔 `drift_wait` 再采一次） | `PLACE_VERIFY_DRIFT` |
| I-6 | 箱子中心在内腔 AABB 内（`point_inside_container_inner_box`） | P11 | `PLACE_VERIFY_XY/_Z` |

I-1 是最重要的一条：没有它，「规划失败」和「箱子半路掉了」会混成同一个码。

---

## 4. 每段的技术要点

### transit（pose_target，OMPL）
带载做自由空间大位移。箱子是 attached object，会参与碰撞检查。
调试关注：`MoveGroup error_code`，以及 IK 是否走了 `_ik_joints` 分支
（消息里是 `IK joint goal` 还是 `pose constraint fallback`）。

### traverse（cartesian，允许 OMPL 回退）
从 portal 平移到槽位正上方。fraction 低于 `cartesian_min_fraction=0.95` 会回退 OMPL，
**消息里会带 `fallback from cartesian 0.xxx`** —— 这条要单独记成字段，
否则「成功了但其实是回退」看不出来。

### insert（cartesian，禁止回退）
下降到 `contact_z + insertion_clearance(box_h, clearance)`，
即 `max(0.06, min(0.35*box_h, 0.15))`。这是唯一由箱高决定的量，记进 debug。

### descend（cartesian，禁止回退）
落到接触。**必须先处理 A-3**，两条路线二选一，在实现时定：

- **d-1（建议）**：落位前把「箱子 ↔ 集装箱地板/已放箱」加进 ACM 允许集
  （对应 ROS 1 `_set_place_support_touch(True)`），退出 P7 后再关掉。
- d-2：`descend` 目标抬高一个 `release_gap`（例如 5–10 mm），靠松手后自由落体贴地。
  代价是落点 Z 误差变大，且 I-5 漂移判据要放宽。

无论哪条，`release_gap` / ACM 开关状态都要落进 `trial.json`。

### retreat（cartesian，禁止回退）
抬回 `contact_z + place_clearance_z`。此时臂是空的，箱子必须**不动**（I-5）。

---

## 5. Debug 设计（事后可追溯）

原则：**任何一个失败码，都能只靠 dump 目录复现判断，不用回头看终端**。

### 5.1 落盘结构

沿用现有约定（`write_trial_dump` 的 `trial_%02d_<FAILCODE>` 风格）：

```
docs/status/evidence/place_smoke_<tag>/
  summary.json                 聚合
  trials.jsonl                 每箱一行
  dumps/place_00_ok/
    trial.json                 见 5.2
    segments.jsonl             每段一行，见 5.3
    tf_trace.jsonl             suction_contact_frame + 箱子位姿时间序列
    vacuum_events.jsonl        /vacuum/diag 原样落盘
    state_timeline.jsonl       状态迁移，见 5.4
    slot.json                  槽位（两个坐标系都存）
    scene.json                 实际下发的 PlanningScene diff 摘要
```

### 5.2 `trial.json` 关键字段

```jsonc
{
  "index": 0,
  "place_state": "COMMITTED",        // 死在哪个状态，一眼可见
  "fail_code": "",
  "catalog_id": "carryon",
  "box_size": [0.487, 0.390, 0.25],
  "slot": {
    "planning_frame": "world",        // D-1：全链唯一规划系
    "pose_world": {...},
    "pose_base_link": {...},          // 纯诊断：两者应差 yaw -90° + 0.86 m
    "frame_delta_ok": true,           // 上一条自动校验
    "yaw": 0.0,
    "source": "fixed_floor_center"    // 切片 C 后变成 compute_placement
  },
  "opening_info": {
    "frame": "world",
    "point": [...], "normal": [-1.0, 0.0, 0.0],
    "outward_clearance": 0.15,
    "stage_outward_clearance": 0.65,
    "stage_offset": [-0.65, 0.0, 0.0], // 三轴通用外推的实际位移，A-2 回归哨兵
    "staging_degenerate": false        // |stage_offset| < 0.1 时置 true
  },
  "insertion_clearance": 0.0875,
  "release_gap": 0.0,
  "place_support_touch": true,       // A-3 用了哪条路线
  "unimplemented_flags": ["keep_camera_down"],
  "verify": {
    "box_pose_after_release": {...},
    "box_pose_after_drift_wait": {...},
    "err_xy": 0.012, "err_z": 0.004, "err_yaw": 0.02,
    "roll": 0.001, "pitch": 0.002, "drift": 0.003,
    "inside_inner_box": true
  },
  "invariants": {"I1": "ok", "I3": 0.021, "I5": 0.003},
  "timings": {"P4": 6.1, "P7": 3.4, "total": 41.2},
  "wall_time_sec": 41.2
}
```

### 5.3 `segments.jsonl` 每段一行

```jsonc
{
  "seq": 3, "name": "descend", "type": "cartesian",
  "reference_frame": "world",
  "target_pose_world": {...}, "target_pose_base_link": {...},
  "keep_tool_down": true, "keep_camera_down": true, "lock_wrist": false,
  "allow_ompl_fallback": false,
  "ok": true,
  "message": "cartesian ok (fraction 1.000); NOT_IMPLEMENTED: keep_camera_down",
  "fraction": 1.0,
  "used_ompl_fallback": false,       // 从 message 里 "fallback from cartesian" 解析
  "moveit_error_code": null,
  "plan_sec": 0.42, "exec_sec": 3.1,
  "joints_before": [...], "joints_after": [...],
  "suction_before": [x,y,z], "suction_after": [x,y,z],
  "box_gz_before": [x,y,z], "box_gz_after": [x,y,z],
  "vacuum": {"attached": true, "fail_reason": "",
             "contact_distance": -0.004, "retention_margin": 7.11,
             "tilt_deg": 0.0, "follow_skipped": 0},
  "settle": {"criterion": "displacement", "peak_excursion": 0.0007,
             "excursion_joint": "elfin_joint2", "tail_ratio": 0.98}
}
```

`suction_*` 与 `box_gz_*` 成对存在，是判断「臂动了但箱子没跟上」的唯一依据。

### 5.4 `state_timeline.jsonl`

每次状态迁移一行，ROS 时间和墙钟都要：

```jsonc
{"t_ros": 812.44, "t_wall": 1756...., "from": "TRAVERSE", "to": "INSERT",
 "guard": "aperture_ok", "note": "clearance=0.0875"}
```

失败时最后一行是 `{"to": "ABORT_CARRYING", "guard": "PLACE_FRACTION_descend"}`。

### 5.5 在线可见性

- 驱动把当前状态发到 `/luggage/place/state`（`std_msgs/String` JSON，
  RELIABLE + TRANSIENT_LOCAL），跑的时候 `ros2 topic echo` 就能看到卡在哪。
- `waypoint_generator` 已有的 `/luggage/debug/pick_targets` MarkerArray 保持发，
  place 段沿用（注意 marker frame 要与 D-1 的选择一致）。
- 每段结束打一行单行日志：`place[3] descend ok frac=1.000 f_ompl=0 vac=1 dz=-0.150`。

### 5.6 debug 必须能回答的问题

设计验收标准就是这五个问题能只看 dump 回答：

1. 失败发生在哪个状态？→ `place_state` + `state_timeline` 最后一行
2. 是规划不出来，还是执行报错？→ `segments.jsonl` 的 `fraction` vs `moveit_error_code`
3. 箱子是全程跟着的吗？→ `suction_* / box_gz_*` 配对 + I-3
4. 坐标系有没有转错？→ `pose_base_link` vs `pose_world` + `staging_degenerate`
5. 松手时机对吗？→ `settle` 段 + `RELEASE_SETTLE` 的耗时与 `tail_ratio`

---

## 6. 交付物

| 文件 | 改动 |
|---|---|
| `luggage_planning/luggage_planning/waypoint_generator.py` | **staging 改三轴通用**（A-2）；返回 `stage_offset` 供自检 |
| `luggage_planning/scripts/waypoint_generator_node.py` | 传 `request.place_slot`；槽位 base_link→world 的**唯一**转换点；在 world 下组 `opening_info`；place marker frame 改 `world`；`staging_degenerate` 自检 |
| `luggage_planning/luggage_planning/motion_executor.py` | 不改参考系（D-1 统一 world）；只把 `fraction` / `error_code` / 是否走了 OMPL 回退回传成结构化字段 |
| `luggage_planning/scripts/motion_planner_node.py` | `descend` 后的 release-settle 门；place 支撑面 ACM 开关 |
| `luggage_planning/luggage_planning/planning_scene_client.py` | 支撑面 touch link 集（A-3 d-1） |
| `luggage_gazebo/scripts/place_smoke_driver.py` | 新：第 3 节状态机 + 第 5 节 debug |
| `luggage_gazebo/luggage_gazebo/place_metrics.py` | 新：无 ROS 的聚合 + 失败码归类 |
| `luggage_gazebo/test/test_place_metrics.py` | 新 |
| `luggage_planning/test/test_waypoint_generator.py` | 补：staging 三轴通用用例 —— 开口法向分别取 `[-1,0,0]`（本场景 world）和 `[0,1,0]`（旧 base_link），两者都必须产生非退化外推 |

`pick_retreat_eval_driver.py` **不动**，n50_v2 基线保持可复现。

---

## 7. 验收（N=3，carryon，loafbrr）

| 项 | 门槛 |
|---|---|
| 五段（含 staging）全成功 | 3/3 |
| `descend` 的 cartesian fraction | ≥ 0.95，且**没有**触发 OMPL 回退 |
| 松手时机 | 3/3 先过 `RELEASE_SETTLE` 才 `enable:false` |
| 箱子落点 | `err_xy < 0.04`、`err_z < 0.03`、`inside_inner_box == true` |
| 松手后漂移 I-5 | < 0.02 m |
| 倾倒 | `|roll|,|pitch| < 2°` |
| 载荷丢失 I-1 | 0 次 |
| `GOTO_FAILED` | 记录但不计入 place 通过率 |

产出：`docs/status/evidence/place_smoke_n3/` + `docs/status/todo5_place_action.md`。

---

## 8. 实施顺序（每步都能独立验证，失败不牵连下一步）

1. **S1 三轴 staging 单测**（无需仿真）：先把 `build_sequence` 的外推改成沿完整
   `normal`，用 `[-1,0,0]` 和 `[0,1,0]` 两组法向做单测，确认 `stage_offset` 都非退化。
   *已实测的期望值*：world 系下 `normal = [-1, 0, 0]`，`stage_offset ≈ [-0.65, 0, 0]`。
2. **S2 干跑**：给定 world 槽位 → `BuildMotionSequence(place)` → 每段跑 `/compute_ik`
   和 `GetCartesianPath`（**只规划不执行**），把 fraction 全量落盘。
   A-3 会在这里暴露，且此时臂没动、箱子没吸。
3. **S3 空载真跑**：不吸箱子，执行五段。验证轨迹本身可达、不撞容器。
4. **S4 带载真跑**：接上 pick + 真空，跑完整状态机，N=3 验收。

S2 是关键节流点：**只有 S2 的 fraction 全绿才允许进 S3**。
