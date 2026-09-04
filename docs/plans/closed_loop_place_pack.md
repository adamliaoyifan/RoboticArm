# Todo 5 — place 动作 + 占据栅格 + 顺序装箱

对应 sim 闭环方案 M4 剩余：pick 已验收，下一步是 **带着箱子放入集装箱**，
place 后更新内部占据，再按占据栅格和下一个箱子体积算下一槽，直到塞满。

Todo 4 停在 `pick_retreat` + 关真空 + 回 `pickup_observe`。那是评测复位，
不是工作流。本 todo 接上真正的 place。

## 范围

一条 Humble 链：

```
pickup_observe → spawn → detect → pick (真空开)
  → ComputePlacement(当前箱体积 + 占据栅格)
  → BuildMotionSequence(phase=place) → PlanMotion 各段
  → 真空关（箱子留在槽位）
  → AddPlacedBox（栅格 rasterize 该 AABB）
  → 回 pickup_observe → 下一箱
  → 直到 ComputePlacement 失败（BIN_FULL）或 spawn 尽
```

**不做**：完整 orchestrator / GUI、集装箱内部 NBV 探索、
把 ROS 1 `placement_planner_node.py` / `cargo_volume_mapper_node.py` 整文件搬过来、
`keep_camera_down` / `lock_wrist`、多臂、真机。

第一版占据以 **几何 commit**（`mark_placed_box`）为准；深度积分是加分项，
不挡 A/C/D。

## 已有、不要重做

| 块 | 状态 |
|---|---|
| Pick 四段 + 真空 attach/follow | Todo 3/4，T6 10/10 |
| `build_sequence(..., phase="place")` | 算法已有：`transit → traverse → insert → descend → retreat` |
| `CargoVolumeMapper` | `mark_placed_box` / `surface_map_2d` / `integrate_points` |
| `placement_solver.generate_candidates` | 空图走 floor-prior；未知高于地板拒绝堆叠 |
| msgs | `ComputePlacement`、`AddPlacedBox`、`VerifyPlacedBox`、`SlotSpec` |
| ROS 1 参考 | `scripts/ros1_reference/cargo_volume_mapper_node.py`；packing 脚本仍是 rospy，**未 install** |

## 当前缺口（按依赖）

1. `waypoint_generator_node._handle` 把 `place_slot` 丢成 `None`，也没有
   `opening_info`。`phase=place` 现在编不出可用轨迹。
2. 评测在 retreat 后立刻 `enable:false` 并 `GoToRobotPose(observe)`。
   工作流要 **真空一直开到 place descend**。
3. 没有 Humble `cargo_volume_mapper` 节点，place 后栅格不会更新。
4. 没有 Humble `ComputePlacement` 服务，下一箱槽位算不出来。
5. 没有「直到塞满」的驱动（`pick_retreat_eval_driver` 明确不做 place）。

---

## 切片 A — 单槽 place 动作（先证明进得去）

**详细设计见 [todo5_place_action.md](todo5_place_action.md)**：状态机、不变量、
失败码、debug 落盘结构、S1–S4 实施顺序都在那里。

目标：吸着箱子，沿 opening 进箱，放到 **已知槽**，松手，箱子留在集装箱里。

A 的槽位：从 `scene_tf` 取集装箱内底板中心 + 当前箱 `height/2`，yaw=0。
**先不接 packing solver**，避免运动和求解器同时 debug。

读代码后发现三个阻断项，接线前必须先处理（详见该文第 0 节）：

| # | 问题 | 处置 |
|---|---|---|
| A-1 | place 段是 `elfin_base_link` 原生，`MotionExecutor` 却把 frame 写死成 `world`；实测两系差 yaw −90° + 0.86 m | **已定：pick/place 统一用 `world`**，槽位在 `waypoint_generator_node` 边界转一次 |
| A-2 | `build_sequence` 的 staging 只沿 **Y** 外推，本场景开口是 `negative_x`；world 系下 `normal[1]=0` 会退化成原地抬高 | staging 改成沿完整法向量的三轴通用写法，纯 Python 单测覆盖 |
| A-3 | `descend` 落到箱底贴地板，属于设计上的接触，但箱子的 `touch_links` 不含集装箱，`avoid_collisions=True` 且该段禁止 OMPL 回退 | 放开支撑面 ACM（对齐 ROS 1 `_set_place_support_touch`），备选是留 `release_gap` |

另有两条非阻断但要记账：`keep_camera_down` 未实现（只在成功消息后追加
`NOT_IMPLEMENTED`），以及 ROS 1 的 `_ensure_release_settled`（松手前的静止门）
在 Humble 侧还没有。

验收（N=3，carryon 即可）：

- place 五段 `PlanMotion` 成功，`descend` fraction ≥ 0.95 且未走 OMPL 回退
- 真空在 `RELEASE_SETTLE` 通过之后才 `enable:false`
- gz 箱子中心进容器内腔（不在 pickup 平台），松手后漂移 < 2 cm
- 不把 `GOTO_FAILED` 算进 place 失败

## 切片 B — place 后更新占据栅格

目标：每放成功一箱，容器体素里出现该 AABB；`surface_map_2d` 高度抬高。

交付：

| 文件 | 性质 |
|---|---|
| `scripts/ros1_reference/cargo_volume_mapper_node.py` | 现有 rospy 挪走（若仍在 scripts/） |
| `luggage_perception/scripts/cargo_volume_mapper_node.py` | 新 Humble 薄壳 |
| 服务 | `AddPlacedBox` / `RemovePlacedBox`（沿用 ROS 1 `~mark_placed_box` 语义） |
| 话题 | `/luggage/cargo_map/surface_2d`（JSON string，合同与 `surface_map_2d()` 一致） |
| launch | `use_cargo_map:=true` 时启动 |

驱动在 vacuum off 且箱子已落槽后调 `AddPlacedBox`。栅格用
`SOURCE_GEOMETRY` rasterize，fail-closed：没 commit 的箱子对下一轮
`ComputePlacement` 不可见。

B2（可选，本 todo 不挡验收）：`integrate_points` 一次箱内深度。需要相机看进
开口，等于把 cargo exploration 拉进来。放到 Todo 6。

验收：空图 `occupancy_ratio≈0`；A 放 1 箱后对应柱高度 ≈ 箱高；
`unmark` 后恢复。

## 切片 C — 按栅格 + 下一箱体积算槽

目标：`ComputePlacement(box, placed[])` 读 `surface_2d` 和请求里的
`width/depth/height`，返回 `SlotSpec`。

交付：

| 文件 | 性质 |
|---|---|
| `luggage_packing/scripts/ros1_reference/placement_planner_node.py` | rospy 搬家 |
| `luggage_packing/scripts/placement_planner_node.py` | 新 Humble 壳 |
| CMakeLists | `install(PROGRAMS ...)` |
| launch | `use_packing:=true` |

求解器已有规则，不要改：水平放置、yaw ∈ {0, π/2} 足迹、
`z = peak + height/2`、未知高于地板拒绝、`top_n` 打分。
第一箱走 **floor-prior**（空图）。可达性 atlas 本切片可关，place 失败再打开。

验收（离线 + 仿真）：

- 空图 + carryon → 槽在底板上、箱体在内腔 AABB 内
- 已 mark 一箱后再要同尺寸 → 新槽足迹不与第一箱重叠
- 剩余空间放不下 large → `success=false`，message 含 `BIN_FULL` / `no_candidate`

## 切片 D — 塞到满

目标：新驱动循环直到满，不是 N 次固定 pick。

交付：`luggage_gazebo/scripts/pack_eval_driver.py`（不要把 place 塞进
`pick_retreat_eval_driver` 的默认路径，n50 基线必须还能真空关、只测 pick）。

每轮：

1. `GoToRobotPose(pickup_observe)` → spawn → detect → pick + 真空
2. `ComputePlacement`；失败 → 记 `BIN_FULL`，停
3. `BuildMotionSequence(place)` + 五段执行；失败 → 记 `PLACE_*`，可选 unmark 不 commit
4. 真空关 → `AddPlacedBox`
5. 回 observe，下一箱

停条件（任一）：`BIN_FULL`、`SpawnNextBox` 失败、`--max-boxes`。

验收（mesh catalog，`carryon,standard` 或 mixed，`--max-boxes` 足够大）：

- ≥1 次成功 place + 栅格 commit
- 最后一轮 `fail_code=BIN_FULL`（或达到体积上限），不是随机 `GOTO_FAILED`
- 栅格里的箱数 = 成功 place 数
- gz 内腔可见对应数量的箱子

报表：`docs/status/evidence/pack_eval_*/` + `docs/status/todo5_place_pack.md`。

---

## 实现约束

- 评测仍用 `ROS_DOMAIN_ID=7`。停 sim：launch PID / 进程组，禁止
  `pkill -f "ign gazebo"`。
- 真空跟随仍是 30 Hz `set_pose`；place 途中不要 detach。
- `named_pose_duration` 已改为 4 s 封顶（按关节差 / 1 rad/s）。回 observe
  只发生在 **两箱之间**，失败不进 place 分母。
- 已运行的 launch 读的是 install 树；改 planner / launch 后要 `colcon build`
  再起。

## 风险

| 风险 | 处理 |
|---|---|
| 无 `opening_info` 的 place 会撞箱壁 | A 必须接 `container_opening_frame` |
| 松手后箱子倾倒，AABB 与栅格不一致 | D 先记几何槽；VerifyPlacedBox 可后补 |
| vintage + 新 observe → `YOLO_NOT_READY` | D 先 `sequence_ids:=carryon,standard` 或 loafbrr |
| 带着 vac 做大关节 FJT 超时 | place 用笛卡尔段；observe 复位用已缩短的 named pose |
| 规划场景仍 attach 着箱子 | descend 后 `VacuumCommand false` 必须 detach scene |
