# 装箱全流程评测指标体系（v1）

状态：**评测规范**。定义装箱 pipeline 的完整性能口径，供评测驱动实现与报表
生成。所有指标均给出：定义、计算位置（谁算）、数据来源（哪个话题/落盘文件）、
口径规则（分母怎么取）、v1 阈值（初值，P95 回填后成契约，同 Todo 2 的做法）。

适用范围：仿真闭环（Fortress）。真机复用时把仿真时间列换成墙钟，其余不变
（见 [corridor_constraints.md](corridor_constraints.md) 部署备忘）。

---

## 0. 指标总览（六组）

| 组 | 回答的问题 | 指标数 |
|---|---|---|
| A 装箱效率（核心） | 箱子占了多少、装了几只、还剩多少 | 5 |
| B 节拍 | 一个箱子从头到尾多久、慢在哪 | 7 |
| C 可靠性 | 成功率、失败在哪层 | 6 |
| D 抓放质量 | 放得准不准、稳不稳 | 6 |
| E 安全/约束 | 有没有违反走廊、碰撞、误吸附 | 4 |
| F 资源 | 规划延迟、CPU/内存、仿真健康 | 5 |

---

## A. 装箱效率（suite 级，一轮装箱结束算一次）

**A1 体积占有率 `volume_fraction`（suitcases 占内部空间比例，主指标）**

```
volume_fraction = Σ(已放入箱子体积) / 内腔可用体积
```

- 分子：`AddPlacedBox` commit 过的箱子 AABB 体积之和（`W×D×H`，GT 尺寸）
- 分母：内腔 `inner_l × inner_w × (ceiling_z − floor_z)`（scene_tf：
  1.49 × 1.97 × 1.48 = 4.34 m³）
- 计算位置：评测驱动（从 cargo_map 的 committed_boxes 或驱动自己的 commit
  账本；两者必须一致，不一致记 `MAP_MISMATCH`）
- v1 参照：几何上限 standard 81% / carryon 51% / large 35%（纯网格堆叠，
  无间隙）；**实测 ≥ 40%（standard 混合目录）为 v1 及格线**，因走廊约束会
  主动放弃深堵槽位
- 注意：这是 commit 口径（几何真值），不是体素口径。体素口径见 A4

**A2 地板覆盖率 `floor_coverage`**

```
floor_coverage = 已放箱子底面投影面积 / 内腔地板面积（1.49 × 1.97）
```

- 数据源：`surface_map_2d()` 中 `height > floor_z + ε` 的格子占比，或直接
  由 commit 账本算 AABB 投影并集（v1 用账本，栅格交叉验证）
- 意义：第一层装满前不上二层；A2 停滞 + A1 增长 = 开始堆叠
- v1 参照：这是**面积比**不是体积比。standard 8 只铺满一层 ≈ 8×0.70×0.45 /
  (1.49×1.97) ≈ **86%**。16% 是第一层体积占内腔的比例（A1），不要和 A2 混用。

**A3 装箱数量 `boxes_packed` / `bins_used`**

- 成功 COMMITTED 的箱子数；同一内腔为 1 bin
- 附带：按 catalog 分组计数（carryon/standard/large 各几只）

**A4 体素占用一致性 `voxel_occupancy_ratio` vs `volume_fraction`**

- `GetCargoMapStats.occupancy_ratio`（体素）与 A1（AABB）之差
- 用途：校验 rasterize 正确性；**|差| > 5% 记 MAP_MISMATCH（fail-closed，
  该轮 A 组数据标无效）**
- 数据源：cargo_volume_mapper（切片 B 提供）

**A5 终止原因 `termination_reason`**

- 枚举：`BIN_FULL`（ComputePlacement 无可行槽）/ `SPAWN_EXHAUSTED`
  （箱源用尽）/ `ABORT`（连续失败达到上限）/ `TIMEOUT`
- BIN_FULL 时必须附：被否的候选数与 reason 直方图
  （`corridor_blocked` / `outside_aperture` / `unknown_above_floor` /
  `clearance_top` / `overlap`），否则无法归因是"真的满了"还是"约束太紧"

---

## B. 节拍（trial 级，每箱一条；suite 级汇总 P50/P95）

**B1 完整周期 `cycle_sec`**（主节拍指标）

```
cycle = spawn 完成时刻 → 该箱 COMMITTED 时刻（墙钟）
```

- 含：检测 + pick 四段 + 真空 + 转运 + place 各段 + 松手 + commit
- **不含**：回 observe / 下一箱 spawn 等待（那些进 B2）
- v1 阈值：p50 ≤ 60 s（当前实测 pick 18s + place 22–30s ≈ 40–48s，留余量）

**B2 端到端装箱吞吐 `boxes_per_min`**

```
boxes_per_min = boxes_packed / (总时长 − 首箱 spawn 前的准备时间)
```

- suite 级一条；与 B1 的差 = 复位/间隙开销

**B3–B6 分段耗时**（`PlanMotion.Result.plan_sec` + 执行墙钟，已有数据源）

| 指标 | 定义 | v1 阈值 |
|---|---|---|
| B3 `pick_motion_sec` | pre_grasp→approach→attach→retreat 四段墙钟和 | ≤ 25 s |
| B4 `place_motion_sec` | transit→…→place_retreat 墙钟和 | ≤ 35 s |
| B5 `plan_sec` per segment | 规划延迟（**墙钟**，真机同口径） | transit ≤ 8 s，笛卡尔段 ≤ 2 s |
| B6 `settle_sec` per segment | settle 等待 | p95 ≤ 10 s |

**B7 感知就绪延迟 `spawn_to_detect_sec`**（已有）

- v1 阈值：p95 ≤ 5 s（当前 mean 0.1s，语义链热身后）

---

## C. 可靠性（分母独立，沿用 Todo 4 三率原则：不混）

| # | 指标 | 分子 / 分母 | v1 阈值 |
|---|---|---|---|
| C1 | 检测过门率 | DetectionAccuracy.ok 数 / 有感知结果数 | ≥ 70%（现 72%） |
| C2 | 规划执行率 | 4+7 段全成功数 / 检测过门数 | 100%（现 100%） |
| C3 | 抓取保持率 | 真空 attached 从 pick attach 到 place release 无 fail 数 / attach 成功数 | 100%（现 10/10） |
| C4 | 放置成功率 | COMMITTED 数 / 进箱开始数（带载跨过 portal） | ≥ 90% |
| C5 | 端到端装箱率 | COMMITTED 数 / spawn 数 | ≥ 60% |
| C6 | 失败直方图 | fail_code 计数（DETECT_GATE:* / VACUUM_* / SEG_* / GOTO_*）+ 归因层标注（感知/规划/执行/真空/驱动） | 信息型 |

**口径规则**（防止重复 Todo 2 的假通过）：
- GT 回退的检测记 C1 分母、不记分子；`fail_code` 保留原始 reason
- `GOTO_FAILED`（回 observe 超时）不进 C4/C5 分母（复位不算装箱动作），但
  单独计数——它增多说明节拍恶化
- 连续 3 箱同层失败 → suite 提前终止（`ABORT`），已放数据保留

---

## D. 抓放质量（trial 级，沿用现有验收口径）

| # | 指标 | 定义 | v1 阈值 |
|---|---|---|---|
| D1 | 检测 xy 误差 | DetectionAccuracy.err_xy | p95 ≤ 3 cm（现为贴线） |
| D2 | attach 对 GT 的 xy/z | 真空 attach 时 suction 对 GT 箱中心 | xy p95 ≤ 3 cm，\|z\| ≤ 2 cm |
| D3 | 落点误差 | release 后 gz 箱中心 vs 槽位目标 | xy ≤ 4 cm，\|z\| ≤ 3 cm（现 11.5/−3.7 mm） |
| D4 | 松手漂移 | release→settle 后箱心位移 | < 2 cm（现 9 µm） |
| D5 | 倾倒 | release 后箱体 roll/pitch | < 2°（现 <1.13°） |
| D6 | 堆叠压塌 | 下层箱在放箱后 Δz | < 1 cm（多箱引入） |

数据源：`ign model --pose`（gz 真值）+ `segments.jsonl` 已有字段。

---

## E. 安全与约束（suite 级汇总，每违反记一次）

| # | 指标 | 检测方式 | v1 阈值 |
|---|---|---|---|
| E1 | 走廊违反 | retreat/extract 段高度 < 该槽走廊高度（轨迹点采样核对） | 0 |
| E2 | OMPL 箱内回退 | `used_ompl_fallback=true` 且段在箱内（insert/descend/retreat） | 0 |
| E3 | 误吸附 | VacuumCommand 成功但 gate 判据不满足（伪 attached） | 0 |
| E4 | 载荷丢失 | C3 的补集，带丢失时刻与当时臂位 | 0 |

E1/E2 是[走廊架构](corridor_constraints.md)的直接验收：任何一次违反都说明
约束层没接全，**优先级高于效率指标**。

---

## F. 资源与健康

| # | 指标 | 来源 | v1 阈值 |
|---|---|---|---|
| F1 | 仿真 RTF | 采样前后各读一次 `/world/*/stats` | ≥ 0.9（低于则本轮时序数据标 `SLOW_SIM`，B 组不进对比） |
| F2 | 规划延迟分布 | plan_sec per segment（同 B5） | 信息型（G6 降面前后对比用） |
| F3 | 感知链频率 | preprocessed points Hz | ≥ 3 Hz |
| F4 | 驱动 RSS | 驱动进程内存 | suite 结束 < 1.5 GB |
| F5 | 残留进程 | 结束后 process count | 0（todo4 教训） |

---

## 当前评测缺口

**没有完整的在线「塞到满」评测。** 现有能力停在：

| 已有 | 能证明什么 | 不能证明什么 |
|---|---|---|
| `place_smoke_driver` N=3，固定槽 `[1.5,0,0.655]` | 空箱能进、能放 1 只 carryon | 多箱、下一槽、容量 |
| `packing_replay_eval.py` | 离线求解器在序列上能排多少只 | 手臂进得去、走廊、真空、gz 接触 |
| Humble `ComputePlacement` / cargo_map 节点 | 消息已有 | **未接 launch，切片 B/C 未交付** |
| `pack_eval_driver.py` | 切片 D 计划中 | **文件不存在** |

因此「container 里最多能放几只」目前没有仿真闭环数字。容量数字只能来自切片 D 的 suite：循环 spawn→pick→ComputePlacement→place→commit，直到下面的停条件。

---

## 停条件（无法再放置）

判定「这只箱子放不进去」和「这轮装箱结束」分开。只有前者的直方图才能解释 A5。

### 单箱：这只放不进

按顺序尝试，任一命中则该箱失败（不 commit）：

1. **感知/抓取失败**（`DETECT_*` / `VACUUM_*` / `PLAN_pick_*`）— 不是满箱。
2. **`ComputePlacement.success=false`** — 当前占用下，该尺寸+yaw 集合没有任何可行槽。这是容量判定的核心。`message` 必须能映射到 reason（`no_candidate` / `corridor_blocked` / `outside_aperture` / `unknown_above_floor` / `insufficient_clearance` / `overlap` / `atlas_unreachable`）。
3. **候选槽运动全失败** — 服务给了 `top_n` 槽，笛卡尔/IK 全部 `fraction<0.95` 或碰撞。记 `PLACE_CANDIDATE_EXHAUSTED`，**不是** BIN_FULL（求解器认为有槽，执行进不去）。dump 里保留每一个试过的槽。
4. **place 段失败且 `--on-place-fail=stop`** — `PLACE_*`；可选切换下一候选。不 unmark，因为没 commit。

### suite：这一轮结束

任一即停，写入 `suite.json.termination_reason`：

| `termination_reason` | 何时 | 是否算「装满」 |
|---|---|---|
| `BIN_FULL` | 当前箱 `ComputePlacement` 失败 | **是**（对该尺寸/序列而言） |
| `PLACE_CANDIDATE_EXHAUSTED` | 有槽但所有候选运动失败 | 否（规划/走廊执行问题） |
| `SPAWN_EXHAUSTED` | `SpawnNextBox` 失败（序列用尽） | 否（箱源不够，容量未测到头） |
| `MAX_BOXES` | 达到 `--max-boxes`（安全帽，须 > 几何上限） | 否（帽太小） |
| `ABORT` | 连续 3 箱同层失败（感知或真空，非 placement） | 否 |
| `TIMEOUT` | suite 超时 | 否 |

`GOTO_FAILED`（回 observe）**不停 suite**、不改 `termination_reason`。若该箱已 COMMITTED，仍计入 A3 / ledger。

### 「最多能放多少只」怎么跑

`BIN_FULL` 的含义是 **当前这一只 SKU 在当前堆型下无槽**，不是「任何更小的箱子也放不进」。

- **均质容量**（回答「最多几只 carryon / standard / large」）：`sequence_ids` 单一型号，`--max-boxes` 大于几何上限（carryon 建议 50，standard 40，large 20）。停在 `BIN_FULL`。A3 即容量。
- **混合序列**（回答「这条到达顺序能装几只」）：默认 `skip_unplaceable:=false`，第一只无槽就 `BIN_FULL` 停（贴近上线：来了一只 large 塞不进就停）。可选 `skip_unplaceable:=true`：无槽则记 `SKIP_NO_SLOT`、清当前箱、继续下一只，直到连续跳过或 spawn 尽。混合跳过模式的 A3 **不能**当成单一 SKU 容量。

求解器失败时必须把 **全部被否候选** 写入该箱 dump（A5），否则无法区分真满和约束过死。

---

## 落盘契约

根目录：`docs/status/evidence/packing_eval_<name>/`。

原则：只读 dumps + ledger 就能回答「第 k 只什么尺寸、第几个放、计划槽在哪、实际落到哪、为什么停」。体积大的栅格/点云进 `dumps/box_XX_*/`，ledger 只留指针。

```
packing_eval_<name>/
  suite.json                 # A/C/E/F + termination + 容量结论
  sequence.json              # 计划箱源（跑之前就写死，可复现）
  ledger.jsonl               # 每只箱子一行（成功、失败、跳过都有）
  trials.jsonl               # 与 ledger 对齐的指标宽表（B/D）
  rtf.jsonl                  # 每箱起止 RTF（F1）
  dumps/
    box_00_ok/
    box_01_PLACE_FRACTION_traverse/
    box_07_BIN_FULL/         # 停箱那一次，无 commit
  packing_eval_<name>.md     # 报表（驱动收尾写）
```

### `sequence.json`（箱源，跑前写）

```json
{
  "sequence_ids": ["carryon"],
  "visual_kind": "mesh",
  "size_mode": "catalog",
  "max_boxes": 50,
  "skip_unplaceable": false,
  "seed": 0,
  "planned": [
    {"seq": 0, "catalog_id": "carryon", "size_wdh": [0.55, 0.40, 0.25], "mass_kg": 8.0}
  ]
}
```

均质容量 suite 的 `planned` 可只写模板 + `max_boxes`，不必预生成 50 行。

### `ledger.jsonl`（回溯主账本，每箱一行）

成功、失败、`SKIP_NO_SLOT`、最后一次 `BIN_FULL` 都写。这是「大小 / 顺序 / 点位」的唯一索引。

| 字段 | 说明 |
|---|---|
| `seq` | 本轮尝试序号，从 0 连号（含失败） |
| `commit_index` | 成功 commit 后的 0..n-1；失败为 `null` |
| `spawn_id` / `gz_model` | spawner 模型名，如 `pickup_box_0018_carryon` |
| `catalog_id` | `carryon` / `standard` / `large` |
| `size_wdh` | `[width, depth, height]` 米，catalog GT |
| `mass_kg` | catalog |
| `yaw_requested` | 允许集合（如 `[0, 1.57]`） |
| `yaw_selected` | 选中槽的箱体 yaw（world） |
| `slot_rank` | 在 `ComputePlacement` 可行列表中的名次，0=best |
| `slot_id` | `placed_{layer}_{row}_{col}` 或生成 id |
| `pose_planned_world` | `{position:[x,y,z], rpy:[r,p,y]}` 槽中心，**world** |
| `pose_planned_base` | 同内容，`elfin_base_link` |
| `pose_gz_release` | 松手瞬间 `ign model --pose`（静止后查） |
| `pose_gz_settled` | drift-wait 后同一格式 |
| `err_xy` / `err_z` / `drift` / `roll` / `pitch` | D3–D5 |
| `inside_inner_box` | 内腔判定 |
| `committed` | bool |
| `fail_code` | 空字符串=成功；`BIN_FULL` / `SKIP_NO_SLOT` / `PLACE_*` / … |
| `dump` | 相对路径 `dumps/box_00_ok` |
| `t_ros_spawn` / `t_ros_commit` / `cycle_wall_sec` / `cycle_sim_sec` | B1 |
| `volume_m3` | `W*D*H`，A1 分子逐项 |

`BIN_FULL` 那一行：`committed=false`，pose 全 `null`，`dump` 指向否决快照。

### `suite.json`（一轮结论）

除 A/B/C/E/F 汇总外固定这些字段：

```json
{
  "termination_reason": "BIN_FULL",
  "capacity_claim_valid": true,
  "boxes_packed": 12,
  "boxes_attempted": 13,
  "volume_fraction": 0.15,
  "floor_coverage": 0.82,
  "catalog_counts": {"carryon": 12},
  "last_rejected": {
    "seq": 12,
    "catalog_id": "carryon",
    "size_wdh": [0.55, 0.40, 0.25],
    "n_candidates_total": 40,
    "n_feasible": 0,
    "reject_histogram": {"corridor_blocked": 18, "outside_aperture": 6, "overlap": 16}
  },
  "rtf_min": 0.92,
  "slow_sim": false,
  "map_mismatch": false
}
```

`capacity_claim_valid`：仅当 `termination_reason=BIN_FULL` 且 `skip_unplaceable=false` 且均质目录（或调用方显式声明）时为 true。混合跳过或 `MAX_BOXES` / `SPAWN_EXHAUSTED` 时为 false，避免把「源用尽」写成容量。

### 每箱 `dumps/box_XX_<slug>/`

`<slug>` 与 pick/place 相同：`ok` 或 `fail_code`。失败箱同样建目录。

**必写（回溯最小集）**

| 文件 | 内容 |
|---|---|
| `trial.json` | 该箱 B/C/D 指标 + `fail_code` + 指向 ledger 的 `seq` |
| `box.json` | catalog / size / mass / spawn_id / detection 摘要 / GT AABB |
| `compute_placement.json` | 请求尺寸、返回 `success/message`、**全部候选**（可行+被否）：center_local/world、yaw、score、reason、corridor 高度 |
| `slot.json` | 实际执行的那一个槽（失败尝试则列出 `tried_slots[]`） |
| `commit.json` | `AddPlacedBox` 成败、`placed_*` id、commit 前后 `map_revision`；未 commit 则 `committed:false` |
| `segments.jsonl` | pick 四段 + place 段：name/ok/plan_sec/fraction/used_ompl_fallback/settle_json/suction_before_after |
| `corridor_audit.json` | 该槽走廊 AABB、沿程 `surface_max`、要求高度、extract 采样高度、E1 是否违反 |
| `state_timeline.jsonl` | 状态机 |
| `vacuum_events.jsonl` | attach/detach + gate 数值（E3） |

**成功落盘另写（切片 A 已有同类）**

| 文件 | 内容 |
|---|---|
| `place_retreat_color.png` | retreat 后、回 observe 前手腕图 |
| `occupancy_gt.png` + `occupancy_gt.json` | commit 后几何 2.5D（空腔 free + 已放 occupied） |
| `gt_interior_box.ply` | 内壁 + **本轮全部已 commit 箱** 表面（不是只含当前一只） |
| `surface_2d.json` | `ComputePlacement` **调用时** 的 mapper 快照（决策用图，不是 commit 后） |

**失败 / BIN_FULL 另写**

| 文件 | 内容 |
|---|---|
| `reject_histogram.json` | reason→count，供 A5 |
| `surface_2d.json` | 当时的图，解释「为什么无槽」 |
| `occupancy_gt.png` | 停箱时堆型（已 commit 的箱子） |

不要把完整 `cloud_xyz` 塞进 `trial.json`。运动中禁止 `ign model --pose`；pose 只在松手后静止采样。

### `trials.jsonl`

每箱一行，字段为 ledger 的指标投影（B3–B7、D1–D6、`fail_code`、`dump`），给报表用。点位仍以 ledger 为准，避免宽表和 JSON 嵌套各写一套。

## 实现挂点（改动最小路径）

| 指标 | 挂哪 |
|---|---|
| A1/A2/A3/A5 | suite 收尾：驱动从 commit 账本 + ComputePlacement 最后一次调用结果 |
| A4 | GetCargoMapStats（切片 B 交付的服务） |
| B1/B2 | 驱动时间戳（spawn/COMMITTED 两点） |
| B3–B6 | PlanMotion.Result（已有 plan_sec/fraction）+ 驱动 wallclock |
| C 全部 | eval_metrics.summarize 扩展（分母规则已在） |
| D1 | DetectionAccuracy（已有） |
| D2–D6 | 驱动 gz pose 采样（place_smoke_driver 已有 box_gz_before/after 模式） |
| E1/E2 | 驱动在 retreat/extract 前后采样轨迹高度 vs corridor_surface_max；segments 的 used_ompl_fallback |
| E3 | vacuum_events（gate 判据值已随事件落盘） |
| F1/F5 | 驱动起止钩子（pick_retreat_eval_driver 已有 RTF 采样先例） |

**优先级**：A1/A5/B1/C5 是切片 D 验收的最小集；E1/E2 随走廊高度（G1）
落地时同批；其余随驱动逐步补齐。v1 阈值在第一轮 N≥10 suite 后按 P95 回填。
