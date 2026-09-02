# Todo 5 切片 B–D 验收指南（P1–P5）

日期：2026-09-02。对照 [packing_eval_metrics.md](packing_eval_metrics.md)
落盘契约与 [corridor_constraints.md](corridor_constraints.md)。

P1–P5 代码已接线。**塞到满（B4）在修驱动之前不能当真跑容量**：旧版
`pack_eval_driver` 会二次 spawn、仍用 `fixed_floor_center`、成功后
`ClearCurrentBox` 删掉 gz 模型。当前驱动已改为：

- 只 spawn 一次，place 用 `ComputePlacement` 的槽（`elfin_base_link`）
- 成功后 `FinalizeCurrentBox`（箱子留在集装箱里）+ `cargo_map/add_placed_box`
- 每箱 `dumps/box_XX_<slug>/` 写入 placement / slot / corridor / commit 快照

仿真全流程仍需你在本机 launch 后跑；下面 B1–B3 可先无臂或纯 Python 验。

## 交付物索引

| 任务 | 文件 | 验什么 |
|---|---|---|
| P1 走廊高度 G1 | `waypoint_generator.py` + `waypoint_generator_node` 订 `committed` | 高邻箱抬 traverse/retreat |
| P2 占据栅格 | `cargo_volume_mapper_node.py` + `use_cargo_map` | Add/Remove/Reset/GetStats |
| P3 槽位求解 | `placement_planner_node.py` + `use_packing` | floor-prior z、aperture、corridor、BIN_FULL 直方图 |
| P4 走廊审计 | `corridor_audit.py` | E1 verdict |
| P5 塞满驱动 | `pack_eval_driver.py` | ledger / suite / dumps |

坐标系（验收时不要混）：

| 接口 | `place_pose` 坐标系 |
|---|---|
| `/placement_planner/compute_placement` 返回槽 | **elfin_base_link**（与 place_smoke / waypoint 一致） |
| `/cargo_map/add_placed_box` | **world**（节点内部转到 container_link） |
| `/placement_planner/last_result` JSON | `center_base` = container_link；另有 `center_world` / `center_base_link` |

空箱地板槽：container_link / world **z ≈ 0.53 + h/2**（carryon → **0.655**）。
不要拿 SlotSpec 的 base_link z 去对 0.655。看 `last_result.selected.center_world[2]`。

---

## A. 启动（B1/B2 最少集 / B4 全套）

B1/B2 只要 mapper + planner（可挂在已有 sim 上，或单独起这两个节点）。B4/B5 需要全套：

```bash
cd ~/work/elfin_humble_ws && source install/setup.bash
export ROS_DOMAIN_ID=7 DISPLAY=:1

ros2 launch luggage_gazebo sim_world.launch.py gui:=false use_rviz:=false \
  use_semantic:=true use_motion:=true use_vacuum:=true \
  use_cargo_map:=true use_packing:=true \
  visual_kind:=mesh size_mode:=catalog \
  sequence_ids:=carryon \
  observe_pose_name:=pickup_observe \
  semantic_require_backend:=bbox_fill
```

就绪标志：

- `cargo_volume_mapper ready (inner 1.49x1.97x1.48 m, res 0.05)`
- `placement_planner ready (aperture_y=..., smallest=[0.55, 0.4, 0.25])`
- B4 另需：`motion_planner ready`、`vacuum_controller ready`

先编译本次改动：

```bash
colcon build --packages-select luggage_perception luggage_packing luggage_planning luggage_gazebo --symlink-install
source install/setup.bash
```

---

## B1. P2 占据栅格

`AddPlacedBox` 的 pose 是 **world**。container_link 在 world `(1.5, 0, 0)`，
箱中心 world x=1.0 → 局部 x=-0.5，落在内腔。

```bash
ros2 service call /cargo_map/get_stats luggage_msgs/srv/GetCargoMapStats

ros2 service call /cargo_map/add_placed_box luggage_msgs/srv/AddPlacedBox \
  "{slot: {place_pose: {position: {x: 1.0, y: 0.0, z: 0.655}}, width: 0.55, depth: 0.40, height: 0.25}}"

ros2 service call /cargo_map/get_stats luggage_msgs/srv/GetCargoMapStats
```

门槛：

- add 前 `occupancy_ratio=0`，`message` 含 `committed=0`
- add 后 `occupied_count>0`、`committed=1`
- 同参 `remove_placed_box` → `committed=0`
- `reset` → 全清

`surface_2d` 是 transient-local，先订阅再触发：

```bash
ros2 topic echo /luggage/cargo_map/surface_2d --once | head -c 200; echo
```

---

## B2. P3 槽位求解

空图 / floor-prior：

```bash
ros2 service call /placement_planner/compute_placement luggage_msgs/srv/ComputePlacement \
  "{box: {width: 0.55, depth: 0.40, height: 0.25}}"

ros2 topic echo /placement_planner/last_result --once
```

门槛：

- `success=true`
- `last_result.selected.center_world[2]` ≈ **0.655**（误差 ≤ 5 cm，体素量化）
- `last_result.floor_prior=true` 若 mapper 还没发图；mapper 已发空图时为 false，z 仍应是 0.655
- `reject_histogram` 可非空（开口外 / 重叠被否的候选），但 `n_feasible>0`

走廊墙（`placed[]` 与返回槽一样，都是 **elfin_base_link**）。先对空图拿一个槽，
再在开口与深处之间塞一只铺满内腔 Y 的墙（container_link：y 跨 ±0.985，x 靠开口，
z=floor+h/2），转到 base_link 后填 `placed`。手填容易错，用：

```bash
python3 - <<'PY'
from luggage_description.scene_tf_config_utils import (
    load_scene_tf_config, resolve_scene_tf_config_path, _local_point_to_base_link)
cfg = load_scene_tf_config(resolve_scene_tf_config_path())
# 全宽墙中心：container_link x=-0.2（开口在 -X），y=0，z=0.53+0.16
print(_local_point_to_base_link([-0.2, 0.0, 0.69], cfg))
PY
```

把打印的 xyz 填进 `placed[0].place_pose`，`width=0.40, depth=1.97, height=0.32`，
orientation 用 **world yaw=0 转到 base_link**（当前场景四元数约
`z=-0.7071, w=0.7071`）。再 `compute_placement`：`message` 的 rejected JSON
必须出现 **`corridor_blocked`**（开口侧槽仍可能 `success=true`，因为墙挡的是深处）。

开口外：直方图出现 `outside_aperture`。

全否：`success=false`，`message` 前缀 `BIN_FULL no_candidate:`，且
`last_result.reject_histogram` **非空**。

---

## B3. P1 走廊高度（纯 Python，不需 sim）

```bash
python3 - <<'PY'
import sys
sys.path.insert(0, "src/luggage_planning")
from luggage_planning.waypoint_generator import build_sequence
from luggage_planning.pose import Point, Pose, Quaternion

class B:
    pose = Pose(position=Point(x=-1, y=0, z=1.0))
    height = 0.25
class S:
    place_pose = Pose(position=Point(x=1.5, y=0, z=0.655))
    height = 0.25
opening = {
    "point": [0.755, 0, 1.3], "normal": [-1, 0, 0],
    "outward_clearance": 0.15, "min_height_above_opening": 0.35,
}
for label, csm in (("none", None), ("tall", 1.0)):
    segs = build_sequence(B(), S(), "place", corridor_surface_max=csm,
                          opening_info=opening)
    print(label, [(s.name, round(s.target_pose.position.z, 3)) for s in segs])
PY
```

门槛：

- `corridor_surface_max=None`：traverse z = contact_z + 0.15（切片 A）
- `=1.0`：traverse / retreat z ≥ 1.0 + 0.25 + 0.05 = **1.30**
- insert / descend **不变**

在线路径：`waypoint_generator_node` 订 `/luggage/cargo_map/committed`，place
时自动把 `corridor_surface_max` 传给 `build_sequence`。B4 第二箱起日志应出现
`corridor_surface_max=...`（走廊里已有邻居时）。

---

## B4. P5 塞到满

不要一上来 `--max-boxes 50`。先 2 只确认 dumps 和 Finalize（箱子还在 gz 里），再容量跑。

```bash
# 冒烟：2 只 carryon，确认循环 / 落盘 / 不删 gz 模型
ros2 run luggage_gazebo pack_eval_driver.py \
  --sequence-ids carryon --max-boxes 2 --goto-timeout 60 \
  --out docs/status/evidence/packing_eval_carryon_n2

# 均质容量（几何上限以下要让它自己 BIN_FULL；帽须大于上限）
ros2 run luggage_gazebo pack_eval_driver.py \
  --sequence-ids carryon --max-boxes 50 --goto-timeout 60 \
  --out docs/status/evidence/packing_eval_carryon_n50
```

`suite.json` 门槛（均质、不 skip）：

| 字段 | 门槛 |
|---|---|
| `termination_reason` | 容量跑应为 `BIN_FULL`（n=2 冒烟是 `MAX_BOXES`，`capacity_claim_valid=false`） |
| `capacity_claim_valid` | BIN_FULL + 单一 `sequence_ids` + 未 `--skip-unplaceable` |
| `last_rejected.reject_histogram` | BIN_FULL 时 **非空** |
| `boxes_packed` | 冒烟 = 2；容量 carryon 地板层应 ≥ 8 才谈容量，否则先查 PLACE_* |
| `floor_coverage` | 面积比（不是 16% 体积口径） |

`ledger.jsonl`：`seq` 连号；成功行 `committed=true`、`pose_planned_world` /
`pose_planned_base` 都有；`dump` 指向存在的目录。

每箱 dumps（成功）：

```
dumps/box_00_ok/
  trial.json  segments.jsonl  state_timeline.jsonl  vacuum_events.jsonl
  box.json  compute_placement.json  slot.json  commit.json
  corridor_audit.json  reject_histogram.json
  surface_2d.json          # ComputePlacement 当时的图
  place_retreat_color.png occupancy_gt.png occupancy_gt.json gt_interior_box.ply
```

BIN_FULL 箱：`compute_placement.json` + `reject_histogram.json` + `commit.json`
（`committed:false`），无运动段也可。

Ctrl-C：ledger 已逐行 flush；`suite.json` 在 `run()` 正常收尾才写。中断后仍应能
从 `ledger.jsonl` 数已完成箱。

---

## B5. 零回归（不接 packing）

```bash
ros2 run luggage_gazebo place_smoke_driver.py --payload vacuum --n 3 \
  --goto-timeout 60 --out docs/status/evidence/place_smoke_n3_regress
```

3/3、descend fraction=1.0、落点与 9/1 基线一致。不要加 `use_packing`。

---

## 已知限制

1. `corridor_blocked` 仍是**单箱全宽墙**（多箱拼墙后补）
2. `slot_rank` 恒 0（只执行 best；`--on-place-fail=next` 未做）
3. `mass_kg` 来自 current_box JSON；缺字段时为 0，不影响 A1 体积
4. mapper `remove_placed_box` 中心+尺寸匹配（tol 0.05），与 add 不一致会 no match
5. 候选 JSON 在 `last_result` 里可能较大（`keep_rejected=5000`），这是 A5 需要的
6. 在线 G1 依赖 cargo_map commit 先于下一箱 `BuildMotionSequence`；第一箱走廊为空，不抬高度

结果写 `docs/status/packing_eval_<name>.md`：B1–B5 命令摘录、suite 六组表、E1
`verdict` 非 occupied。
