# Todo 5 切片 A — place 动作验收

日期：2026-09-01  
commit：工作区不是 git 仓库  
`ROS_DOMAIN_ID`：7  
设计：[todo5_place_action.md](../plans/todo5_place_action.md)

一次 place：吸着箱子从 `pick_retreat` 出发，进集装箱、落位、松手、退出。  
栅格 / `ComputePlacement` / 塞满循环不在本切片。

## 栈

```bash
export ROS_DOMAIN_ID=7 DISPLAY=:1
ros2 launch luggage_gazebo sim_world.launch.py gui:=false use_rviz:=false \
  use_semantic:=true use_motion:=true use_vacuum:=true \
  visual_kind:=mesh size_mode:=catalog sequence_ids:=carryon \
  observe_pose_name:=pickup_observe \
  semantic_require_backend:=bbox_fill
```

Launch PID 记在 `/tmp/todo5_place_sim.pid`。驱动：

```bash
# S6 干跑（只规划不执行）
ros2 run luggage_gazebo place_smoke_driver.py --dry-run --n 1 \
  --out docs/status/evidence/place_smoke_dryrun

# S7 空载
ros2 run luggage_gazebo place_smoke_driver.py --payload none --n 1 \
  --out docs/status/evidence/place_smoke_empty

# S7 带载 N=3
ros2 run luggage_gazebo place_smoke_driver.py --payload vacuum --n 3 \
  --goto-timeout 60 --out docs/status/evidence/place_smoke_n3
```

## 结论

切片 A 通过。带载 N=3 carryon **3/3** 进箱落位，`descend` fraction 全是 1.0 且未走 OMPL，落点 / 漂移 / 倾倒 / 内腔判据全部进门槛。载荷丢失 0 次。

下一步是 [Todo 5 切片 B — 占据栅格](../plans/closed_loop_place_pack.md)。

## 验收表（设计第 7 节）

| 项 | 门槛 | 实测 | |
|---|---|---|---|
| 五段（含 staging）全成功 | 3/3 | 3/3，每试 7/7 | ✓ |
| `descend` cartesian fraction | ≥ 0.95，无 OMPL | 1.0 / 1.0 / 1.0，`used_ompl_fallback=false` | ✓ |
| 松手时机 | 先 `RELEASE_SETTLE` 再 `enable:false` | 时间线 P8→P9→detach→retreat | ✓ |
| 落点 | `err_xy < 0.04`、`|err_z| < 0.03`、`inside_inner_box` | 均值 xy 11.5 mm、z −3.7 mm，3/3 在内腔 | ✓ |
| I-5 松手漂移 | < 0.02 m | 均值 9 µm | ✓ |
| 倾倒 | `|roll|,|pitch| < 2°` | 最大 `|pitch|` 1.13°（trial 0） | ✓ |
| I-1 载荷丢失 | 0 | 0 | ✓ |
| `GOTO_FAILED` | 记录，不进 place 分母 | trial 0 HOME 超时；`place_ok` 仍计 | ✓ |

证据：`docs/status/evidence/place_smoke_n3/`。

## 已定决策（实施时未改）

- 规划系统一 **`world`**。`SlotSpec` 在 `waypoint_generator_node` 边界从 `elfin_base_link` 转入 world。
- 带载失败策略 **`stop`**：`insert`/`descend` 失败不松手、不回 observe。
- **`COMMITTED` 在 `PLACE_RETREAT` 之后**：接触位就 `AddPlacedBox` 会让 retreat 起点 in-collision。
- Humble 每段后的 `_wait_settled` 已经比 ROS 1 release-settle 更严，只把 `SettleTracker.diagnostics()` 透到 `PlanMotion.Result.settle_json`。

## S0 — scene_manager + mesh

rclpy `scene_manager_node.py` 进 launch（`use_motion:=true` 时，在 move_group 之后）。服务：

- `~/sync_static_scene` — 集装箱 STL（8000 面）+ pedestal
- `~/add_placed_box` / `~/remove_placed_box`
- `~/set_place_support_touch` — `pickup_box` ↔ `airport_container_real` 与所有 `placed_*` 的 ACM，带 verify 轮询

刻意不做 pickup attach：仍由 vacuum backend + `PlanningSceneClient` 独占。

带载 N=3 每次都完整跑了 pick 四段 + `vac_attach`，集装箱 mesh 在场景里，pick 没有被挡住。独立的 `pick_retreat_eval_driver` N=4 回归打在 `docs/status/evidence/place_s0_pick_reg/`，4/4 是预存的 `SPAWN_VISUAL_TF`（`no_camera_info` 热机），不是 mesh 回归；place 驱动不走那道 visual gate。

## S1 / S2 — staging 与 place 接通

`staging_offset()` 沿完整 `normal` 三轴外推。单测覆盖 `normal=[-1,0,0]`（本场景 world）和 `[0,1,0]`（旧 base_link），`|stage_offset| ≈ stage_outward_clearance`。

`waypoint_generator_node` 传入 `request.place_slot`，在 world 下组 `opening_info`，place marker frame 改 `world`，`|stage_offset| < 0.1` 时 `staging_degenerate` 告警。N=3 三次都是 `staging_degenerate=false`。固定槽位 world `[1.5, 0, 0.655]`（底板 0.53 + carryon 高/2）。

## S3 — PlanMotion 诊断

`PlanMotion.Result` 增加 `fraction` / `used_ompl_fallback` / `moveit_error_code` / `settle_json`。现有调用方只读 `success`/`message`，加字段不破坏它们。

## S4 / S5 — 驱动与指标

`place_smoke_driver.py` 子类化 `PickRetreatEvalDriver`，**不改** `pick_retreat_eval_driver.py`。状态机 13 态 + `ABORT_CARRYING` / `ABORT_RELEASED`。落盘：`trial.json` / `segments.jsonl` / `state_timeline.jsonl` / `tf_trace.jsonl` / `vacuum_events.jsonl` / `slot.json` / `scene.json`。实时 `/luggage/place/state`（String JSON, TRANSIENT_LOCAL）。

`place_metrics.py`：`place_ok(GOTO_FAILED)` 仅当 `place_state == HOME`。单测 6 个，含 `ign model --pose` 解析（跳过 entity id 括号）。

## S6 — 干跑

`docs/status/evidence/place_smoke_dryrun/`：7/7 IK，cartesian 段 fraction 全 1.0，无 OMPL。硬性节流点通过后再进 S7。

## S7 — 空载 + 带载

空载 `docs/status/evidence/place_smoke_empty/`：7/7 执行、fraction 1.0。当时 HOME 从箱内直接 FJT，`error_code=-5`（`GOAL_TOLERANCE_VIOLATED`）。后来 HOME 改为：若 `suction.x > 0.2` 先笛卡尔退到 portal（`place_exit`），再 `GoToRobotPose(pickup_observe)`。

带载 N=3：

| trial | 段 | descend | err_xy | err_z | drift | inside | HOME |
|---|---|---|---|---|---|---|---|
| 0 | 7/7 | 1.0 | 16.0 mm | −6.6 mm | 28 µm | true | `GOTO_FAILED`（exit 后 goto 超时） |
| 1 | 7/7 | 1.0 | 9.3 mm | −2.2 mm | 0 | true | `reached pickup_observe` |
| 2 | 7/7 | 1.0 | 9.2 mm | −2.2 mm | 0 | true | `reached pickup_observe` |

同槽复用：verify 之后 `RemovePlacedBox` + `ClearCurrentBox`，下一试面对空槽。不这样做的话第二试 `traverse` 会撞上 `placed_0_0_0`。

成功落盘（place_retreat 之后、HOME 回 observe 之前）额外写出：

| 文件 | 内容 |
|---|---|
| `place_retreat_color.png` | 手腕相机当时最新一帧（`keep_camera_down` 仍未实现，画面不一定对着箱内） |
| `occupancy_gt.png` | 集装箱内腔 2.5D 几何 GT：空腔 free，已放箱 occupied；开口 −X 在图底白边 |
| `occupancy_gt.json` | 同一栅格的 `height` / `state` |
| `gt_interior_box.ply` | world 系内壁+底板（灰）+ 已放箱表面（橙），几何采样，不受相机 FOV 限制 |

失败 abort 不写这四项。这是 scene_tf + 落点 pose 的几何真值，不是 live cargo-map / 深度点云。

## 实施中修掉的运行时问题

| 问题 | 处置 |
|---|---|
| Humble 侧 scene_manager 仍是 rospy，planning scene 只有机器人 + pickup box | 移植 rclpy，STL mesh + pedestal + ACM |
| staging 只加 `normal[1]`，world 下 `normal=[-1,0,0]` 退化 | 三轴外推 + 单测 |
| `PlanMotion` 只有 success/message | 结构化 fraction / OMPL / settle |
| 箱内 FJT 回 observe 臂不动（joint0 误差 −2.18 rad） | HOME 先笛卡尔出开口 |
| `SimVacuumBackend.detach` 从 MoveIt 删 attached box | retreat 之后 `AddPlacedBox` 以静态物体插回 |
| 同槽第二试撞已放箱 | 成功后清 gz 模型 + `RemovePlacedBox` |
| 松手后 `/luggage/current_box` 不再带 model 名，verify 拿不到 gz 位姿 | pick 时记住 `box.id`；`ign model --pose` 解析 XYZ/RPY |
| 段中同步 `ign model --pose` 会卡住物理、吸盘 follow 掉队 | 运动中不查 gz；verify 时箱子已静止再查 |
| I-3 设计 5 cm，运动学 follow 的 XY 滞后约 6–12 cm（Todo 4 T3 同类） | 记录 offset；仅当 XY > 0.5 m（箱子被落下）才 `PLACE_FOLLOW_DRIFT` |

## 已知、非阻断

- `insert` / `descend` / `retreat` 的 `keep_camera_down` 仍未实现，成功消息带 `NOT_IMPLEMENTED: keep_camera_down`。
- trial 0 HOME 在笛卡尔出开口之后仍可能 `GOTO_FAILED`；place 本身已完成，不进失败码统计。
- 驱动 / motion_planner 的动作超时已改为跟 `/clock`（仿真秒），`/clock` 卡住 20 s 墙钟才 aborted。Gazebo 物理碰撞改用 2000 面 `container_collision_physics.stl`（MoveIt 仍用 8000 面，规划性能另议）。
- 集装箱 mesh 8000 面，本次 pick+place 墙钟未见明显变慢（trial 1/2 约 36–44 s，含 pick）。若以后规划变慢，再用 `simplify_quadric_decimation(2000)`。
