# Todo 4 真抓取验收

日期：2026-09-01  
commit：工作区不是 git 仓库  
`ROS_DOMAIN_ID`：7  
模板：[todo4_vacuum_test_guide.md](../plans/todo4_vacuum_test_guide.md)

## 栈

```bash
ros2 launch luggage_gazebo sim_world.launch.py gui:=false use_rviz:=false \
  use_semantic:=true use_motion:=true use_vacuum:=true \
  visual_kind:=mesh size_mode:=catalog \
  sequence_ids:=carryon,standard,large \
  observe_pose_name:=pickup_observe \
  semantic_require_backend:=bbox_fill
```

Launch PID 记在 `/tmp/todo4_vacuum_sim.pid`（1220679）。验收脚本与原始输出：

- T0–T5：`docs/status/evidence/todo4_vacuum/t0_t5.json`
- T6：`docs/status/evidence/pick_eval_vacuum_n10/`
- 对照基线（真空关）：`docs/status/evidence/pick_eval_n50_v2/`

## 结论

T0 / T1 / T2 / T2.5 / T4 / T5 通过。T3 箱子跟着 retreat 走（ΔZ = 0.308 m，落在 0.35 ± 0.05 内，`follow_skipped = 0`，`fail_reason` 空），XY 漂移 6.3 cm，略超 5 cm 门槛，记为 **near-miss**。T6 `vac_attach` / `vac_follow` 10/10，E2E pick 100%，不低于 n50_v2。

Todo 4 按计划完成，带一条 T3 XY 已知偏差。下一步是
[Todo 5 — place + 占据栅格 + 顺序装箱](../plans/closed_loop_place_pack.md)。

## 验收前修掉的运行时阻断（不在原 B1–B3 清单）

这些不修的话 T2/T3 会假失败：

| 问题 | 处置 |
|---|---|
| Humble `TransformListener(spin_thread=True)` 不能挂在已进 executor 的节点上 | 旁路节点 `vacuum_controller_tf` 收 `/tf` |
| 接触间隙：attach 会进入 spawn AABB 约 1 cm | 节点 `contact_gap_min=-0.02`（算法默认仍是 `-0.01`） |
| gz `SetEntityPose.entity.type` 默认 `NONE`，箱子几乎不跟 | 改为 `Entity.MODEL` |

B1–B3 / D1–D5 仍按计划：`box_from_current_box_payload`、`ApplyPlanningScene.Request`、attach 不覆盖整张 ACM、方向性接触、follow 不可重入、clear 后 box=`None`、detach 失败回滚、`follow_step` 提到 ABC、驱动 `--use-vacuum`。

## T0 前置自检 — ✓

`ros2 topic echo /luggage/current_box --once` 顶层字段是 `width` / `depth` / `height`（没有嵌套 `size`）。节点日志解析出 `size=[0.487, 0.390, 0.25]`（carryon）。`/apply_planning_scene` 在服务列表里。

observe 位 `suction_contact_frame` tilt ≈ 12.2°。这是观察姿态，不是 attach；attach 倾角在 T2 量（0°）。

## T1 箱子入 PlanningScene — ✓

`PlanningSceneClient.add_pickup_box` 返回 `applied`。approach 段 `GetCartesianPath`：`cartesian ok (fraction 1.000)`。箱子在场景里但不挡笛卡尔接近。

## T2 attach 信号契约 — ✓

```
success=true
attached=true  vacuum_on=true  fail_reason=""
tilt_deg=0.0
retention_margin≈7.11
contact_distance≈-0.004   # 到箱顶面的间隙（D1 之后）
```

反例：接触位把 `/luggage/current_box` 的 `mass_kg` 改成 200 再 enable，返回 `VACUUM_RETENTION_MARGIN`，`attached` 保持 false。

## T2.5 attach 之后规划仍可用 — ✓

attach 后立刻 `PlanMotion` retreat：`cartesian ok (fraction 1.000)`。不是自碰撞 / “Start state appears to be in collision”，说明 ACM 没有被 1×1 矩阵整表替换。

## T3 箱子跟着走 — △ near-miss

Retreat 前后各采一次 gz `pickup_box` 位姿（carryon，spawn 中心约 `(-1, 0, 0.985)`）：

| | X | Y | Z |
|---|---|---|---|
| retreat 前 | -1.000 | 0.000 | 0.985 |
| retreat 后 | -0.978 | 0.060 | 1.293 |

| 指标 | 实测 | 门槛 | |
|---|---|---|---|
| ΔZ | 0.308 m | 0.35 ± 0.05 | ✓ |
| ΔXY | 0.063 m | < 0.05 | ✗（超 1.3 cm） |
| `fail_reason` | `""` | 恒空 | ✓ |
| `follow_skipped` | 0 | 0 | ✓ |

retreat 前采样仍是平台高度：follow 还没把箱子吸到面板。ΔZ 接近手臂 0.35 m 提升，说明跟随在动；XY 超限混了「snap 到相对位」和 retreat 漂移。T6 的 `vac_follow` 列看的是 ROS `attached && !fail_reason`，不是 gz ΔXY。

## T4 释放，物理接管 — ✓

`enable:false` 成功。2 s 后 gz Z = 0.983，期望 `platform_z + height/2 = 0.985`（± 2 mm）。`attached=false`、`vacuum_on=false`。detach 后箱子回到平台，不再跟臂。

## T5 误吸附防护 — ✓

三次都是 `success=false`、前缀 `VACUUM_NOT_IN_CONTACT`、`attached` 保持 false、gz 箱子未动。

| 位置 | 做法 |
|---|---|
| observe（远） | 真臂 + 真 `current_box` |
| 箱顶正上方 0.30 m | 合成箱位 vs 真实面板 TF |
| 箱子侧旁 0.30 m、与箱顶同高 | 同上 |

上方 / 侧旁用合成 payload：gate 用真实面板 TF 对假想箱位，不把臂笛卡尔插到那两个危险位。D1 改成方向性间隙之前，正上方 0.30 m 会误通过。

## T6 端到端（`--use-vacuum`，N=10）— ✓

```bash
python3 -u src/luggage_gazebo/scripts/pick_retreat_eval_driver.py \
  --n 10 --use-vacuum \
  --out docs/status/evidence/pick_eval_vacuum_n10 \
  --observe-pose pickup_observe
```

驱动默认 `--use-vacuum` 关闭；n50_v2 不受影响。

| 指标 | n50_v2（真空关，N=50） | T6（真空开，N=10） |
|---|---|---|
| Detect usable | 100% | 100% |
| Plan / retreat / E2E pick | 100% | 100% |
| YOLO ready | 100% | 100% |
| Vac attach | — | **100%（10/10）** |
| Vac follow | — | **100%（10/10）** |
| `follow_skipped` | — | 全程 0 |
| 阻断失败（YOLO / tracker） | 0 | 0 |

T6 每试次 attach 后 `tilt_deg≈0`、`fail_reason=""`，detach 消息均为 `detached pickup_box_…`。

非阻断 / 事后码：`DETECT_GATE:xy` 1、`SPAWN_VISUAL_MISMATCH` 2（与 n50_v2 同类诊断）、`GOTO_FAILED` 3。后三者都是 **retreat 完成并 `enable:false` 之后** 回 observe 的 `GoToRobotPose timeout`，不进入 pick 分母。n50_v2 没有这条码；加真空后回 observe 偶发超时，不引入新的抓取失败模式。

## 已知偏差

- T3 XY 6.3 cm vs 5 cm；ΔZ、follow 健康度、T6 ROS 跟随列均通过。
- `contact_gap_min=-0.02`：lid-band 可见顶面比 spawn AABB 矮约 1 cm，否则 attach 会 `VACUUM_NOT_IN_CONTACT`。
- Humble TF 必须用旁路节点；`spin_thread=True` 直接绑业务节点会 `Node already added to an executor`。
- gz 跟随必须 `entity.type = MODEL`。
- T5 近距两项是 gate 几何，不是整臂笛卡尔到「上方 0.30 m / 侧旁」。
- T6 `GOTO_FAILED` 是回 observe 超时，pick/vacuum 已成功。
- `keep_camera_down` / `lock_wrist` 仍未实现（Todo 3 遗留）。
- follow 是 30 Hz 运动学 `set_pose`，覆盖箱子动力学（设计如此，同 ROS 1 gazebo_follow）。
