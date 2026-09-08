# sim_world Launch Profile 验收记录

日期：2026-09-04。`ROS_DOMAIN_ID=7`（Test 9）。指南：
[sim_world_launch_profile_test.md](../plans/sim_world_launch_profile_test.md)。

## 结果汇总

| Test | 内容 | 结果 |
|---|---|---|
| 1 | launch 文件语法编译 | ✅ exit 0 |
| 2 | profile YAML 解析（runtime/features/spawner 分组采样） | ✅ `profile ok` |
| 3 | `--show-args` 参数面（profile_config + 12 个既有参数全在） | ✅ |
| 4 | 无 profile 时回落到原默认（gui=true, use_semantic=false, visual_kind=box, observe） | ✅ `fallback ok` |
| 5 | profile 覆盖默认（7 个 feature/runner/spawner 断言全过） | ✅ `profile override ok` |
| 6 | CLI 覆盖 profile（use_rviz/sequence_ids 覆盖，use_semantic 保持 YAML 值） | ✅ `cli override ok` |
| 7 | 缺失 profile 文件 → RuntimeError("profile_config not found") | ✅ `missing profile rejected` |
| 8 | 未知键 → WARN 可见且不影响其余值 | ✅ `WARN: ... typo_parameter_should_warn` |
| 9 | headless 冒烟（GPU 机器） | ✅（下详） |

**Test 1–8 全过，整体验收标准满足。**

## Test 9 冒烟详情

```bash
ros2 launch luggage_gazebo sim_world.launch.py \
  profile_config:=.../sim_world.profile.yaml gui:=false use_rviz:=false
```

- GPU 硬门通过；Gazebo server 起在 airport_loading.sdf；**RTF = 1.0**
- 控制器 `joint_state_broadcaster` / `elfin_arm_controller` 双 active
- **全部 11 个 profile 控制节点按 YAML feature 标志启动**：
  semantic_segmenter（backend=bbox_fill）、semantic_point_filter、
  sensor_preprocessor、luggage_detector（semantic=True）、scene_manager
  （auto sync_static_scene: 8000 triangles）、waypoint_generator、
  motion_planner、cargo_volume_mapper（inner 1.49×1.97×1.48）、
  placement_planner（aperture_y=(-0.928,0.398)）、vacuum_controller
  （backend=sim）、scene_viz + move_group + 三桥 + spawner(mesh/catalog)
- 探针服务可见：`/luggage_detector/detect_luggage`、
  `/pickup_box_spawner/spawn_next_box`、
  `/placement_planner/compute_placement`、`/vacuum/command`、
  `/cargo_map/add_placed_box`
- Python launch 例外：**0**（唯一 ERROR 行是 move_group 的
  "No 3D sensor plugin(s) for octomap"——已知无害，此前所有栈同款）
- observe_pose_hold 完成后干净退出（spawn_at_observe 生效）
- 栈已关闭，残留进程 0

### 备注

- `ros2 node list` 首次为空是本机 ROS 2 daemon 发现滞后（`ros2 daemon
  stop` 后立即可见全部 28 个节点），非 launch 问题
- `semantic_point_filter` / `vacuum_controller` / `waypoint_generator`
  在 node list 中出现两次：rosout 重复注册 + vacuum TF 旁路节点的已知
  显示现象（vacuum_controller_tf sidecar），非重复进程
- Test 6 证实覆盖优先级 **CLI > profile YAML > 内置默认**，且 CLI 只覆盖
  指定项，其余 YAML 项继续生效
