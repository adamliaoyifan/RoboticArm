# PF-R6 generation 3 — 30-trial Gate 4 accuracy matrix (zmode detection-path regression)

- 运行环境：隔离干净 git worktree `/tmp/pfr6_gen3_clean` @ **f03ccc3**（dirty=0）
- 口径：`docs/plans/platform_free_height_gate4_revision.md`（与 PF-R5 run8 同一判定文档）
- 目的：gen2 接管线程验收项 — zmode_median 改变了在线检测路径，关闭 PF-R6 前必须重跑原始 PF-R5 30-trial Gate 4 精度矩阵
- 机器：AMD Ryzen 9 9950X3D, RTX 5090, Linux 6.8.0-138, ROS Humble, `ROS_DOMAIN_ID=7`
- 启动参数：见 summary.json `launch_params`（accepted profile，与 run8 相同）
- 判定：**全部门通过（gate4_pass=true, failures=[]，coverage_failures=[]）**

## 命令

```bash
git worktree add /tmp/pfr6_gen3_clean f03ccc3
cd /tmp/pfr6_gen3_clean && source /opt/ros/humble/setup.bash && colcon build \
  --packages-select elfin_control elfin_description elfin_moveit_config \
  luggage_msgs luggage_description luggage_perception luggage_planning \
  luggage_packing luggage_gazebo luggage_bringup --symlink-install
cp <main_ws>/yolov8s-world.pt .   # launch cwd 权重
cp -r <main_ws>/install/luggage_perception/share/luggage_perception/vendor \
  install/luggage_perception/share/luggage_perception/   # CLIP vendor
source install/setup.bash && export ROS_DOMAIN_ID=7
ros2 launch luggage_gazebo sim_world.launch.py gui:=false use_rviz:=false \
  use_semantic:=true use_motion:=true use_vacuum:=true visual_kind:=mesh \
  size_mode:=catalog sequence_ids:=carryon,standard,large \
  xy_jitter_range:=0.12,0.12 yaw_range:=-0.6,0.6 \
  observe_pose_name:=pickup_observe &
echo $! > /tmp/elfin_humble_sim.pid
python3 scripts/platform_free_height_gate4_eval.py \
  --out <this_dir> --trials 30 --settle-sec 4.0 --launch-params "<同上 + worktree 记录>"
scripts/stop_sim.sh   # 清场
```

## 结果（30-trial，settled 291 帧）

| 门 | 限值 | 实测 | 判定 |
|---|---|---|---|
| top_surface_rate | ≥0.95 | **0.9725** | ✓ |
| full3d_rate | ≥0.95 | **0.9823** | ✓ |
| top Z p95/max | 15/25 mm | 10.70/10.72 | ✓ |
| support Z p95/max | 15/25 mm | **0.00066/0.00074**（亚微米，zmode） | ✓ |
| height p95/max | 25/40 mm | 10.70/10.72 | ✓ |
| XY p95 | 30 mm | 7.85 | ✓ |
| width/depth p95 | 50 mm | 35.6/43.9 | ✓ |
| false measured height | 0 | 0 | ✓ |
| coverage | 3 尺寸×10 | 3×10 / 30 trials / 26 yaw | ✓ |
| stale instance / spawn fail | 0 | 0/0 | ✓ |
| active Hz（诊断，G6 另行） | — | 3.764 | —（PF-R10 整链门） |

## 与 run8（PF-R5 官方基线）对比

| 指标 | run8 @ c5921d5 | 本运行 @ f03ccc3 | 变化 |
|---|---|---|---|
| top_surface_rate | 0.9606 | 0.9725 | +1.2 pp |
| full3d_rate | 0.959 | 0.982 | +2.3 pp |
| support Z p95/max | 0.0/0.0 mm | ~0.0007/0.0007 mm | 同级（亚微米级） |
| height p95/max | 10.7/10.8 mm | 10.70/10.72 mm | 同级 |
| active_output_hz（诊断） | 3.91 | 3.764 | 上游 preprocessor 吞吐所致（见下） |

结论：zmode_median 替换支撑面 RANSAC 后，全部精度/契约门通过且无回退；
top/full3d rate 较 run8 提升。`active_output_hz` 为诊断项，受
`sensor_preprocessor` 3.63 Hz 发送上限约束（gen3 stage_probe 实测），
按已共识计划 `docs/plans/pf_r8_r9_perception_acceptance.md` @3460bff
分柱归属 PF-R9（吞吐）/ PF-R10（整链 PF-G6S）。

## 清场

- `scripts/stop_sim.sh` 执行后残留进程计数 0
  （`ps -eo comm= | grep -cE "^(ign|gz|ruby|ros2|parameter_bridge)"`）。
