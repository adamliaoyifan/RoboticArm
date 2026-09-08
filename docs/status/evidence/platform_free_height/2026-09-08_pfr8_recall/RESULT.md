# PF-R8 A4 — accepted-detection recall measurement (baseline vs PF-R8)

- 机器：AMD Ryzen 9 9950X3D, RTX 5090, ROS Humble, `ROS_DOMAIN_ID=7`,
  accepted profile（与 gate4 相同 launch 参数）
- 测量定义（恒定，跨 baseline/post 两端相同）：捕获脚本
  `src/luggage_perception/test/pf_r8_recall_capture.py` 以**生产谓词函数**
  对每帧每条 cargo 检测分类（live camera TF @ 消息 stamp + 配置静态工作区
  几何），不依赖被测栈自身的标注；hold 桥接由脚本离线重放修复后的
  `DetectionTemporalGate`（instance epoch 变化即 reset，同在线节点行为）。
- settled 帧定义：instance_id 非空且距 instance 变化 ≥5 帧（对齐 gate4
  WARMUP_FRAMES）。raw recall 分子只计非 held 的 accepted 检测；gate 桥接
  recall 另行给出。

## 命令（两端一致，仅被测栈的 install 不同）

```bash
# baseline 栈：/tmp/pfr6_gen3_clean @ f03ccc3（gen3 提交，PF-R8 之前）
# post 栈：主工作区 install（PF-R8 代码 + confidence_threshold 0.01）
ros2 launch luggage_gazebo sim_world.launch.py gui:=false use_rviz:=false \
  use_semantic:=true use_motion:=true use_vacuum:=true visual_kind:=mesh \
  size_mode:=catalog sequence_ids:=carryon,standard,large \
  xy_jitter_range:=0.12,0.12 yaw_range:=-0.6,0.6 \
  observe_pose_name:=pickup_observe &
echo $! > /tmp/elfin_humble_sim.pid
python3 src/luggage_perception/test/pf_r8_recall_capture.py \
  --out <本目录>/<run> --trials 15 --trial-sec 10
# 复评（离线、确定性）：
python3 src/luggage_perception/test/pf_r8_recall_capture.py --score <run>/raw.jsonl
scripts/stop_sim.sh   # 每次运行后；残留计数 0
```

## 结果

| run | 栈 | settled | recall_raw | recall_gate_bridged | max_miss_run |
|---|---|---|---|---|---|
| baseline_gen3 | f03ccc3（conf 0.04） | 508 | 0.8740 | 0.8740 | 33 |
| post_pfr8_iter1 | PF-R8（conf 0.01） | 489 | **1.000** | **1.000** | **0** |
| post_pfr8_iter2 | PF-R8（conf 0.01，确认运行） | 512 | **1.000** | **1.000** | **0** |
| agreement_probe | PF-R8（生产标注一致率探针） | 107 | 1.000 | 1.000 | 0 |

## 判定（A4）

- **A4-1** max consecutive miss run ≤ 2：iter1 = iter2 = **0** ✓（baseline
  33——两次整 trial 掉检 47/46 帧 ≈ 10 s，任何有界 hold 均无法桥接）
- **A4-2** settled 帧 accepted 召回 ≥ 0.95：iter1 = iter2 = **1.000** ✓
  （baseline 0.874，delta +12.6 pp）
- **A4-3** 场景/机器人误检产生有效几何 = 0：谓词对捕获的全部 319 例右缘
  静态误检实例全部拒绝（`outside_workspace`，逐帧复核 baseline 与 post 两
  端原始数据）；detection_frame 结果仅 `ok` / `DETECT_NO_CLOUD`
  （iter1 428/115，iter2 同类），无任何由误检产生的有效几何；
  false_measured_height 语义由 PF-R10 C1 在 GT 对比下复验。
- **A4-4** baseline 同命令重算于 PF-R6 gen3 提交 `f03ccc3`（上表），delta
  如上；`test_vintage_pose_regression.py` 通过（全量 498 测试内）。

## 掉检根因与修复

baseline 的全部 settled miss 中，93/109 帧属于两次整 trial 掉检
（`pickup_box_0005_standard`、`pickup_box_0011_standard`）：这些帧里 YOLO
只输出右缘静态误检（conf 0.215/0.07 双框，逐帧字节级相同），行李箱得分
低于 0.04 floor——与 PF-R5 记录的对角 yaw/光照状态 0.01-0.03 得分一致。
将 `confidence_threshold` 0.04 → 0.01 后这些状态被重新召回（p25 0.124 /
p50 0.221，最低 0.0131）。低阈值带入的额外检测由 A1 谓词（工作区投影）
拒绝在工作区之外，`false_measured_height` 语义由几何 fail-closed 门与
PF-R10 C1 继续把关。

## 生产标注一致率（agreement_probe）

`production_matched` 845 条 cargo 检测，raw 一致率 0.9858；12 条不一致全部
是捕获脚本自身 TF buffer 未就绪的启动帧（脚本 `predicate_unavailable`
fail-open 判 accepted，生产端有 TF 判 `outside_workspace`）——两侧均有 TF
时一致率 **100%**。启动帧不属于 settled 窗口，不影响 recall 计量。捕获
端 744/716 行中仅此启动期 2 行 fail-open，其余全部给出几何判定。

## 清场

每次运行后 `scripts/stop_sim.sh` 执行，残留进程计数 0
（`ps -eo comm= | grep -cE "^(ign|gz|ruby|ros2|parameter_bridge)"`）。
