# DYNAMIC-SUCTION ST-1 — A5 真机录制与回放实验协议

日期：2026-09-16
状态：待排期（ST-1 已以 blocked 挂起，A5 完成并转 pass 后解锁 ST-2）
依据：`docs/plans/dynamic_top_surface_and_suction_patch.md` §A5（plan
revision `5822d66`）；实现与 A0–A4 证据见
`docs/status/evidence/dynamic_suction/2026-09-15_st1_a0a4/`。

---

## 1. 目的

在真机（D555 + 静止机械臂）上录制 **45 例**箱子摆放的 RGB-D 观测，
离线回放验证动态顶面估计器在真实数据上的：

1. 有效率与时延（1.5 s 内出有效顶面）；
2. 时间稳定性（同一静置窗口内逐帧抖动）；
3. 相对人工测量参考的绝对精度；
4. 对 `scene_tf` 取值的独立性（改配置不改变结果）；
5. 失败案例的可回放证据（T2 bundle + 稳定 reason code）。

**全程机器人不运动**（机械臂停在 pickup_observe 位）。

## 2. 实验矩阵（45 例）

3 个目录箱型 × 5 个摆放位置 × 3 个 yaw：

| 维度 | 取值 | 数量 |
|---|---|---|
| 箱型 | carryon 0.55×0.40×0.25 / standard 0.70×0.45×0.28 / large 0.80×0.50×0.32 (m) | 3 |
| 位置 | 平台中心 + 沿世界 X/Y 各 ±0.25 m | 5 |
| yaw | 0° / 45° / 90° | 3 |

位置的世界坐标（pickup 平台中心 `(-1.0, 0)`，平台顶面 z=0.86）：

| 位置代号 | world (x, y) | 说明 |
|---|---|---|
| C | (-1.00, 0.00) | 平台中心 |
| X+ | (-0.75, 0.00) | |
| X- | (-1.25, 0.00) | |
| Y+ | (-1.00, 0.25) | |
| Y- | (-1.00, -0.25) | |

⚠️ **平台悬空问题（需现场决定）**：平台为 1×1 m（x∈[-1.5,-0.5]）。large
箱（0.80 长）在 X± 位置时沿 X 悬出平台边缘约 0.15 m；yaw 90° 时在 Y±
同样悬出约 0.15 m。两种处理任选其一并记录在案：
(a) 允许悬空放置（箱底刚性搭住即可，放置后静置确认不晃动）；
(b) 该类案例位置内收至刚好满沿（记录实际中心坐标作为参考，并在 evidence
中注明偏差）。
默认建议 (a)——A5 的参考是人工测量值，不是名义坐标。

**案例命名**：`a5_<size>_<pos>_<yaw>deg_<seq>`（如 `a5_large_X-_90deg_07`）。
**顺序纪律**：位置/箱型顺序预先排定并交错（沿用 D2 的预声明规则），不得
根据前面案例的结果挑选后面的案例；每个案例只录一次，不重录不挑选。

## 3. 现场准备

1. **箱子**：三个目录箱型各一只。录制前用卷尺/卡尺实测每个箱子的
   长×宽×高（每边测 2 处取均值，精度目标 ≤5 mm），填入附录 A。
   若现场箱与目录尺寸偏差 >20 mm，以实测值作为 GT 尺寸并在 evidence 注明。
2. **位置标记**：按 §2 坐标在平台上贴 5 个中心十字标记（误差 ≤10 mm）。
   建议同时画 ±0.25 m 的方向线便于对准。
3. **yaw 对准**：0°/90° 沿平台边缘；45° 用对角模板或量角器（目标 ≤5°，
   实际摆放角可从录制的 RGB 目测复核并记录）。
4. **参考测量（GT）**：每案例摆放后、录制前测量——
   - 箱中心 XY：从平台标记读偏移（x/y 各 ≤10 mm）；
   - 顶面 Z：`0.86（平台顶实测）+ 该箱实测高度`；平台顶面高度本身也实测
     一次填入附录 A（预算：top-Z P95 ≤15 mm，测量精度 ≤5 mm 即足够）。
   - 填入附录 A 的 CSV 模板，一行一案例。
5. **资源所有权（D8 纪律）**：单 CPS executor、单 D555 owner、单 scene TF
   owner、单 Livox owner；确认没有遗留的录制/评估进程。

## 4. 录制流程（每案例）

前置：机械臂走到 pickup_observe 并完全静止；D555 采集正常。

1. 启动录制（沿用 `docs/status/ros2_bag_site_recording.md` 的
   `record_site.launch.py` 流程，ROS_DOMAIN_ID=7），bag 落在
   `~/work/robotarm_bags/record_a5_<case_id>_<hhmmss>/`（一案例一 bag，
   便于按 case 切分；也可一日志连续录制 + 记录每案例起止时刻表）。
   必录话题（Humble rosbag2 打不开这批 bag，回放一律走
   `luggage_perception/eval/bag_mcap_source.py` 直读）：
   - `/camera/d555/color/image_raw/compressed`
   - `/camera/d555/aligned_depth_to_color/image_raw/compressed`
   - 两路 `camera_info`、`/tf`、`/tf_static`
   - `/luggage/preprocessed/status`（settle/geometry_ok 证据）
2. 按 §2 摆箱（戴手套的手离开画面），静置 ≥1 s 后开始计 **3 s 静置窗口**
   （实际录 5–6 s 余量）。
3. 停止录制；确认 bag 落盘且大小合理（3–6 s ≈ 30–60 MB 量级）。
4. 在附录 A 勾选该案例完成并填写备注（如悬空、遮挡、YOLO 目测置信）。

单案例耗时 ≈ 1.5–2 min；45 例 + 设备起停 ≈ **1.5–2 小时**（含重摆与测量）。

**YOLO 路线（需选其一，录制前定）**：
- 方案 1（推荐）：只录原始流，回放时离线跑 bbox_fill YOLO（复用
  `replay_evaluate` 的 ROS-free 推理路径；无 GPU 站点 CPU 离线可慢不可缺）。
  优点：链路最短、与 A0–A4 同一套估计代码；缺点：回放耗时更长。
- 方案 2：录制时同时起完整硬件感知栈（semantic chain），把
  `YoloDetections` 也录进 bag。优点：回放快；缺点：录制时多一份在线栈
  的资源/稳定性风险。
两方案都不允许在线读取摆放真值（D6：online GT/fixture-pose reads = 0）。

## 5. 离线回放与评分（指标与阈值）

对每个案例：从 bag 中取 3 s 静置窗口，取窗口内**最后 10 个有效观测**聚合
（逐帧跑：YOLO bbox → 深度组件 → 动态顶面，全部走 ST-1 已提交代码，
revision `5cc5511`）。

| # | 指标 | 定义 | 阈值（plan §A5） |
|---|---|---|---|
| 1 | 有效时延 | 自首个被接受的 YOLO instance 到首个有效顶面 | **≥44/45 案例在 1.5 s 内**出有效顶面 |
| 2 | 时间稳定 XY | 窗口内逐帧 center_xy 的离散度 | median/P95 spread ≤ **5/12 mm** |
| 3 | 时间稳定 Z | 窗口内逐帧 top_z 的离散度 | median/P95 spread ≤ **4/8 mm** |
| 4 | 绝对精度 XY | 与人工测量参考比 | P95 ≤ **30 mm** |
| 5 | 绝对精度 Z | 与人工测量参考比 | P95 ≤ **15 mm** |
| 6 | scene_tf 不变性 | 回放时改 pickup_source XY / workspace 半径后重跑 | **结果零变化**（数值 ≤1e-9，reason 一致） |
| 7 | 失败证据 | 每个未达标案例 | T2 bundle 可回放 + reason code 稳定（重复回放一致） |

补充口径：

- spread = 窗口内 10 帧该量的 max−min（或 P95−P50，取更保守者，evidence
  中写明所用口径）；
- yaw 不在 A5 阈值内（真值 45° 目测误差大），但逐案例记录 yaw/yaw_valid
  供诊断；
- 45 例分箱型/位置/yaw 的分组结果全部列入报告（per-position/size/yaw）。

## 6. scene_tf 不变性检查（指标 6 的执行）

同一 bag 各回放 6 遍，仅改 scene_tf 配置：

| 变体 | pickup_source XY | workspace 半径 |
|---|---|---|
| 基准 | (-1, 0)（现场值） | 现场值 |
| V1–V3 | (0,0) / (-1,0) / (5,-3) | 现场值 |
| V4–V5 | 现场值 | 0.1 / 5.0 m |

硬件 crop/predicate 参数保持 false（与 A1 相同）。六次输出必须完全一致。

## 7. 证据要求（debug-evidence 分层）

落盘 `docs/status/evidence/dynamic_suction/<run-id>-a5/`：

- **T0**：manifest（commit、dirty、config SHA-256、bag 清单与哈希、参考
  测量表、命令、起止时间、各指标结果、teardown 状态）；
- **T1**：每案例 JSONL（stamp/frame、组件像素数/coverage、平面候选与得分、
  逐帧顶面结果、时延、spread）；
- **T2**：仅失败案例冻结（depth/bbox/组件掩码/平面候选 NPZ + JSON，
  ≤40 MiB，`--replay --stage {ingest,segmentation,cargo,top}` 可复算一致）；
- 参考测量（GT）单独存放，绝不能进在线/回放输入路径。

失败处置：先保证据（不拆台不盲重跑），按 reason code 定位；修复属代码变更
则按 plan 规则评估是否需要新一代合成矩阵回归。

## 8. 通过判据与收尾

A5 通过 ⇔ §5 表格 1–7 全部满足（在 ST-1 干净 revision 上）。之后：

1. 在 ST-1 线程追加 A5 通过的 Result 修订（blocked → pass），删 OPEN.md 行；
2. ST-2（ suction-patch B0–B7）解除依赖门控，可开工；
3. 角色笔记 + 本协议状态更新为 done。

## 9. 前置软件工作（录制前后均可做，回放前必须完成）

A5 回放 stage 目前在 `dynamic_suction_acceptance.py` 中为 `deferred` 桩，
需要实现（预计半天内）：

1. bag → 案例窗口：`bag_mcap_source` 直读 + 静置窗口/最后 10 有效帧选取；
2. 离线 YOLO（方案 1）或读录制的 `YoloDetections`（方案 2）；
3. 逐帧跑 ST-1 估计器 → 指标 1–5 计算；
4. scene_tf 六变体回放（指标 6）；
5. T0/T1/T2 写出与 `--replay` 支持；
6. 附录 A CSV 的读取与 GT 对齐。

## 附录 A：现场记录表模板（CSV）

```csv
case_id,size_actual_LxWxH_m,position_code,measured_center_xy_m,measured_top_z_m,yaw_deg,overhang_note,bag_path,operator_notes,done
a5_carryon_C_0deg_01,0.551x0.402x0.253,C,"(-1.00, 0.01)",1.113,0,,,,
a5_large_X-_90deg_07,0.803x0.501x0.321,X-,"(-1.25, -0.02)",1.181,90,overhang_x_0.15,,,,
```

## 附录 B：执行清单（现场当天）

- [ ] 平台 5 位置标记复核（±10 mm）
- [ ] 三箱实测尺寸 + 平台顶面 z 实测 → 附录 A
- [ ] 机械臂 pickup_observe 静止，D555/TF/Livox 单一 owner 确认
- [ ] 录制链路试录 1 例并回放抽查（色深对齐、TF 完整）
- [ ] 45 例按预声明交错顺序录制（每例：摆箱 → 测参考 → 静置 3 s 窗口）
- [ ] 附录 A 完整、bag 路径齐全
- [ ] teardown：无任务遗留进程（record/eval/launch）
