# Todo 2 验收记录：检测 vs GT 精度门

日期：2026-08-27
launch：`ros2 launch luggage_gazebo sim_world.launch.py gui:=false use_rviz:=false use_semantic:=true`
位姿：`pickup_observe`（驱动内 FJT 一次，循环中手臂不动）
backend：`bbox_fill:yolov8s-world.pt`（非 stub）
权威口径：`DetectionAccuracy` + `scripts/detection_gt_gate_run.py`（检测器 `evaluation_compare_gt` 保持关闭）

## 交付物

| 文件 | 说明 |
|---|---|
| `luggage_perception/box_geometry.py` | 无 ROS：IoU / 长宽比 / yaw 折叠（估计器也用） |
| `luggage_perception/eval/detection_accuracy.py` | 评测：`compare` / `summarize`，检测节点不 import |
| `luggage_perception/eval/detection_gate_sampling.py` | 评测：GT 回退不计分、stamp 对齐、dump |
| `scripts/detection_gt_gate_run.py` | N 次 spawn → 等活 geometry_ok → detect → GT → compare → clear |
| `docs/status/evidence/detection_gt_gate/trials_healthy.jsonl` | 本轮 N=20 |
| `docs/status/evidence/detection_gt_gate/summary_healthy.json` | 汇总 |
| `docs/status/evidence/detection_gt_gate/trials_invalid_gt_fallback.jsonl` | 首轮无效对照（20/20 GT 回退，gz 失速） |
| `docs/status/evidence/detection_gt_gate/rtf_stall_evidence.txt` | 失速判别实验 |

## 本轮栈健康度（先于精度）

采样前：`real_time_factor=1`，控制器 active，segmenter ready，gz 内无残留 `pickup_box_*`。
采样后：RTF 仍为 **1.000**，`ign model --list` 无 pickup 箱。

spawner 删除补了 `Entity.type = MODEL`（原先默认 `NONE=0`，gz 报 `of type [0] not found`，ROS 仍可能 success，箱子叠在同一点把物理拖死）。没有这次修复，N=20 会再次变成基础设施尸体。

驱动三处缺陷已生效：GT 回退不再 compare；geometry_ok 要求 wait 开始后的新点云 + 更新的 `primary_stamp`；`summary.json` 用 `"w"` 写出。

## 验收对照 plan 第 6 节

| 项 | 结果 |
|---|---|
| N=20 全部有 `AccuracyResult` | **否**：16/20。4 次 GT 回退（3×`DETECT_STALE_CLOUD` 在 FJT 刚结束、1×`DETECT_LOW_CONFIDENCE`） |
| 报表含 GT/测量/误差、P50/P95、通过率、失败码 | ✓ `trials_healthy.jsonl` + `summary_healthy.json` |
| 通过率 ≥ 90%（有结果的样本） | **否**：3/16 = **18.75%**（整次 3/20 = 15%） |
| 主因 | **尺寸欠估**（12/16 `reason=size`），不是 XY/yaw，也不是 `DETECT_TOO_FEW_POINTS` |
| 放行 todo 3（pick/retreat） | **不放行**（按当前书面门槛） |

## 精度（16 次感知结果）

门槛初值：`tol_xy=0.03` `tol_z=0.02` `tol_size=0.05` `tol_yaw=0.15` `tol_iou=0.60`。

| | P50 | P95 | 越门槛次数 |
|---|---|---|---|
| err_xy | 1.1 cm | 2.9 cm | **0**（P95 仍 < 3 cm） |
| \|err_z\| | 1.1 cm | 2.0 cm | 1（trial 14，−2.6 cm，同时 size 也越） |
| \|err_yaw\| | 0.005 rad | 0.016 rad | **0** |
| IoU | 0.75 | 0.91 | **0**（最低 0.60） |
| \|err_width\| | 7.4 cm | 22.7 cm | 见 size |
| \|err_depth\| | 5.5 cm | 10.5 cm | 见 size |
| \|err_height\| | 2.2 cm | 4.0 cm | 见 size |

`reason` 分布：size 12、ok 3、z 1。若暂时不计 size，15/16 过其余门（唯一例外是 trial 14 的 z）。

这与 Todo 1 网格复测一致：中心已经进 3 cm 门（当时 0.6 cm；本轮连续尺寸随机，P50 仍 1.1 cm），宽深欠估仍是 bbox 收边量级。plan 写过 size 不进运动学，只用于 catalog snap / 可视化——**抓取位姿这条链是过的，catalog snap 这条不是**。

## 失败码（检测器 `diagnostics_json.reason`）

| 码 | 次数 | 层 |
|---|---|---|
| 感知 `ok`（随后 Accuracy 再判） | 16 | 过了 ①–⑥ |
| `DETECT_STALE_CLOUD` | 3 | ① 云龄，trial 00–02（FJT 刚完） |
| `DETECT_LOW_CONFIDENCE` | 1 | ⑤ trial 06 |
| `DETECT_TOO_FEW_POINTS` | 0 | — |

YOLO / RANSAC / PCA 在 16 次有结果的样本里都跑过；STALE 三次算法未调用。

## 采样偏差（影响 size 数字，必须写明）

整轮墙钟约 **17 s**（含 FJT）。相邻若干次 `measured` 字节级相同，例如 trial 03 与 04 都是 `0.566×0.358×0.304`，GT 却是两只不同的箱子。驱动等的是预处理点云新鲜，**没有等 cargo 点云 / YOLO mask 换到新箱子**。预处理 4–6 Hz，语义链更慢，0.2 s 一轮会把上一只箱子的估计套到下一只 GT 上，把 size 误差放大。XY 仍稳，是因为连续箱子都在 `(-1, 0)`。

后续若重测 size 门槛，应在 spawn 之后等 cargo stamp > spawn 时刻（或固定间隔 ≥1 s），否则 P95 宽深不能当算法上限。

## 与首轮无效数据的对照

首轮 `trials.jsonl` / `trials_invalid_gt_fallback.jsonl`：20/20 `DETECT_STALE_CLOUD` + GT 回退，IoU 假 1.000。根因是 gz RTF≈0.009（叠箱：删除 type=NONE）。本轮同一驱动在 RTF=1 下得到 16 次真测量，假通过漏洞已关掉。

## 门槛回填建议（plan：用本轮 P95 决定维持/放宽/收紧）

| 项 | 建议 | 理由 |
|---|---|---|
| `tol_xy` 0.03 | **维持** | P95 2.9 cm，贴线但过 |
| `tol_z` 0.02 | **维持** | 1 次越界；attach 对 z 敏感 |
| `tol_iou` 0.60 | **维持** | 16 次全过，主分数健康 |
| `tol_yaw` 0.15 | **维持** | 远小于门槛 |
| `tol_size` 0.05 | **先不放宽、也不据此拦 todo 3 的位姿** | 欠估是已知 bbox 问题；本轮 P95 还被「旧 cargo 套新 GT」污染 |
| 通过率 90% | 本轮未达到 | 要等 cargo 新鲜度修好后再算一次 size 门 |

## 结论

- **不放行 todo 3**，若把书面通过率（含 size、且要求 20/20 有结果）当硬门。
- **位姿可以进入 pick 调试**：XY/IoU/yaw 已过初值门；size 失败不应解释成「找不到箱子」。
- 下一步不是调 PCA。优先：（1）detect 前等新 cargo 帧；（2）Todo 1 已标的 bbox/prompt 收边，专门打 size；（3）FJT 后多等 1–2 s 再开第一轮，消掉开头三次 STALE。
