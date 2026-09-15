# Preprocessor A/B + CPU YOLO：本轮验收指标

- 日期：2026-09-15
- 树：`/home/adamliao/work/RoboticArm-master` @ `8ac1fca`（`origin/master`，detached）
- 主机：ThinkPad，无 NVIDIA GPU，`/opt/ros/jazzy`，RAM **30 GiB**，CPU Intel Ultra 5 225H（14 核）
- 本轮只验：**D555 流 → preprocessor A/B → YOLO-World CPU**。臂不动（`start_executor:=false`）。不做 FJT / pick / Livox detect。

权威副本在 master 树：`docs/status/site_preprocessor_ab_accept.md`。本文件是现场记事本同步件。

证据目录：

```text
docs/status/evidence/site_no_gpu_verify/20260915_pp_ab/
```

每 profile 一份 `meta.json` + 下表日志。A 与 B **分表，不平均**。

---

## 0. 合同（不许改阈值来凑数）

| 项 | 值 | 来源 |
|---|---|---|
| 相机 profile | **640×360@15**，禁止 896×504@30 | launch / SOP |
| 彩色 | `/camera/d555/color/image_raw/compressed` JPEG | architecture |
| 对齐深度 | `/camera/d555/aligned_depth_to_color/image_raw/compressed` 无损 PNG | architecture |
| `frame_id` | `d555_color_optical_frame` | SOP Gate 1a |
| 语义设备 | `semantic_device:=cpu` | 无 GPU |
| YOLO 上限 | `max_rate_hz:=2.0`（硬封顶，不是相机帧率） | `hardware_pick.launch.py` |
| 检测云超时 | `cloud_max_age_sec:=8.0` | 同上（CPU 慢时防 `DETECT_STALE_CLOUD`） |
| Profile **A** | `preprocessor_d555_live.yaml`：motion gate **on**（0.02 rad/s，settle 0.5 s），pair **5 ms** | master 合同 |
| Profile **B** | `preprocessor_d555_site.yaml`：gate **off**，pair **50 ms** | master 合同 |
| 禁止 | `preprocessor_d555_replay.yaml`（`use_sim_time: true`） | SOP |
| YOLO 权重 | `yolov8s-world.pt` + CLIP `ViT-B-32.pt`（≥ 300 MB） | Gate 0.5 |

仿真 yaml 里写过 yolov8s-world **640×480 CPU ≈ 29 ms/帧**。那是单次 predict、GPU 机上的 CPU 对照，**不是**本机 YOLO-World + 8 条 prompt + CLIP 的现场速率。现场以 **2 Hz 封顶** 为准，不追 15 Hz，也不用 29 ms 当门槛。

---

## 1. 帧率

窗口一律 **20 s**。测前相机已稳定 ≥ 5 s。

| 符号 | 话题 / 量 | 期望 | **PASS** | **FAIL** |
|---|---|---|---|---|
| `f_rgb` | `ros2 topic hz` 彩色 compressed | 15 Hz | 均值 **12–16 Hz**，且窗口内不低于 10 Hz | 均值 < 10，或 DDS 掉设备 |
| `f_d` | 对齐深度 compressed | 15 Hz | 同 `f_rgb` | 同左 |
| `f_pp` B，臂静止 | `/luggage/preprocessed/camera/color/image` | ≈ 相机 | 均值 **≥ 8 Hz**；`last_rejection` 无持续刷屏 | 均值 < 5 Hz，或持续拒对 |
| `f_pp` A，臂静止 | 同上 | ≈ 相机（gate=`stable`/`disabled` 以外应能过） | 均值 **≥ 8 Hz**；`motion_gate.state` 不是一直 `moving` | 示教器未动但 gate 一直 `moving`（先查 `/joint_states` 噪声，**禁止**关 gate 或放宽 5 ms 来签字） |
| `f_pp` A，关节在动 | 同上 | 应被闸住 | 运动期间接受率明显下降（相对静止） | 关节速度 > 0.02 rad/s 仍接近 15 Hz 出图（gate 没干活） |
| `pair_dt` B | status `depth_dt` | ≤ 50 ms | 接受帧 p95 ≤ **50 ms** | 持续超 50 ms 仍标 geometry_ok |
| `pair_dt` A | status `depth_dt` | ≤ 5 ms | 接受帧 p95 ≤ **5 ms** | 用 B 的 50 ms 文件冒充 A |

分辨率：`camera_info` 宽高必须是 **640×360**。不是则整闸 `CAM` FAIL。

---

## 2. YOLO 频率（必须和相机分开记账）

相机 15 Hz **不得**等于 YOLO 15 Hz。Launch 把 segmenter 封在 **2.0 Hz**，避免 CPU 队列把 DetectionFrame 堆过 8 s。

| 符号 | 怎么采 | 期望 | **PASS** | **FAIL** |
|---|---|---|---|---|
| `f_yolo_cap` | `ros2 param get /semantic_segmenter max_rate_hz` | 2.0 | **= 2.0**（±0.01） | 0（不限速）或 > 2.2 |
| `f_mask` | `ros2 topic hz /luggage/semantic/mask`（或 `/luggage/semantic/yolo_detections`）20 s | ≤ 2 Hz | 均值 **1.0–2.0 Hz** | 均值 **> 2.2 Hz**（封顶失效）或 **< 0.5 Hz**（有稳定 pair 仍几乎不推理） |
| `t_infer` | `/semantic_segmenter/stats_json` 的 `inference_ms` | CPU 上数百 ms 正常 | p50 **≤ 800 ms**，p95 **≤ 1500 ms** | p95 > 2500 ms 且伴随持续 `DETECT_STALE_CLOUD` |
| `yolo_drops` | stats `rate_limited_drops` | 相机 15 Hz 时应当丢 | 20 s 内 drops **增加**（说明 2 Hz 在限） | drops 为 0 **且** `f_mask`≈15 Hz（没限速） |
| `STALE` | detect 日志 `DETECT_STALE_CLOUD` | 非持续 | 偶发、driver 默认 5 次重试内恢复 | 连续失败 / 持续刷屏。**禁止**靠加大 `cloud_max_age_sec` 掩盖 |
| `backend` | stats `backend` | `yolo_world` | 含 `yolo_world` | `stub` / fallback（权重没挂上） |

`DetectLuggage` 本轮（箱子在观察位、`--skip-observe`）：B 上 **3/3**（`20260915_1905_g1d_yolo_crop`）；A 静止 **3/3**（`20260915_1936_g3` A4–A6）。`top_surface_valid=true` 才算检测成功。A 失败时先看 gate，不改 yaml。ROI 跟 YOLO bbox，不跟 `scene_tf` pickup 方框。

---

## 3. 内存（SOP 原先没有数字；本机 30 GiB 当场设上限）

RSS = `VmRSS`。窗口：图起来后 **稳态 60 s**（YOLO 已完成首次加载）。不要把 RViz 算进「图 RSS」，RViz 另记。

本轮 launch：`start_executor:=false`，`use_rviz:=false`，`use_moveit:=false`（本闸不测规划）。

| 对象 | 期望稳态 RSS | **WARN** | **FAIL** |
|---|---|---|---|
| `realsense2_camera` / `camera.d555` | 0.3–0.8 GiB | > 1.5 GiB | > 2.5 GiB 或进程 OOM |
| `sensor_preprocessor` | 0.2–0.6 GiB | > 1.0 GiB | > 2.0 GiB |
| `semantic_segmenter`（YOLO-World + CLIP CPU） | **2–5 GiB**（含首次加载尖峰到 ~7 GiB） | 稳态 > 8 GiB | 稳态 > **12 GiB**，或被 OOM killer |
| `luggage_detector` | 0.2–0.6 GiB | > 1.5 GiB | > 2.5 GiB |
| 上述感知图合计（不含 RViz / move_group） | **4–8 GiB** | > 12 GiB | > **16 GiB** |
| 主机 `MemAvailable` | 跑着仍 **≥ 8 GiB** | < 4 GiB | < **2 GiB**，或 Swap 开始持续上涨 |
| 泄漏 | 60 s 内 RSS 斜率 | segmenter 稳态后 60 s 增长 < 500 MiB | 60 s 增长 > 1.5 GiB（疑泄漏，停） |

首次加载 CLIP+YOLO 允许尖峰；**稳态**以加载后 30–90 s 的中位数为准。

---

## 4. 本轮判定

| 等级 | 条件 |
|---|---|
| **FAIL** | 任一帧率 FAIL、YOLO 封顶失效、持续 STALE、OOM、后端是 stub、误用 replay yaml、分辨率不是 640×360 |
| **CONDITIONAL** | B 的帧率 / YOLO / 内存 / detect 3/3 全过；A 因示教器微抖一直 `moving` 且已书面记录，未改阈值 |
| **PASS（preprocessor 闸）** | A 与 B 静止帧率都过；A 在运动时能闸住；YOLO 1.0–2.0 Hz；内存未破 FAIL；B 与 A 各 detect 3/3 |

完整无 GPU SOP 的 **PASS** 仍要 Gate 5 的 5/5 密封抓取。本文件只覆盖 preprocessor + YOLO 这一闸。

失败代码沿用 SOP：`CAM` / `STALE` / `DETECT` / `NET`。内存单独记 `MEM`。

---

## 5. 采集命令（master 树，Domain 7）

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=7
unset ROS_LOCALHOST_ONLY
export LD_LIBRARY_PATH=/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
source /home/adamliao/work/RoboticArm-master/install/setup.bash
source /home/adamliao/work/RoboticArm-master/deployment_ws/install/setup.bash

# B（先做；gate off，更容易出 pair）
cd /home/adamliao/work/RoboticArm-master/deployment_ws
./scripts/hardware_pick.sh start_executor:=false use_moveit:=false use_rviz:=false \
  semantic_device:=cpu \
  preprocessor_config:=/home/adamliao/work/RoboticArm-master/src/luggage_perception/config/preprocessor_d555_site.yaml

# 另终端，各 20 s
ros2 topic hz /camera/d555/color/image_raw/compressed
ros2 topic hz /camera/d555/aligned_depth_to_color/image_raw/compressed
ros2 topic hz /luggage/preprocessed/camera/color/image
ros2 topic hz /luggage/semantic/mask
ros2 param get /semantic_segmenter max_rate_hz
ros2 topic echo /luggage/preprocessed/status --once
ros2 topic echo /semantic_segmenter/stats_json --once
```

RSS：

```bash
ps -o pid,rss,comm,args -p $(pgrep -d, -f 'realsense2_camera|sensor_preprocessor|semantic_segmenter|luggage_detector_node')
free -h
```

A 把 `preprocessor_config` 换成 `.../preprocessor_d555_live.yaml`。默认 `hardware_pick.sh` 不加该参数就是 A。
