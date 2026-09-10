# PF-R9 g2 — D1 complete-path baseline（功能重构前，bebaa7c+ 工作树）

- 栈：主工作区当前代码（含 g1 的发布线程/stride2/B2 拆分），accepted
  profile，`pickup_observe`，有箱，完整接受订阅图 + D1 探针
  （`test/pf_r9_g2_d1_probe.py`：raw rgb、pre rgb/depth/cloud、
  preprocessor status、filter stats）
- 窗口：125 s 计分 + 15 s 预热；数据 `d1_baseline.json` +
  `depth_rgb_alignment.json`（30 s 补测）
- 阶段计时为节点内 last-256 滚动样本（~最后 25 s），字节为窗口累计差

## 每阶段归因（p50 / p95 / max，ms；n=256）

| 阶段 | p50 | p95 | max | 说明 |
|---|---|---|---|---|
| rgb_view（接收→视图，含 .copy()） | 0.036 | 0.103 | 5.19 | 廉价 |
| rgb_core（校验+插入+配对+观测构建） | 0.049 | 0.118 | 5.17 | 廉价 |
| depth_view / depth_core | 0.026 / 0.183 | 5.09 / 5.11 | ~5.1 | 廉价 |
| queue_wait（入队→发布线程取出） | 2.22 | 3.53 | 6.25 | 廉价 |
| **build_color**（ascontiguousarray+tobytes→Image.data） | **37.30** | 41.48 | 48.07 | **0.9 MB，~40 ms/MB** |
| **build_cloud** | **36.05** | 38.36 | 40.91 | 0.9 MB |
| **build_depth** | **24.17** | 26.94 | 30.68 | 0.6 MB |
| build_info | 0.14 | 0.21 | 0.28 | 廉价 |
| pub_color（Publisher.publish 返回） | 0.319 | 0.458 | 0.73 | **廉价**（CDR/传输在 FastRTPS 内部线程异步） |
| pub_cloud / pub_depth / pub_info | 0.32 / 0.19 / 0.03 | — | — | 廉价 |

窗口总账：emission 1360/3292 = **0.413×**（bar 0.8）；raw→pre_rgb p50
**269 ms**（bar 60）；字节 color/cloud 各 1.418 GB、depth 0.409 GB/125 s。

## 根因结论

1. **全部瓶颈在输出消息构建**（`build_*` ≈ 97 ms/发射周期）：
   `np.ascontiguousarray(...).tobytes()` + rclpy `Image.data` 赋值，
   ~40 ms/MB。发布线程周期 ≈ 37+24+36+0.3 ≈ 97 ms → 发射被钳在 ~11 Hz，
   raw→pre 延迟 p50 ≈ 269 ms。g1 记录的"发布 p95 233 ms"实为构建+发布
   合测，已在本 D1 中拆分更正：**publish() 本身 0.2-0.5 ms**。
2. 接收→视图→配对→队列全链 <1 ms 量级；`rgb_view` 内的全帧 `.copy()`
   （0.9 MB）实测 0.04 ms——**Python 侧像素拷贝不是当前瓶颈**，
   `tobytes()` 才是。D2 的价值因此精确落在：`out.data = 原始
   array.array('B')`（恒等赋值）替代 ascontiguousarray+tobytes。
3. depth 图流与 rgb **100% 精确同戳**（756/756，30 s 补测）——与生成式
   点云（84.8% 匹配）不同，depth-primary 配对门不会因戳缺失丢帧。
   本基线中 pre_depth 附着率仅 44%（595/1360）是 g1 残留 cloud 路径
   挤占单线程执行器导致 depth 订阅队列丢包，g2 删除 cloud 路径后消失。

## 避免型全帧物化清单（D2 移除对象）

1. `image_array_from_msg` 的 `.copy()`（接收，RGB+depth）
2. `update_rgb/update_depth` 的 `frame.copy()`（入环）
3. `_build_if_ready` 的 `rgb.copy()/depth.copy()`（观测构建）
4. `_try_emit` 返回的 `emitted.copy()`（含 rgb/depth 再拷贝）
5. `image_msg_from_frame/depth_msg_from_frame` 的
   `ascontiguousarray(...).tobytes()`（输出构建，**主要成本**）

## 改后速率投影（D1 要求）

- 仿真对 1.536 MB（RGB8 0.9 + 16UC1 0.6）：构建≈头重写 ~0.1 ms →
  发布线程周期 ≈ queue 2.2 + 4×pub 0.3 + status ≈ **~5 ms**（上限
  ~200 Hz）；发射受配对语义约束：depth 100% 同戳可附着、RGB 有效 ~23-26
  Hz → 投影 **0.85-1.0×**（≥0.8 ✓）。延迟 = 双流到齐（≤1 帧间隔
  ~33 ms 量级）+ 队列 2 ms + 发布 <1 ms → p50 投影 **≤40 ms**（≤60 ✓）。
- 硬件对 1.152 MB（640x360）：同比例；D8 实测。

## 清场

`stop_sim.sh` 后残留 0。
