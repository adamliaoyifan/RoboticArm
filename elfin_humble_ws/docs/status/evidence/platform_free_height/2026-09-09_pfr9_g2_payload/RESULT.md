# PF-R9 g2 — payload-backed depth-primary（D1-D7 仿真侧证据；D8 硬件待跑）

- 栈：主工作区 g2 实现（提交见下），accepted profile，`pickup_observe`，
  有箱，完整接受订阅图，`ROS_DOMAIN_ID=7`
- 探针：`test/pf_r9_g2_d1_probe.py`（D1 基线）、`pf_r9_g2_d34_probe.py`
  （D3/D4，125 s 计分窗 + 15 s 预热）；全部可 `--out`/重放复跑
- 测试终态 **543 passed + 44 subtests**；`colcon build --packages-select
  luggage_perception luggage_gazebo` 通过；生产 `rg` 审计无
  `/luggage/preprocessed/camera/depth/points` 发布/订阅（仅测试夹具
  清单残留，checker 已不再要求该话题）

## D1 — 完整路径基线（功能重构前）✅

见 `D1_RESULT.md`。根因：输出构建 `ascontiguousarray+tobytes` ~40 ms/MB
（build_color p50 37.3 / build_cloud 36.0 / build_depth 24.2 ms）钳住
发布线程（0.413×、p50 269 ms）；publish() 本身 0.2-0.5 ms（CDR 异步）；
depth 图流与 colour **100% 精确同戳**（756/756）。

## D2 — 载荷恒等与隔离 ✅（单测 + 在线计数器）

- 接收→视图→缓冲→配对→copy-out→队列→重发布的完整路径
  **payload_materialisations = 0**（正式 D3 窗口内，第一方计数器）
- `out.data is 原始 array.array('B')`（双分辨率夹具断言恒等）
- 视图只读：`setflags(write=True)` / 赋值均 raise；帧/观测拷贝共享载荷
- 淘汰索引不失效在飞引用；CDR 仍会序列化（明示，不称端到端零拷贝）

## D3 — 吞吐与延迟 ✅（`d34_sim_final_v4.json`，125 s）

| 量 | 实测 | bar |
|---|---|---|
| 发射 / 唯一 colour 戳 | **1.248×** | ≥0.80 |
| raw→pre_rgb p50 | **18.6 ms**（p95 22.3 / max 38.6） | ≤60 ms |
| 载荷物化计数 | **0** | =0 |
| emit 队列丢弃 | 0 | — |

（>1 的比值来自探针 BEST_EFFORT raw 订阅欠计；预处理器 RELIABLE 输入
全量。真实发射 ≈ 相机有效速率。）

## D4 — depth-primary 连接 ✅（同窗 + detector 计数）

| 量 | 实测 | bar |
|---|---|---|
| 有效配对 depth / 发射（第一方逐发射 status flags.depth_ok） | **1.000**（n=3892） | ≥0.95 |
| filter 精确 join / 收到 depth | **0.983** | ≥0.95 |
| (stale_depth+stale_mask)/(depth+mask) | **0.0419** | <0.05 |
| detector 支撑 depth 命中 / joined cargo | **0.981**（hits 4434 + park 补发 1053；miss 107、expired 2） | ≥0.95 |
| 命名计数 | depth_wait_skips 0 / info_wait_skips 0 / acquisition_mismatches 0 | — |

修复链（测量根因逐项）：filter/segmenter 传输队列 depth=1 丢帧→10；
segmenter 每帧 stats json + overlay→1 Hz 定时器 + overlay 关（资源卫生，
不计 D3 分）；`_take_newest_join` 清空两缓冲误杀未来半对→外科手术式
只 retire 旧于已 join 键的项；detector lazy 查找的 3×20ms 回调内 sleep
在单线程执行器下永远等不到新到达→停靠-补发（bounded park + 到达完成 +
250ms 超时兜底 + 5ms 互斥组定时器发射——reentrant 定时器与 65ms 估计
并发风暴是最大尾部来源）；支撑深度 BEST_EFFORT 丢 ~9%→发布端 RELIABLE
（对 BE 订阅者兼容）。

## D5 — 单元与变异夹具 ✅

`test_pf_r9_g2_d5_fixtures.py`（19 测试 + 44 subtests，双分辨率参数化）：
RGB8/16UC1 恒等重发布、padded step、截断、大端 depth、已知内参/平面
反投影（z=2.0 m 全点差 <1e-6）、零/无效深度排除、维度/帧/K/编码/精确
戳失配（命名计数器）、同戳替换不增占用、第 16 帧容量淘汰、1 秒视界
淘汰、回滚容限内乱序、回滚冲刷+epoch+查找缺失、joint/IMU/lidar 时戳
不老化相机缓存、视图/帧拷贝/观测拷贝/队列/淘汰变异隔离。

## D6 — 支撑保留 ✅

配置 stride=4 与 stride+1=5、边距 0.03-0.18（合成 nadir 场景，双分辨率）：

| stride | inner 0.03 | 0.08 | 0.13 | 0.18(=outer, 退化) |
|---|---|---|---|---|
| 4 | 3120 | 2208 | 1168 | 0 |
| 5 | 1982 | 1430 | 708 | 0 |

`min_support_points=80`：非退化最小 708（裕度 **8.9×**）；live stride-4
实测 support_inliers=1785（裕度 22×）。几何极限不回归由既有 543 套 +
PF-R10 C1 承担。

## D7 — 迁移与有界性 ✅（RSS 趋势留 D8 会话补测）

- 生产 `rg` 审计：无 `/luggage/preprocessed/camera/depth/points` 生产
  发布/订阅；launch/RViz/Gate5/stage 探针/eval 驱动全部迁移；Gate5
  必备集移除点云话题并改为 manifest 声明光学帧（默认
  camera_depth_optical_frame，D455 后端声明 d555_color_optical_frame）
- 全部相关测试套通过（543）；`colcon build` 通过
- 相机缓冲 =15、下游 join 缓冲 =15、emit 队列 =4（正式窗 counters：
  队列丢弃 0；buffers 恒 15）；`payload_bytes_buffered` 报告在 status，
  无单调增长；节点 RSS 趋势采样在 D8 会话的长跑中一并记录

## D8 — D455 实测 ⏸ 按用户决定延期（2026-09-10）

用户决定：实机验证暂缓，待所有 depth-primary 相关改动完成后统一验收。
已登记为独立验证项（Q-20260909-11，线程
`2026-09-09_1939_d455-hardware-validation-deferred.md`）；触发条件 =
PF-R10 集成及后续感知管线改动全部关闭；验收内容 = 本计划 D8 全部 bar
+ 每节点 RSS 趋势采样。执行包：`D8_cell_procedure.md`（单元侧命令）。
D1-D7 仿真侧结果不受影响。

## 清场

每轮 `stop_sim.sh` 后残留 0。

## 提交

`7bd6ac8`（载荷+depth-primary 预处理器）、`42f97af`（filter/detector
迁移）、本轮修复链与证据（见 git log）。
