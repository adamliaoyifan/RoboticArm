# PF-R9 — preprocessor throughput and cloud-wait semantics (B1-B6)

- 机器：AMD Ryzen 9 9950X3D, RTX 5090, ROS Humble (FastRTPS),
  `ROS_DOMAIN_ID=7`, accepted profile, 机械臂 `pickup_observe`
- 共同测量窗（B3/B4）：单次探针 ≥120 s，前 15 s 丢弃（plan 规定）
- 探针：`src/luggage_perception/test/pf_r9_period_probe.py`（B1）、
  `pf_r9_b34_probe.py`（B3/B4）；全部可用 `--score`/重放命令复跑

## B1 — 判别性测量（先行，改动前栈 @ f03ccc3+PF-R8 提交 7b0b41a）

`b1_period_probe_pre.json`（130 s 窗，有箱，全语义负载）：

| 量 | 值 |
|---|---|
| /camera/color/image_raw header 周期 p50/p95/max | 33 / 164 / 331 ms（RELIABLE 订阅，19.7 Hz 有效） |
| /camera/depth/points header 周期 p50/p95/max | 33 / 67 / 199 ms（25.9 Hz 有效） |
| 同 header 配对收达时延（wall 域，cloud−rgb） | p50 2.8 ms（signed −1.2）/ p95 22.9 / max 39.3 ms |
| RGB 无同戳 cloud 占比 | 15.24%（384/2520；cloud 侧不存在，非迟到） |
| BEST_EFFORT vs RELIABLE | BE 丢 31-37%（rgb 1581 vs 2519；cloud 2270 vs 3309）→ 保持 RELIABLE（B5 该项以测量为据跳过） |

时钟域：header 周期为 sim /clock 域；收达时延为同域 wall(monotonic) 相减，绝不混域。

**参数规则**（测量 → 参数）：gz rgbd_camera 同步同戳 → 配对容差取 ~exact
5 ms；等待截止取 > 收达时延 max 39.3 ms 且留 ~50% 裕度 → 60 ms，远低于
camera_horizon_sec 350 ms（无钳制问题）；截止时钟 = RGB 流自身（最新 RGB
header），/joint_states 不再推进它（旧 now_hint 缺陷）。

## B2 — camera_slop_sec 拆分

`camera_pair_tolerance_sec: 0.005` + `camera_wait_deadline_sec: 0.060` +
`camera_emit_rgb_only: false`（截止到期无 cloud → 跳过并计数
`cloud_wait_timeout`；RGB-only 通路保留可开、被测、永不伪造 cloud）。
B1 实测 15.2% RGB 戳无 cloud，发 RGB-only 会把 cloud_ok 压到 ~85% < 95%
（B4），故默认跳过。诊断输出新增 pair/deadline/skips 字段。

## B5 — 已实施成本项（及以测量为据跳过的项）

- float32 cloud 数学 + BLAS-free 变换：原 `pts @ R.T` 把 (307k,3)@(3,3)
  派给 BLAS，OpenBLAS 16 线程自旋，preprocessor 一度 **435% CPU**；
  改为 ufunc 列表达式后 110%。节点入口 env 钉死 BLAS 线程=1。
- 解码快路径：传感器布局 point_step=24（xyz@0/4/8 + rgb@16），结构化视图
  + float32 stack；通用路径实测 21 ms/帧（负载下），快路径显著更低。
- 除去冗余拷贝：观测内部别名缓冲数组，返回值单次隔离拷贝（B6 隔离测试）。
- 发布与传感器回调解耦：3.7 MB（stride2 后 0.9 MB）cloud 发布在回调内
  实测 p95 233 ms（`cloud_stage_ms`），改为有界队列 + daemon 发布线程；
  无订阅者时跳过 0.6 MB depth image 发布。
- 降采样（plan 建议 E，条件启用）：stride 2（保留全部行列的 1/4，
  307k→77k 点，0.9 MB）。保留密度：cargo ~29.5k → **~7.4k** 点
  （min_points 50 / min_top_points 80，裕度 >90×）；几何非回归见文末
  gate4_short6 与 PF-R10 C1。
- filter 节点 stats 节流：~60-70 Hz json.dumps(RELIABLE+TRANSIENT_LOCAL)
  改 1 Hz 定时器（epoch 立即发）。
- 跳过项及依据：BEST_EFFORT 输入（B1 实测丢 31-37%）；MultiThreadedExecutor
  方案（实测 1 Hz 状态定时器与参数服务从不被调度、cloud 派发时延 ~70 ms
  与线程数无关——回退单线程 spin + 发布线程）；stride 3（B3 升至 0.615 但
  下游 filter join 0.910 / stale 0.080 破坏 B4，见下表）。

## B3/B4 — 探针迭代（125 s 计分窗，15 s 预热）

| run | 配置 | 发射/RGB | raw→pre_rgb p50 | cloud_ok | filter join | stale |
|---|---|---|---|---|---|---|
| 改动前基线 | gen3（参考） | 3.63 Hz ≈ 0.17× | 248.7 ms | ~50% depth_ok | — | — |
| v2 | MTE+BLAS 修复 | 0.139× | 108 ms | (探针 bug) | 0.967 | 0.030 |
| v4 | +降采样 stride2 | 0.264× | 109 ms | 0.986 | 0.994 | 0.006 |
| v5 | +发布线程 | 0.402× | 275 ms | 0.978 | 0.992 | 0.007 |
| v6 | +快解码/廉价跳过/免depth | 0.462× | 256 ms | 0.966 | 0.989 | 0.012 |
| v7 | stride 3 | 0.615× | 237 ms | 1.000 | **0.910 ✗** | **0.080 ✗** |

**最终配置 = v6（stride 2）**：B4 全过（cloud_ok 0.966 ≥ 0.95；join
0.989 ≥ 0.95；stale 0.012 < 0.05）。发射 11.0 Hz（0.462×，bar 0.8），
raw→pre_rgb p50 256 ms（bar 60 ms）—— **B3 未达**。

### B3 未达的测量根因（阻塞项）

1. **DDS 传输成本**：每次发射 ~1.8 MB（rgb 0.9 + cloud 0.9）经 FastRTPS
   unicast 至 ≥2 订阅者；发布线程周期 ~90 ms（≈24 MB/s 有效吞吐），单此
   一项就把发射率钳在 ~11 Hz，而 B3 bar = 0.8 × 23.7 Hz ≈ 19 Hz。发布
   前实测（未解耦时）p95 233 ms 直接阻塞回调。
2. **Python 解码/拷贝**：307k 点在 Python 侧的解码/isfinite/变换即便
   float32 + BLAS-free，负载下仍 ~16-59 ms/帧（`cloud_stage_ms`）。
3. **配对等待下限**：同戳 cloud 收达时延 max 39.3 ms → 发射延迟天然
   ≥40 ms 量级，60 ms p50 bar 本身接近物理下限。
4. **stride 3 转移瓶颈**：发射 0.615× 时下游 filter（不在本子任务文件
   范围，仅 stats 节流）join/stale 破坏 B4。

结论：在不越出 PF-R9 限定文件范围的条件下，B3 (≥0.8×, ≤60 ms) 以测量
为据不可达。plan 预案的触发条件成立（B1/B2/B6 达成、B4 达成、B3 未达）：
**像素空间掩膜（F3：depth 图上做 mask，仅反投影 cargo 像素，取消
307k 点云的预处理发布）应作为新共识项交 reviews**；该项把 cloud 发布
（0.9 MB×2 订阅者）与 Python 点云路径整体移除，是唯一同时满足 B3 两条
bar 的架构路径。

## B6 — 测试

`test_pf_r9_preprocessor_split.py`（8 项）：迟到同戳 cloud 在截止内配对；
容差外 cloud 不配对；截止不被 /joint_states 推进；截止到期在 RGB 时钟上
跳过并计数；RGB-only 发射被标记且不伪造；裁剪窗内 skip 集合有界；
返回观测与内部状态变异隔离（2 项）。既有 test_sensor_preprocessor 19 项
全部通过（1 项按拆分语义更新）。全量 510 通过。

## 整链 sanity（最终配置，非验收）

gate4_short6（本工作树，非 clean-worktree）：**active_output_hz 12.30**
（基线 3.6-3.8 → 3.1×，远超 PF-R10 C1 的 4.0 bar）；top_surface_rate
**1.000**（降采样未损伤顶面）；support_z 误差 0.0008 mm（亚微米）；
top_z p95 10.7 mm；false_measured_height 0。full3d_rate 0.935 略低于
0.95（18/278 帧，短运行波动，留待 PF-R10 三连跑判定；coverage 差异为
6-trial 短跑的预期项）。

## 清场

每次运行 `scripts/stop_sim.sh` 后残留计数 0。
