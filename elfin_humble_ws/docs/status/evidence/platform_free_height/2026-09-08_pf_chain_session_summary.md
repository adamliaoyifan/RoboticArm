# PF 链路会话总结 — 2026-09-08（PF-R6 gen3 → PF-R8 → PF-R9 → PF-R10 blocked）

- 执行者：`eng/claude/glm-5.3/claude`（用户会话授权直推全链条）
- 机器：AMD Ryzen 9 9950X3D（16 核）、RTX 5090、ROS Humble（FastRTPS）、
  Linux 6.8.0-138，`ROS_DOMAIN_ID=7`，accepted profile，机械臂
  `pickup_observe`
- 依据：plan `docs/plans/pf_r8_r9_perception_acceptance.md` @ `3460bff`
  （Codex 共识 2026-09-07 21:10）+ Q-20260907-1 记录的用户指令（PF-R6
  gen3 改派 claude）
- 本文档作用：跨子任务的单一查阅入口；各分项的完整证据在文末指针索引。

---

## 1. 生命周期总表

| 子任务 | 判定 | 实现提交 | 权威线程 | 关闭时间 |
|---|---|---|---|---|
| PF-R6 generation 3 | **pass** | `f03ccc3`（验证证据 `65164f8`） | `2026-09-08_1047_pf-r6-generation3-claude-user-directed.md` | 2026-09-08 10:4x |
| PF-R8 | **pass**（A1-A5 全过） | `7b0b41a` | `2026-09-07_2039_pf-r8-cargo-detection-availability.md` | 2026-09-08 12:0x |
| PF-R9 | **blocked**（仅 B3 未达，有测量依据） | `bebaa7c` | `2026-09-07_2039_pf-r9-preprocessor-throughput.md` | 2026-09-08 13:1x |
| PF-R10 | 未启动（依赖 PF-R9 pass，helper 正确拒绝） | — | `2026-09-07_2039_pf-r10-gate4-integration.md` | — |

会话提交序列：`f03ccc3` → `65164f8` → `135203a`（PF-R6 生命周期）→
`7b0b41a` → `1f764dc`（PF-R8）→ `bebaa7c` → `c0b83a0` → `59a2eba`（PF-R9）。

整链测试终态：**510 passed**（含 PF-R8 新增 20 项、PF-R9 新增 8 项）。
每次 sim 运行后 `scripts/stop_sim.sh` 清场，残留进程计数 0。

---

## 2. PF-R6 generation 3（检测器去瓶颈 + 关闭）

**变更**（基线 = gen2 脏检查点 @ `f34d917`）：
- `top_support_estimator.py`：支撑面拟合 RANSAC → **zmode_median**（8 mm
  z-bin 主导簇 + 中值；计数优先、5% 平票取高面）。其余全部不动（顶面
  RANSAC、环带、侧覆盖、残差、reason codes）；无在线回退。
- 保留 gen2 检查点的仪表与热点工作（stage_perf_probe、检测器
  instrumentation、lazy raw transform、band-first filter、cargo_voxel_size）。

**验证**：
- 离线 156 帧研究捕获（生产估计器）：valid 156/156，高度误差
  med/p95 3.68/10.72 mm（与基线一致），拟合 p50 0.102 ms。
- **30-trial Gate 4 精度矩阵**（干净 worktree @ `f03ccc3`，dirty=0，
  zmode 改变在线检测路径的强制回归）：**全门通过** ——
  top_surface_rate **0.9725**（≥0.95）、full3d **0.9823**（≥0.95）、
  support-Z 误差 **0.0007 mm**（亚微米）、height p95 10.7 mm（≤25）、
  XY p95 7.8 mm（≤30）、width/depth p95 35.6/43.9 mm（≤50）、
  false_measured_height **0**、覆盖 3 尺寸×30 trial / 26 yaw、
  0 spawn 失败 / 0 stale。对 run8 官方基线（c5921d5）：top +1.2pp、
  full3d +2.3pp，无任何精度回退。
- 检测器侧 stage 探针：valid geometry_ms p50 210.8 → **65.3 ms**；
  support 拟合 p50 126.0 → **0.604 ms**（~208×）；RSS 平坦。

**验收口径**：≥4 Hz 整链 PF-G6S bar 经共识计划 3460bff 分柱移交
PF-R9（吞吐）/PF-R10（整链门）——显式记录，非静默降级。

---

## 3. PF-R8（检测可用性：误检抑制 + 时域门修复 + 召回）

**A1 接受判定谓词**（工作区投影）：
- 规则：cargo bbox 中心像素射线反投影到取货平台平面（z=0.86，中心
  (-1,0)，实测静态几何，与检测器裁剪同类输入），落在
  max(half_extents)+margin = **0.65 m** 半径内才接受。世界系径向距离
  → 与相机 yaw 无关，夹具几何可精确判定。
- 实测分离度：右侧静态基座误检中心射线 ~**0.95 m**（拒绝）、行李箱
  ≤~0.5 m（接受）、边缘裁切大箱可见中心 ~0.47 m（接受，边界测试会误杀
  的那种场景）。
- 冻结夹具 `test/fixtures/pf_r8/acceptance_fixture.json`：**440 实例**
  （319 右缘负样本 = 计划六个复现 bbox/184 实例的超集 + 120 行李箱
  正样本 + 1 几何推导的边缘裁切正样本——全部历史捕获无真实样本，
  来源在夹具内注明）。**0 误收 / 0 误拒**。
- camera_info / 带戳 TF 缺失 → fail-open + `predicate_unavailable`
  标记（被标记，不伪造）；生产端每帧在 `~/stats_json` 暴露逐检测
  accepted/reason。

**A2 时域门**：正样本 = accepted 检测（`largest_cargo_bbox(
accepted_only=True)`）；窗口只记录 accepted bbox → 行李箱/误检交替不再
清窗；epoch 变化清窗（节点 current_box 路径，已测）；hold 到期归空不
留陈旧 bbox。

**A3 因果重放**（155 帧捕获，bbox_fill 标签图重建，20 帧 RGB 快照，
其余恒定图=无场景变化假设）：miss 集 = 计划独立测得的 **42 帧**（交叉
验证）；**0 无因果 hold / 0 漏 hold**；40 帧任何 5 帧因果 hold 均不可
恢复（归属 A4）。

**A4 召回**（测量定义恒定：捕获脚本用生产谓词函数 + 离线重放修复后
的门；settled = instance ≥5 帧）：

| run | 栈 | settled | recall | max miss run |
|---|---|---|---|---|
| baseline | `f03ccc3`（conf 0.04） | 508 | 0.874 | 33 |
| post iter1 | PF-R8（conf 0.01） | 489 | **1.000** | **0** |
| post iter2（确认） | 同上 | 512 | **1.000** | **0** |

根因与修复：两次 ~10 s 整 trial 掉检（93/109 miss 帧）= 行李箱在对角
yaw/光照状态得分 0.01-0.03，被 0.04 floor 切掉（与 PF-R5 记录一致）→
`confidence_threshold` 0.04→0.01 重新召回；低阈值带来的额外检测由 A1
谓词拒绝。A4-3：detection_frame 结果仅 ok/DETECT_NO_CLOUD，无误检产生
的有效几何。生产标注一致率 845 条匹配、双侧有 TF 时 **100%**。
`test_vintage_pose_regression` 通过。

---

## 4. PF-R9（预处理器吞吐 + cloud 等待语义）

**B1（先行测量，改动前栈）** `b1_period_probe_pre.json`（130 s，有箱）：
- 同 header 配对收达时延（wall 同域相减）：p50 2.8（signed −1.2）/
  p95 22.9 / **max 39.3 ms**
- 15.24% RGB 戳无同戳 cloud（发送侧不存在，非迟到）
- BEST_EFFORT 丢 31-37%（vs RELIABLE）→ 输入保持 RELIABLE（该项以
  测量为据跳过）
- 参数规则：容差 5 ms（gz 同戳→~exact）；截止 60 ms（> max 39.3 且
  ~50% 裕度，「 camera_horizon 350 ms，无钳制问题）

**B2 拆分**：`camera_pair_tolerance_sec`/`camera_wait_deadline_sec`/
`camera_emit_rgb_only`；截止走 **RGB 流自身时钟**（最新 RGB header），
50 Hz `/joint_states` 不再能杀死同戳 cloud；到期无 cloud → 跳过并计
`cloud_wait_timeout`（15.2% 无戳率下发 RGB-only 会把 cloud_ok 压到
~85%<95%）；RGB-only 通路保留、被测、永不伪造。

**B5 修复清单（按实测根因排序）**：
1. **BLAS 扇出**：`(307k,3)@(3,3)` 派给 OpenBLAS 全核自旋，节点
   **435% CPU** → ufunc 列表达式变换 + 入口钉死 BLAS 线程=1 → 110%。
2. **回调内大消息发布**：3.7 MB cloud 发布 p95 **233 ms**（PF-R6 时代
   "248 ms/帧"的真正根源）→ 有界队列 + daemon 发布线程；无订阅者时跳过
   0.6 MB depth image。
3. float32 点云数学 + 结构化视图快解码（传感器 point_step=24，
   xyz@0/4/8；通用路径负载下 21 ms/帧）。
4. 降采样 stride 2（plan 建议 E）：307k→77k 点 / 0.9 MB；保留 cargo
   密度 29.5k→**7.4k**（>90× 于 min_top_points）。
5. 冗余观测拷贝移除（变异隔离有测）；filter stats ~60-70 Hz json.dumps
   → 1 Hz 定时器。
6. 以测量为据放弃：MTE 多线程执行器（1 Hz 定时器与参数服务从不被
   调度、cloud 派发 ~70 ms 与线程数无关）→ 单线程 spin + 发布线程；
   stride 3（B3 升 0.615× 但破坏 B4）。

**B3/B4 迭代**（125 s 窗 / 15 s 预热）：

| run | 配置 | 发射/RGB | p50 延迟 | cloud_ok | join | stale |
|---|---|---|---|---|---|---|
| 基线（gen3） | — | ~0.17× | 248.7 ms | ~50% | — | — |
| v6（**最终**） | stride 2 全修复 | **0.462×** | 256 ms | **0.966 ✓** | **0.989 ✓** | **0.012 ✓** |
| v7 | stride 3 | 0.615× | 237 ms | 1.000 | 0.910 ✗ | 0.080 ✗ |

**B3 未达（bar 0.8× / 60 ms）的量化根因**：DDS 有效吞吐 ~24 MB/s
（每发射 1.8 MB × ≥2 订阅者）；Python 点云路径负载下 16-59 ms/帧；
配对等待天然下限 ~40 ms；更深优化需越出限定文件范围。**按计划自带
触发条款，像素空间掩膜（F3：depth 图上做 mask、只反投影 cargo 像素、
取消 307k 点云发布）已作为新共识项提交 reviews（Q-20260908-1）**，并
请示 PF-R10 可否按现状先行。

**整链 sanity（最终配置，非验收）**：gate4_short6 →
**active_output_hz 12.30**（基线 3.6-3.8 → 3.1×，远超 PF-R10 C1 的
4.0 bar）；top_surface_rate **1.000**（降采样未伤顶面）；support-z
亚微米；top_z p95 10.7 mm；false 0；full3d 0.935 略低于 0.95（短跑
波动，留 PF-R10 三连跑判定）。

---

## 5. 未决事项

1. **像素空间掩膜共识**（Q-20260908-1，open → reviews/codex-reviews-main）：
   消息契约变更、(u,v) 反投影归属、PF-R10 是否可按 stride-2 现状先行。
2. **PF-R10**：依赖 PF-R9 pass，暂不可领取；其 C1/C2 各 bar 在当前状态
   的前瞻：active 12.30 ≫ 4.0，top 1.000 ≥ 0.95，full3d 待三连跑。
3. reviews 对 gen3 会话授权代办的追认（Q-20260907-1 已答，无待办行）。

---

## 6. 指针索引

证据（`docs/status/evidence/platform_free_height/`）：
- `2026-09-07_pfr6_gen3/` — gen3 检查点（RESULT、CURRENT_ISSUES、
  stage_probe、gate4_short6 系列、failed_case_capture 155 帧）
- `2026-09-08_pfr6_gen3_gate4_30t/` — 30-trial 精度矩阵（RESULT/summary/frames）
- `2026-09-08_pfr8_recall/` — A4 测量（baseline/post×2/agreement_probe + RESULT）
- `2026-09-08_pfr9_throughput/` — B1 探针、B3/B4 v2-v7、gate4_short6_final + RESULT

线程（`docs/agents/discuss/`）：
- `2026-09-08_1047_pf-r6-generation3-claude-user-directed.md`（gen3，pass）
- `2026-09-07_2039_pf-r8-cargo-detection-availability.md`（pass）
- `2026-09-07_2039_pf-r9-preprocessor-throughput.md`（blocked on B3）
- `2026-09-08_1310_pixel-space-masking-consensus-trigger.md`（Q-20260908-1，open）

eng notes（`docs/agents/eng/`）：
- `2026-09-07_1830_pf-r6-gen3-zmode-implementation.md`（实现）
- `2026-09-08_1047_pf-r6-gen3-closure.md`（关闭）
- `2026-09-08_1200_pf-r8-cargo-detection-availability.md`
- `2026-09-08_1312_pf-r9-preprocessor-throughput.md`

复跑命令：各 RESULT.md 内含完整命令（30t 干净 worktree 流程、recall
捕获/评分、B1/B34 探针）；测试
`python3 -m pytest src/luggage_perception/test/ -q`（510 通过）。
