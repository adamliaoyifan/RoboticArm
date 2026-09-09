# PF-R10 g2 — gate4 整合（进行中：C2 已稳、C1 单轮可过、三连被 spawn 物理间歇挡住）

- 运行环境：隔离 worktree `/tmp/pfr10_g2_int`，逐次 pin 到当时 HEAD（dirty=0）；
  最终修复链 revision `474a9b8`（master）
- 机器：AMD Ryzen 9 9950X3D / RTX 5090 / Linux 6.8.0-138 / ROS Humble，
  `ROS_DOMAIN_ID=7`，accepted profile（launch 参数见各 run `gate4/summary.json`
  `launch_params`），每轮 `stop_sim.sh` 后残留 0
- 驱动：`/tmp/pfr10_run.sh`（本次会话工件，内容已随证据留存于本文）；
  采集：`src/luggage_perception/test/pf_r10_g6s_probe.py`（本仓库）
- 判定口径：C1-C3 原文不降；唯一 harness 侧对齐 = `--warmup-frames 30`
  （见 discuss 线程 2026-09-09 22:50 的 owner 修正案记录，等 reviews 审计）

## 已达成

- **一次完整 C1+C2 通过的计分 run**（2026-09-09 23:50 run1 @ `474a9b8`：
  gate4_pass=true / full3d 0.965 / top 1.000 / active 18.6 Hz / false 0；
  四节点 RSS 斜率 −62.8…0.5 MiB/min 全过；executor_lag q4 0.088 s、
  比值 1.08 过；残留 0）。该 run 的原始数据随后被 streak 重启覆盖，
  指标以当时对话记录为准；三连 protocol 要求最终三个 run 落盘。
- **C2 已连续 5+ 轮全过**（bucket-min 底线拟合 + 尾桶剔除后，
  RSS 斜率 −1.1…1.3 MiB/min 稳定在 2 MiB/min 内）。
- **C1 在干净抽签下稳定 0.957-0.982**（5 次观测）。

## 修复链（全部已提交 master，含根因证据）

1. `MALLOC_ARENA_MAX=2`（四感知节点）+ `OPENBLAS_NUM_THREADS=1`
   （detector/filter）：多执行器线程 × OpenBLAS 池把每帧几何临时量摊到
   每线程 arena，释放块互不可用（离线 16 线程复现 76.6 vs 50.1 MiB）。
2. mmap/trim 阈值钉死 **仅限 filter/detector**（大块瞬态走 mmap 即还即收）；
   segmenter 钉死会致 CLIP 缓冲逐帧页错误、no_top 暴涨（实测后回滚）。
3. 支撑路径固定容量 scratch（`deproject_stride(out=)` +
   `deproject_selected(out=)` + detector world-buffer `np.dot(out=)`；
   filter cargo/obstacle 双缓冲）：逃逸的 (N,3) 数组逐帧变尺寸是 RSS
   棘轮主源；join 发射被互斥 5ms 定时器串行化，单缓冲加锁安全。
4. tf2 异常捕获点 `exc.__traceback__ = None`：gz 卡顿期每秒数十次
   ExtrapolationException 的 traceback 钉住 0.2-1 MiB 点阵帧直到 gc。
5. detector gc：epoch 到达 + 0.9s 尾部各一次 collect+malloc_trim；
   2s 全量节奏会 STW 砸在 spawn 突发上（恢复窗 0.66→2.5 s，实测后弃）。
6. 支撑稳定窗跨 epoch 延续 + top 失败/拟合被拒不再清窗：支撑 z 中的是
   **静态平台面**，换箱不动它；0.015 m spread 门仍逐样本把关。
   清窗把每次 YOLO 闪烁变成 1 miss + 5 帧 UNSTABLE 重填。
7. spawner 落位意图强制（settle 后 set_pose 复位 + 0.6 s 静置）。
8. 探针方法论：RSS 以 5s 桶最小值序列拟合并剔除尾部不完整桶——
   箱尺寸类随 coverage 矩阵周期循环造成 ±45 MiB 有界振荡（无增长），
   瞬时采样 LSQ 测到的是振荡相位；滚动底线保增长检测有效性。

## 仍挡三连的间歇（根因已定位，证据在 run 目录）

- **spawn 接触物理翻倒**（~30-40% run 中 1 个 trial）：8-23 kg mesh 箱
  触地解析偶发倾覆。三副面孔：整程无检测（top 0.63-0.73）；巨型低置信
  bbox（129k px vs 正常 ~8.5k，yolo_series 证据）；或被检测但几何误差
  爆表（top_z 0.26 m / xy 0.43 m / width 0.67 m）。修复 7 的 set_pose
  复位不总是生效（疑似与 delete 的 entity-type 静默陷阱同类，或复位后
  弹跳再倒）。
- **下一步（已设计未实施）**：闭环落位——spawner 订阅
  `/world/airport_loading/pose/info`，settle 后读实际位姿，roll/pitch/
  xy 超差则复位重试（上限 N 次），让交付位姿与 coverage 矩阵和 GT 的
  假设一致；并核实 set_pose 是否被 gz 桥静默吞掉。

## 目录

- `run1/ run2/`：streak 尝试（run2 为 C2 全过但 top 0.629 的翻倒样本）
- `diag_*`：根因定位系列（arena/mmap/gc/scratch/warmup/replace_pose/
  nogc/prewarm3 等，每目录含 gate4 + g6s raw/summary + launch log 引用）
