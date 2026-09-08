# 2026-09-05 18:30 — PFH eng 全量交接（接手 agent 用）

- role: eng
- agent: claude
- model: glm-5.3
- cli: claude-code
- status: done
- revision: 26e0891（交接时 HEAD）
- session: 4a4943b4-4405-44ac-b4a1-18bbf73ba7d7（RUNTIME.md 注册 id
  `claude-eng-pfr5`，接手者应注册自己的 id）

## Summary

Consolidated the PFH engineering state, PF-R6 partial results, exact evidence
pointers, operational lessons, and successor priorities. This note is a
handoff, not a claimed PF-R6 Result.

## 给接手 agent 的一页总览

你在接手 PFH（platform-free height）轨道的 eng 位。产品目标：ROS 2 人
协助装载产线（SIM-R1：操作员确认→探索→放箱→detect/pick/place/commit/
verify 循环）。你的轨道负责"感知几何 sim=real 契约"：在线节点零仿真真值、
fail-closed 语义、精确 revision 证据。并行轨道：TCIG（容器内几何，他人）、
MPF（mailbox，codex）。

### 任务状态（parent PFH-REMEDIATION-20260904）

| 子任务 | 状态 | 关闭点 |
|---|---|---|
| PF-R1..R4 | done | a001be7 |
| PF-R5（30-trial 算法） | done | c5921d5（run8，原门 0.9606 未削减） |
| PF-R5A+FIX1+FIX2（GT fail-closed） | done | 24060c7→192a6aa→5d677ac |
| PF-R5B（验收证据 closeout） | done | a3dba5e，文档至 62a75df |
| **PF-R6（性能 ≥4Hz）** | **open/blocked（唯一活跃项）** | 部分成果 @1ce5f6c |
| PF-R7（cursor 独立集成审计） | 等 PF-R6+PF-A1 | — |

### PF-R6 现状与下一步（你的首要工作）

**已做**（1ce5f6c，测试 461 绿）：
- `stage_perf_probe.py`：逐话题速率 + 跨级延迟 P50/P95/max + RSS
- 惰性 raw 支撑变换（按 stamp 缓存消息、join 命中才解码+TF）
- 支撑候选高度带前置（旋转/环带之前）
- 证据：`docs/status/evidence/platform_free_height/2026-09-05_1800_pfr6-profiling/`

**Blocked 根因（数据充分，等 reviews 裁定范围）**：
```
raw_img 21.6Hz → pre_rgb 4.4Hz → cargo 3.05Hz → frame 3.50Hz（门 4.0）
                              ↑ 28% 丢在语义滤波器 exact-stamp join
```
segmenter 的 temporal hold 重发旧 stamp（yolo 话题 4.8Hz 含保值重复），
与新增云 stamp 永不匹配。**检测器内优化无法突破 cargo 速率上限**。
候选方向：① segmenter 保值班次不再重发旧 stamp（或 filter 容忍保值
配对）② preprocessor 配对输出提速。改动会触碰在线检测路径 → 按闭环
计划 Fixed Decision 5，改后需重跑 30-trial 证明 Gate 4 无回归。

## 必读文档（按序）

1. `docs/agents/README.md` — 通信契约（线程权威/generation/手动加锁规则）
2. `docs/plans/platform_free_height_closure.md` — R5 关闭计划（已完成）
3. `docs/plans/platform_free_height_remediation.md` — PF-R6 节（优化顺序
   硬性：去重复用→预裁剪→早退→最后才算法参数；不弱化精度门）
4. `docs/agents/eng/2026-09-05_1720_pf-r6-handoff.md` — 命令/profile/坑
5. `docs/status/evidence/platform_free_height/2026-09-05_1620_pfr5b-closeout/COMMANDS.md`
   — 隔离 worktree 全流程

## 操作要点（血泪浓缩）

- **claim/close**：`agent_start.sh` / `agent_complete.sh`；pass 需真 commit
- **绝不**对 runnable 线程发 `--kind question` 通知（会覆写线程元数据，
  我已踩过并修复过三个线程）；回复用手动加锁 `## Reply` 或独立线程
- 发 reviews 用精确身份 `codex-reviews-main/gpt-5.6-sol`
- 证据文件（RESULT/JSON/负控目录）完成即 `git add`，不能只提交 note
- 隔离 worktree 三件套：CLIP vendor、`yolov8s-world.pt`、构建依赖链
  （elfin_* + 全 luggage_*）；证据目录用绝对路径指回主工作区
- **controller_manager 启动竞态**（~1/4 概率）：症状=detection_frame 0 帧
  + spawner fatal；处置=stop_sim 整栈重启；根治待做
- 诊断口诀：0 帧→查 process died→查 cargo/yolo 计数→查 join stamp
- 清场残留计数用 `ps -eo comm=`（pgrep 模式会自匹配 shell 包装）
- 定时轮询：cron 每 20 分钟 `agent_poll_self` + `agent_register` 心跳
  （session 级，会话亡则停）

## 遗留清单（按优先级）

1. PF-R6 上游范围裁定（等 reviews）→ 实施到 ≥4Hz → 30-trial 回归
2. controller_manager 竞态根治（独立 subtask 建议）
3. SIM 三箱闭环门（GT-fallback-free 驱动：detection→pick→place 全用
   检测输出；现有 eval driver 用 GetCurrentBox GT 需改造）
4. 代表性检测硬门（textured sim：world 预置纹理模型——Fortress 运行时
   导入纹理网格会永久杀死相机渲染线程，已实测；或等 Gate 5 实机包）
5. 底座 self-body 掩码（base_link 误检目前被工作区裁剪中和）
6. `platform_free_height_gate4_revision.md` 仍是 proposal（待用户确认）

## 验证基线（回归命令）

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
for p in luggage_perception luggage_planning luggage_packing luggage_gazebo luggage_description; do
  (cd src/$p && python3 -m pytest test/ -q | tail -1)   # 461/221/75/48/135
done
```

## Result

- pass: 交接材料完成（本 note + 1720 handoff + COMMANDS.md + 证据链）。

## Pointers

- 本 note + `docs/agents/eng/2026-09-05_1720_pf-r6-handoff.md`
- 证据根：`docs/status/evidence/platform_free_height/`
- 线程：`docs/agents/discuss/2026-09-04_1453_pfh-remediation-r6.md`（open/blocked）

## Open

- 无（交接完成；PF-R6 的 open 在其自身线程，归接手者）
