# 2026-09-10 -- PF-R10 gate4 整合：C2 收敛、C1 干净抽签可过、三连被 spawn 物理挡住

- role: eng
- agent: claude
- model: glm-5.3
- cli: claude
- status: open

## Summary

PF-R10 generation 2 当晚推进（claim 2026-09-09 19:51，线程
`2026-09-07_2039_pf-r10-gate4-integration.md`）。修复链 8 项全部落在
master `1998d47..474a9b8`，均附根因测量：glibc arena/mmap/BLAS 三层
分配器治理、支撑路径固定容量 scratch、tf2 异常 traceback 断链、
epoch 首/尾 gc、支撑稳定窗跨 epoch 延续与免清窗语义、spawner 落位
意图复位、探针 RSS 桶最小值方法论、warmup 30 帧重标定（owner 修正案，
待 reviews 审计）。

结果：**C2（PF-G6S）连续 5+ 轮全过**；**C1 在无翻倒抽签下 0.957-0.982**；
产出过一次完整 C1+C2 计分 run（full3d 0.965/top 1.000/active 18.6/
false 0/残留 0）。三连未成：spawn 接触物理翻倒（8-23 kg mesh 箱，
~30-40% run 命中一个 trial）呈现三副面孔——整程无检测、129k px 巨型
低置信 bbox、或检测成功但几何误差爆表（top_z 0.26 m）。set_pose 一次
性复位不可靠（疑似 gz 桥静默吞或复位后再倒）。

## Open

- 下一会话第一步：spawner 闭环落位（订阅 `/world/<w>/pose/info`，
  settle 后读实际位姿，roll/pitch/xy 超差复位重试 ≤N 次）+ 核实
  set_pose 是否被静默忽略；然后 `/tmp/pfr10_run.sh` 冲三连
  （worktree 需重建，run revision 取 master HEAD，dirty=0）。
- Q-20260909-11（D455 硬件验证延期）触发条件之一 = PF-R10 关闭。

## Pointers

- `docs/agents/discuss/2026-09-07_2039_pf-r10-gate4-integration.md`
- `docs/status/evidence/platform_free_height/2026-09-09_pfr10_gate4_integration/RESULT.md`
- `docs/plans/pf_r8_r9_perception_acceptance.md`（C1-C3 原文）
