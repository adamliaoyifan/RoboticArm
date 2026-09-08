# PF-R5B 结项证据 — 2026-09-05 16:20 closeout

- 运行环境：隔离干净 git worktree `/tmp/pfr5b_clean` @ **a3dba5e**（dirty=0，
  见 pfr5b_evidence.json `revision`）
- 口径：approved plan `docs/plans/platform_free_height_closure.md` @ 28370bc
- 结论：**全部硬门通过；run8 复用判定成立**

## 证据清单

| 项 | 工件 | 结果 |
|---|---|---|
| 精确 revision 证据 | `pfr5b_evidence.json` | a3dba5e，dirty 0 ✓ |
| 参数快照 | `pfr5b_evidence.json:param_dump` | 5 个在线节点全量 dump ✓ |
| 静态在线真值审计 | 同上 | AST 代码级引用 GetCurrentBox/size_eval = 0（docstring 排除）✓ |
| 运行时在线真值审计 | 同上 | 5 节点订阅表无 get_current_box/size_eval ✓ |
| 无箱负控 | 同上 | 36 帧全 DETECT_NO_CLOUD，0 有效 top/几何 ✓ |
| 正控冒烟（三档） | 同上 | GT 尺寸 == 锚定值（±1.5mm）+ version/sha256 身份齐全 ✓ |
| raw-only 负控 | `raw_only_negative_control/` | 3-trial 全帧 DETECT_CARGO_SEGMENTATION_REQUIRED、0 有效输出、active 3.67Hz ✓ |
| 清场 | `teardown_final.txt` | stop_sim 后残留进程 0（按进程名计数）✓ |
| 重跑边界机械判定 | `rerun_boundary_check.md` | ①核心计算 0 改动（仅身份字符串新增）②评测器 0 diff ③在线检测路径 0 diff ④六锚定值测试通过 → **复用 run8** ✓ |

## PF-A3 调和

PF-A3（2026-09-04_2029）check-8 阻塞项——spawner 在 STL 不可用时静默回退
catalog 尺寸——已在 PF-R5A/FIX1/FIX2（24060c7→192a6aa→5d677ac）修复并于
本 closeout 验证：三档正控证明 GetCurrentBox 只携带 STL 可观测参考（值 ==
锚定 + sha256 身份），失败路径测试证明零替代、零状态污染、RNG 确定性回滚。
gt_readiness 的 spawner 侧阻塞已解除。

## PF-R5 状态

- 算法结果：run8（c5921d5，原始未削减门全过，top_surface_rate 0.9606）
- 验收契约：本 closeout 补齐（a3dba5e）
- Gate 4 拆分文档：标记为 **proposal（待用户显式确认）**，关闭不依赖它
