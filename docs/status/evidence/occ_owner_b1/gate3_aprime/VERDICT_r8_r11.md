# r8–r11 计分尝试记录（2026-10-20；按用户目标指令：失败记录原因，等根因确认）

## 逐次记录

| run | 结果 | 直接原因 | 根因状态 | 额度 |
|---|---|---|---|---|
| r8 | 启动段中止（0 箱） | driver 90s 看不到 vacuum_controller（图发现门） | **未确认**；r9/r10/r11 图门均秒过未复现；driver 自诊断已入库（graph_diagnostics.json＋外部同步探针） | 未消耗（无运动） |
| r9 | BUILD_FAILED（箱 0，无运动：tf 空、无 goto） | `CertificationResult` 不含 `failed_edge_id` | **已确认已修**：install/st09_contracts 是 10-14 陈旧拷贝（合并构建包清单漏了该包）；重建后消除 | 未消耗 |
| r10 | HANDOFF_SETTLE_TIMEOUT（箱 0 transit 末端） | transit 后 settle 门 6s 超时 | **复现中**（见下） | 消耗（运动已发生：transit 121 sim-s） |
| r11 | HANDOFF_SETTLE_TIMEOUT（同点） | 同上，签名一致 | **二次复现** | 消耗 |

## HANDOFF_SETTLE_TIMEOUT 签名（r10/r11 一致）

handoff.json（两 run 同构）：
- 关节连续性完美（max_delta=0.0，previous_last==next_first）；
- 动作结果 `previous_leg_success=true`（FJT 目标正常完成）；
- **控制器输出冻结在目标值（out1=-0.854 恒定），反馈持续下垂**
  （fb1: -0.905→-0.930，r10: -0.928→-0.955），误差从 ~0.05-0.07 起
  **单调增长**至 0.075-0.10 rad（关节 1/2），settle 容差 0.01 永不满足；
- RTF≈1.0（仿真不饿）；附着正常（attach ok）；重力负载下垂＋指令不再
  校正的形态——**目标完成后位置保持未生效/被绕过**的候选签名。

**对照基线**：r5/r6 同段 settle 均 ready（0.37-0.39s），4 腿路线、
transit ~8-10 sim-s；r10/r11 是 **10 腿认证路线、transit 121/166 sim-s**。

## 候选根因（未证实，待用户裁定；未做任何修复）

1. **认证/执行器等待合并语义**：motion_executor 的等待改为
   `min(certified duration, G3 公式)`（合并裁决）——若 certified 时长
   覆盖不到目标完成后的位置保持段，动作完成但 hold 不生效，重力下垂
   无人纠正。r5/r6 用旧等待路径无此形态。
2. **10 腿/超长 transit 的路线特征**（prior 分支 route 机制部分自动合
   并后首次在计分路径出现）：长低速段＋负载下垂的组合暴露同一保持
   问题；r5/r6 的 4 腿短路线不触发。
3. r8 图盲区独立未复现（driver 自诊断已备好下次复现时自动留证）。

## prior 集成状态（r11 首次真正触发钩子）

- r10：钩子被 PackEvalDriver.on_detected_luggage 覆盖遮蔽（未调
  super()）——已修（acba6893，离线 MRO 验证 adopted/narrowed 正常）。
- r11：钩子触发，**adopted=false，reason=PRIOR_UNAVAILABLE_OR_STALE_SURFACE**
  （prepare 返回 None：sidecar 服务未就绪或表面未收到）。prior 在线
  采用仍未验证成功。

## 收口

- 按"3 次无报错"目标：当前 0/3，两次消耗额度（r10/r11）、两次未消耗
  （r8/r9 启动段）。
- 未动吸附/规划逻辑；未动 master/G3 分支指针；集成分支 HEAD=
  acba6893；全部证据在各 run 目录（含 graph_diagnostics.json、
  graph_probe_external.jsonl、handoff.json）。
- **等待根因确认后再修**（候选 1/2 均需用户裁定方向）。
