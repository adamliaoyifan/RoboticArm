# 2026-10-31 -- S2 gen5复核：组装修复成立，真实执行器仍未交付

- role: reviews
- agent: codex
- model: gpt-6
- cli: codex
- status: done

## Summary

复核4b71073d850db16194d8c4fa9dd94c7ca6d1f2c3（scheduler d5a309e1），实现工作树干净。认可观测传递与离线反例修复；不认可Q-20261031-20整体outcome=pass。原验收4/5/6要求完整真实runner及同适配器短接线，仍未满足。S3继续关闭，本次未运行正式样本、ROS或新O9。

## Evidence

- 初次pytest缺libuld_occ_observer_io.so，13 passed/2 failed/1 deselected；保留原日志。source /opt/ros/humble/setup.bash及/tmp/s2-install/setup.bash后15 passed/1 deselected。不是产品行为失败，但COMMANDS.sh的运行环境需写全。
- 独立audit禁止任何subprocess.Popen，设置S3_OPEN=1、CONFIRM_OFFICIAL=1、ADAPTER=live：实际exit=2、reason=unknown_adapter，却标dispatched=true。只有injected分支调用OfflineAdapter。这不是未授权阻止运行，而是live分支不存在。
- 同一schedule用慢查询适配器（每次ros_query消耗虚拟1秒），两次发送时间0和1.5秒，合同要求间隔0.5秒。发送与查询同一顺序调用，未实现独立发送。
- 单独O7未满足O5依赖返回非零，符合停止规则。
- src/include/CMakeLists相对7537b033无差异；保留之前独立核对的O9，不要求重复跑。本次未再次重算30条trace。

## Source findings

campaign_executor.py:587 LiveProfileAdapter仍每帧materialize到相同目录、取f000，重启CLI；并非同一地图3预热+30累计，group对应实际config也未在调用中体现。更新后Surface、init缺失已诚实保留，但完整采集未实现。

:675 smoke_ros仍只启动service模式，1次_integrate；query/health/drain返回None。不存在完整topic及stability真实适配器。

:42 MonotonicClock.advance不sleep；:433调度按相对stamp差调用advance，既没有真实截止时刻等待，还会把查询执行时间叠加到发送时间。:503 stability先advance(180)，再调用30帧，无法证明180秒内持续采集。仅虚拟时钟累计时长不等于完整时间轴验证。LiveProfileAdapter默认ManualClock，剩余timeout<=1还传None，不能保证真实全局预算。

## Next todo

维持原gen5验收，不缩减成离线fixture验收。后续实现应以更高generation接续，保留当前Claim/Result历史：

1. 实现真正live路由、profile累计帧与三种独立Surface调用、topic/稳定性适配器；在进程启动边界注入验证真实选择路径，而非用OfflineAdapter替代整个入口。
2. 独立发送线程按单调绝对deadline调度；查询可耗时但不得拖慢计划发送。加入本次慢查询反例，验证实际/计划时刻、窗口终点与排空终点。O8在180秒区间内采集，不是先等再取fixture。
3. 同一真实适配器完成非计分短接线，采集identity、终态、query、health、资源缺项；正式门正确拒绝短样本。绑定真实时钟和剩余子进程timeout，超时不得变无上限。
4. 更新精确环境与COMMANDS，逐条标实现缺失/已验证/未跑正式。未交付完整runner不得整体pass。资源仍NOT_EVALUATED，S3不得开放。

## Adaptive merging

认可现有prune规则与新增受控同概率合并、不同概率不合并、局部更新再展开检查的方向。16/24节点与1536/1728字节、30k剪枝1.50ms为owner报告，本轮未重跑该探测。没有关闭剪枝对照，不能推断整图收益；也没有覆盖整个ROI/Surface/unknown/checkpoint语义矩阵。产品未改算法，本轮不必为此重跑O9。读视图与cache仍独立核算，树字节不能替代512MiB资源门。

## Pointers

- docs/agents/discuss/2026-10-31_1956_octomap-2cm-repair-s2-g5.md
- docs/agents/reviews/2026-10-31_1957_octomap-s2-g4-executor-review.md
- docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_g5_review/RESULTS.json
- docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_g5_review/audit.py
- docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_g5_review/pytest_env.log
