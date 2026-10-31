# 2026-10-31 -- S1 gen3行为复核通过，开放S2非计分预演

- role: reviews
- agent: codex
- model: gpt-6
- cli: codex
- status: done

## Summary

绑定bfece5c32a0b4595827dd10993b88d4638c4f106开放S2，S3继续关闭。S1 gen3最高代done/pass，工作树干净。
独立Release构建observer_repair_s1_behavior通过；原资源probe三个断言全部通过：同快照缓存48000保持48000，reset/release后存活reader的2块计8336字节、缓存48000，8个cell可读。扩展行为测试包含最后所有者释放回空基线、两版本和并发响应。
独立复跑S0 12测试通过。核对owner在线22/22结果与脚本实际断言、节点hash一致；本次未重跑在线22项，不把owner证据描述为reviews实测。
本次认可已交付修复和已覆盖行为进入预演，不是产品或资源认证，也不是完整并发证明。

## S2 handoff

按批准主计划c02c1cfbd9ba16a1c4a6c6169695f1daf8eb6a76第6节，S2发布gen2，base为上述S1提交，依赖S1最高代gen3；owner eng/cursor/grok-4.7，plan_revision保持原主计划。S2 gen1未领取，superseded；新代就绪。
实施范围仍为原S2：真实启动/TF/独立发送/终态/恢复/失败重放预演与修复，预演2小时不含等锁；通过后O9三操作各30对AB/BA、P95比≤1.10、无缺失错配、真实故障均可重放；冻结产品/工具、安装包、最终输入hash、环境及完整COMMANDS.sh。S3 O5–O8正式评分不得提前开始。

### 预演必须处理的已知限制

- 资源覆盖表仍有目录节点、OcTree内部节点、中间件等缺项。保留NOT_EVALUATED，不能用部分字节小于512MiB宣布资源通过。S2交接列出剩余缺项及对S3资源门的阻塞；本次不授权放宽资源门。
- wake_waits把waiters_移至局部pending后释放wait_mu_，入口try_begin只看waiters_.size()。源码提示重叠回调可能低估在途等待数；本次未复现超过8，不声称容量门完整成立。S2预演必须用可控暂停/屏障触发“8个pending正在检查时第9个到达”，要求在途总数≤8且第9个WAIT_PRESSURE；若违例则在预演内修复并回归，提交新候选后才允许O9。不能用当前顺序9请求测试替代这个交错。
- S1在线脚本MultiThreadedExecutor未固定线程数；本次行为证据不作性能成绩。S2按原计划固定客户端4线程与独立发送调度，记录RMW/DDS和线程CPU等环境。
- S1 COMMANDS.sh的/bin/sh与source bash setup不一致，S2须交付实测可运行的bash入口，明确实际库/环境、产物SHA、候选dirty状态和输入hash。不能依赖未冻结/tmp内容或默默覆盖历史证据。
- S1行为用短保留期/持锁hook验证故障。S2真实边界捕获与重放资格另验证；评分配置关闭测试hook，不能以合成案例或S1 22项代签O9。

候选修改仅限预演修复与相关回归；一旦进入O9资格测量冻结产品/工具/配置，失败先保留完整证据，修改后结束原候选并重跑受影响资格。预算耗尽/缺样必须not_evaluated，不能宣称S2全部pass。

## Pointers

- `docs/plans/OCC_OCTOMAP_REPAIR_2CM_G1_20261031.md`
- `docs/agents/discuss/2026-10-31_1010_octomap-2cm-repair-s1-g3.md`
- `docs/status/evidence/occ_octomap_repair_2cm_20261031/s1_g3/FINAL_REPORT.md`
- `docs/status/evidence/occ_octomap_repair_2cm_20261031/s1_g3/live_checks.json`
- `docs/status/evidence/occ_octomap_repair_2cm_20261031/s1_g3/COVERAGE.json`
- `docs/status/evidence/occ_octomap_repair_2cm_20261031/s1_g3_review/MANIFEST.json`
