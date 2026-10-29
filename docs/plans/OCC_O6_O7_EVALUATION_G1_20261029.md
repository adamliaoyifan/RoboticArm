# O6 完整性能矩阵与 O7 持续运行评测计划

## 1. 目标、版本与交付边界

用户授权 Cursor / Grok 4.7 补齐评测工具、验证工具行为、冻结测试版本，再运行 O6/O7 并归档结果。

- 产品固定 `4c8f1862ec357cb01616f4b04be636227461cf44`，沿用 `/tmp/elfin-occ-octomap-perf-cursor-g1`，不新建 worktree。
- 只修改测试工具、测试用例和文档；不改 OCC 算法、ROS 接口、队列、健康发布频率或门限。
- Release、OctoMap 1.9.8、discretize=false；保留全部有效端点和射线。
- 当前测试主机，不启动 Gazebo、不接机械臂；不能推算为 AGX Orin 部署能力。
- 产品、工具、配置、输入分别记录 SHA；记录环境、命令、Git dirty count 和受影响文件。产品源码/运行时必须与固定 SHA 一致，其他并行工作不得混入受验产物或提交。
- 产品缺陷留证后另行修复，不边改产品边认证。分别给出 O6、O7、完整产品验收结论；不自动迁移生产或关闭完整装箱验收。

## 2. 评测工具补充

扩展 `scripts/g3/occ_observer_eval.py` 和包内评测类，复用计时、摘要、冷启动及报告逻辑，不另建一次性程序。

O6 已知工具问题先修复：full 模式覆盖倒角和长方体；底层 benchmark 实际使用 manifest 的 outside/cross 参数，取消写死 0.5/0；manifest 不写死旧产品/计划 SHA；提供完整机器可读结果，不用截断 Markdown 判定。

负载定义冻结：outside_fraction 为全部有效端点中 ULD 域外比例；cross_fraction 为域外端点中射线穿过 ULD 内部比例。域内、域外穿越、域外不穿越互斥，外部两类数量按比例整数取整，余数归域内。独立几何判定验证实际类别，不能凭生成分支认定。性能输入全部通过光学 Z 过滤，无效输入由正确性测试覆盖；不得靠过滤降低负载。新定义的数据不能与旧失真配置直接作速度比。

O7 替换 cmd_concurrency() 占位返回，建立可复用持续运行测试类：启动冻结 ROS observer，subscription 输入、同时间戳 TF；独立定时发布点云、发起查询并订阅 receipt/health，每个输入唯一 ID。发送线程不得等待上一帧完成。真实 acquire→ROI→Surface→release，每链同一 snapshot 并释放。以单调时钟记录计划/实际发送、请求响应、receipt 到达、health 间隔；直接读取已有 receipt.queue_ms 和 health 计数，不新增产品统计接口。沿用现有 500 ms health 发布，文档旧“1 Hz”更新为现状。

统一输出计划/实际样本数、输入身份、结果状态、耗时、计数守恒、资源统计、失败原因及 pass/fail/not_evaluated。

## 3. O6 矩阵与判定

144组：分辨率 {1,2,5} cm × 箱数 {0,5,20} × 容器 {倒角,长方体} × 每帧有效点数 {30000,100000} × outside {0.5,0.9} × cross {0,0.5}。

每组一个累计地图窗口，3帧预热＋30帧计分。计分 seed 0–9 轮转三次，每帧 seed 由组 ID、基础 seed、轮次确定，冻结生成器版本及最终输入 hash。每组另有30个新进程冷启动样本：每进程一帧，然后首次查询及立即重复查询；不声称清空操作系统缓存。

| 指标 | P95 | max |
|---|---|---|
| 30k integrate | ≤500 ms | ≤1000 ms |
| 100k integrate | ≤2000 ms | ≤5000 ms |
| snapshot acquire | ≤100 ms | ≤250 ms |
| ROI | ≤100 ms | ≤250 ms |
| Surface | ≤100 ms | ≤250 ms |

首次 Surface、更新后重建、重复查询分别计分，缓存命中不得替代首次成本。初始化每次≤30秒；进程峰值 RSS≤2 GiB。读视图和缓存≤512 MiB 仅在可靠归属时判定，RSS不可替代；接口不能归属则 not_evaluated，保留 O6 未完成项，不改产品补统计。冷启动 integrate 单独报告，不混入预热窗口门限。

30样本 P95=升序第29个，中位=第15、16个平均。144组逐组判定，不能合并掩盖失败。缺样本、类别不符、归属不明不可通过。AND 仅检验本矩阵所有组满足约定门限，不估计长期单帧成功率。

## 4. O7 矩阵与判定

固定1cm、倒角、30k、outside=0.5、cross=0。正常运行：{0,5,20}箱 × 3次 × 120秒=9窗，每窗新进程。

1. 完成启动、TF及接口预演，3帧预热，确认队列清空。
2. 计分120秒，固定时间表发布240帧（2Hz）。
3. 同期1Hz启动120次 acquire→ROI→Surface→release 查询链。
4. 停发后最多30秒排空；这是测试截止，不是新业务延迟承诺。

正常窗要求完整120秒负载，240帧都有可关联终态，零拒绝、零缺失、零重复处理，不得少发降载。每类查询≥30有效样本，全部请求有最终结果，失败请求不可剔除。integrate/查询沿用 O6 门限，P95 取 ceil(0.95*n)。worker queue age max≤1秒；客户端发送到 receipt 总延迟另列。health 最大接收间隔<1秒，包含计分窗两端覆盖。输入数=成功完成+明确拒绝+待完成，排空后待完成=0；按唯一输入 ID 去重，一次 queue rejection 不可算两次。查询身份与内容一致，不混 snapshot。

过载另1窗：20箱、同配置，10Hz输入60秒、查询1Hz；停写最多30秒排空，再2Hz运行30秒检查恢复。允许过载阶段明确拒绝，不套正常零拒绝/延迟门限；要求无崩溃、死锁、静默丢失，输入可核账，已接受输入最终完成，队列排空，恢复请求正常完成。必须实际观测排队或拒绝，否则过载覆盖 not_evaluated；不自动提负载重试。

9正常窗 AND 汇总，所编码有限窗内每输入/请求成功要求为100%（240输入、120查询链/窗），不外推长期可靠性；任何窗失败不能被其他窗平均掩盖。过载覆盖独立报告，缺样本为 not_evaluated。

## 5. 工具自测与执行程序

工具行为测试覆盖：144组完整且两容器执行；outside/cross 真正改变输入且独立分类符合 manifest；seed/hash复现且产品/工具 SHA 分离；29样本不得通过、阈值相等通过、单窗失败不可合并掩盖；首查/缓存分开、缺摘要及错误响应不算正常样本。O7模拟时钟覆盖独立发送、延迟完成、拒绝、重复 receipt、失联、排空超时、计数守恒。子进程超时/退出清理干净且保留失败输出。启动、就绪和 wiring 测试验证行为，不以源码字符串断言代替。

执行程序（工具 CLI 补齐后由 owner 将精确可运行命令写入 COMMANDS.sh，再冻结并开始计分）：

1. scripts/agent_start.sh 领取本任务。核对 Q-20261029-4 原始证据和产品身份，确认代表性能前置条件，不重复有效同版本样本。
2. 完成工具自测及 ROS 非计分预演，可重复修工具。启动/就绪失败不耗计分样本，记录其次数、失败率和墙钟；失败中间数据先保存。
3. 提交冻结工具；冻结144组 manifest、9窗时间表、过载参数、生成器/hash及 COMMANDS.sh；先验证捕获健康与失败重放路径。记录工具 SHA 和本计划精确提交 SHA。
4. 使用 /tmp/elfin-occ-backend-perf.lock 串行运行重负载，等锁不算失败。先 O6 后 O7；性能失败仍完成当前窗口并留证，语义错误或崩溃停止依赖运行。
5. 测量总预算8小时，不含等锁；未完成逐项 not_evaluated，不削减样本。工具错误影响成绩则旧窗口标无效且保留，修复、提交新工具版本后重新测量，不覆盖旧结果。
6. 归档完整报告、工具提交、角色记录，使用 scripts/agent_complete.sh 按实际结果关闭；不可将 fail/not_evaluated 冒充 pass。因门限/预算/证据阻塞则记录 blocked 与剩余项。

## 6. 证据、转储与交付

保存在 docs/status/evidence/occ_o6_o7_20261029/runNN/，每次新 run，不覆盖。T0 包含产品/工具/配置/输入/环境身份，dirty 状态，命令、起止、CPU/RSS、初始化、清理、遗漏、文件清单及 SHA。完整机器可读每帧/请求记录、144组判定表、9窗守恒表、过载恢复表和报告一并保存。

| 边界 | T1 常开 | 触发 | T2 payload | 窗口 | 格式/预算 | 重放/检查 |
|---|---|---|---|---|---|---|
| 输入生成/发布/TF | ID、seed/hash、类别数量、计划/实际时刻、TF身份 | 类别/时序异常、超时、门限失败 | 全部地图输入可校验生成 manifest 或无损输入、同戳 TF、首失败输入前缀 | O6失败前缀；O7前后各5秒详情及完整轻量时间线 | JSON/JSONL、必要无损压缩；每例512MiB | runNN/COMMANDS.sh 中冻结 replay 子命令 |
| 接收/worker/receipt/health | ID、queue_ms、状态、计数、health间隔、资源 | 拒绝、重复、缺失、失联、超时、守恒失败、崩溃 | 对应消息、receipt/health、stdout/stderr、上游输入前缀 | O7前后各5秒；崩溃后不可得部分明确缺失 | JSONL/日志，同一每例预算 | 同上；核对唯一ID守恒 |
| acquire/ROI/Surface/release | 请求ID、snapshot身份、参数、响应状态/摘要、单调耗时 | 错误、身份/内容不符、超时、门限失败 | 查询参数与响应、输入前缀、TF、stdout/stderr | 首失败及前后各5秒消息 | JSON/JSONL，同一每例预算 | 同上；首次/重建/重复分别复核 |

总证据≤8GiB、每例详细转储≤512MiB；T1轻量且全程启用，预演记录实际捕获开销，不用 debug 重跑计时代替干净成绩。保留全部轻量时间线及重放关键输入，用压缩/生成 manifest 控制大小。超限或缺失不可静默截断，标 capture_complete=false、replay_possible 及 missing_artifacts，不完整证据不可通过；在拆卸、覆盖或盲目重跑前持久化并校验。只在边界定位后才启用 T3，注明开销。

最终明确 O6结论、O7结论、仍未覆盖的完整产品验收项；不得自动迁移生产。

## 7. 派工与指针

| subtask | owner/model | dependency | generation | base | plan revision | acceptance/tests |
|---|---|---|---|---|---|---|
| OCTOMAP-O6-O7-EVALUATION | cursor/grok-4.7 | OCTOMAP-APPLY-EVALUATION | 1 | 4c8f1862ec357cb01616f4b04be636227461cf44 | 以派工 thread 的 plan_revision 精确绑定本文件提交 | 第2–6节：工具自测、非计分预演、O6/O7及失败证据，owner端到端完成 |

- docs/agents/discuss/2026-10-29_1917_occ-apply-evaluation-g1.md（Q-20261029-4，done/pass）
- docs/status/evidence/occ_owner_b1/occ_apply_evaluation_20261029/run01/FINAL_REPORT.md
- docs/plans/OCC_APPLY_BATCH_EVALUATION_G1_20261029.md
- docs/architecture/README.md（按索引读取相关规范）
- .cursor/rules/debug-evidence.mdc
- .cursor/rules/sim-lifecycle.mdc
- scripts/stop_sim.sh

协调文档只写主 workspace docs/agents；现有 worktree 用 AGENT_COORD_ROOT 指向主 workspace。不得修改其他 agent 未提交工作。
