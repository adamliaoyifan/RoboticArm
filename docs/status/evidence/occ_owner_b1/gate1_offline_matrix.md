# B1 离线合同门 — 验收矩阵行记录（2026-10-16）

分支 agent/eng/master-pack-baseline-g3，提交 816ac464(负容差) → 37d66963(mapper手术)
→ 17859a72(store核心) → 475d8eaf(线合同) → f122f528(适配器+单快照发布) →
79f475c6(解码兼容修复)。全部为真实生产逻辑调用（真 store / 真节点 executor），
无源码文本匹配。

| 验收行 | 结果 | 证据（测试 ID） |
|---|---|---|
| 缺证据、NaN/Inf/负容差、错误来源 | PASS | test_occupancy_map_store_anchor.py 11 例（witness_02 0.65 m 拒、负容差拒、NaN 容差拒非静默 legacy、Inf 拒、超 0.30 顶拒、legacy==0 无踪迹、锚踪迹含 op id、零体素=INVALID_PAYLOAD 拒非异常） |
| 重复与并发 | PASS | test_occupancy_map_store.py：同 op id 竞争恰一 ACCEPTED 一 ALREADY_PROCESSED×20；同前提冲突写至多一成功×20；A→B/B→A 收敛同 digest×20；同 id 异内容拒；空 id 拒；重试跨 reset 不重放 |
| 重启、乱序、摘要冲突 | PASS(离线半) | reset 轮换 epoch；双 store 实例（重启）EPOCH_MISMATCH；CONTENT_DIGEST_MISMATCH 腿；迟事实记 pending+修订+1（gate on）/gate off 拒（fail closed）。live：test_reset_rotates_epoch（stale epoch→EPOCH_MISMATCH） |
| pending 与观测 | PASS | test_occupancy_map_store_confirmation.py 10 例：未确认→覆盖观测→确认；非覆盖观测不确认（遮挡≠空闲）；未 settle 前提拒；碰动/移除 fail-closed 待确认（B1 语义，B2 补 observed-free）；时间推进身份不变；同内容重观测 digest 不变 |
| 快照冻结 | PASS | FrozenInstanceError/TypeError；后续事务不泄漏进持有快照（身份/字段/digest 逐项） |
| 路线失效 | DEFER B2 | B1 交付其依赖：digest 属性（同内容稳定、柱高/来源/确认变化可检测）已钉；_same_admitted_map 换 digest 属 B2 |
| 正常释放 | DEFER B2/B3 | remove→pending fail-closed 已钉；observed-free 确认规则待 B2 |
| 生产场景一致 | PASS(投影半) | test_occupancy_map_store_projection.py：一快照三投影（v1 JSON/v2 record/committed）身份一致、状态阵列一致、v1 JSON 双构建字节稳定、commit_revision==map_revision（下游恒 1 bug 修复） |
| 缓存上限 | PASS | 幂等账 FIFO 封顶 256（测试 cap=3）；淘汰后带前提重试 fail closed、无前提重试 mapper 账本去重无重箱 |
| Select 换槽 | DEFER B2 | B1 只供身份；消费者侧比对属 B2 |
| 进程集成（live） | PASS(12/12) | test_cargo_map_integrate_live.py：既有 10 reason code 行为不变 + op-id 重试一次 + reset 轮换 epoch；适配器响应映射 test_cargo_map_node_adapters.py 7 例 |

回归：gazebo+perception 全套对照 stash 基线零新增失败 ID；planning 724 过
（2 存量 atlas flake）；description 4 存量（stash diff 验证）；64 例序列化
矩阵经解码兼容修复后字节兼容。

测试计数：store 核心 24 + 锚门 11 + 确认机 10 + 投影 5 + 适配器 7 +
adopt-release 4 + live 12 + anchor 原语 4 = 77 项（含并发×60 次重复）。
