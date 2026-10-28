# OCC副本summary三字段局部修复计划

- date: 2026-10-28
- status: approved_local_repair
- parent: OCC-QUERY-INDEX-20261025
- subtask: G4-A-SUMMARY-WIRE-FIX
- generation: 1
- owner: eng/claude/glm-5.3
- depends_on: G4-A-ISOLATED-CORE
- base_revision: 0da4086bce23be49ec8eeb934a15fb6829803abe
- dispatch_ready: yes
- scope: A后继局部产品修复；B评分暂停、无关脚手架准备继续

## 1. 问题和裁定

副本_MapSnapshotView缺occupancy_confirmed/pending_changes/change_event，真实summary调用先在第一字段抛AttributeError。修复encode/decode对称性，保持owner事实为唯一权威。不能在B运行器填默认、吞错或向mapper同步取缺字段。

历史数字按已审raw更正：smoke8 summary共63次非成功，其中61次AttributeError、2次REPLICA_LAGGING；不是63次均同一异常。现有提案给了代码片段但未附完整可执行反例/输出，实施时先落原绑定install可复现脚本与原失败输出，不再开全量仿真。

旧流缺关键字段不得用False/空pending/空event冒充正常summary。不接受“旧流默认值+日志即兼容”。本轮采用明确wire schema新版本（现值1→2），按既有UNKNOWN_SCHEMA/完整性错误与有界resync语义处理旧或不完整流；新端不得把旧v1宣称为READY兼容。升级时新writer提供完整FULL重建，旧writer持续提供v1则持续具名不可用，不保证混版本互通。不得无限resync或扩大历史保留。

## 2. 修改范围

允许cargo_query_snapshot_wire.py三字段编码/深不可变解码；cargo_query_wire.py schema版本及所需验证；仅为schema传播/具名拒绝必需的cargo_query_export.py、cargo_query_replica.py最小变动；对应专用tests及最近协议说明。优先不修改后两文件，如必要在提交说明列出直接依赖。不改occupancy_map_store、融合/射线/索引算法、summary语义、服务IDL、CURRENT/pinned/lease、计时门、launch默认和控制/规划逻辑。

pending_changes不是任意JSON列表：核对owner的冻结类型与字段，按相同语义重建不可变对象；原始输入改变不能污染已解码快照。三个字段全部从同一快照编码，metadata-only更新也必须传播到相同目标identity；不能仅块脏集变化时才更新。

在 `/tmp/elfin-occ-summary-wire-fix-g1` 新worktree、`agent/eng/occ-summary-wire-fix-g1` 从base建立。不要在已绑定B树改产品或覆盖旧install。scripts重放/测试可专用，产品改动与B脚手架提交分离。

## 3. 验收：有限定向门

| 门 | 输入/次数 | 必须观察 |
|---|---|---|
| 旧版本反例 | 原绑定install、writer存活、FULL一代、独立reader一次 | owner成功、副本具名AttributeError原输出保留；无B脚本补丁 |
| 三字段往返 | confirmed false/true × pending空/至少2条 × event空/非空，共8例；FULL/DELTA各一遍=16例 | owner与replica summary所有业务字段/完整身份逐字段相同；elapsed_ns等运行耗时单独记录不要求数值相等 |
| 原真实路径修复 | 新install、真实store→publisher→独立replica→facade summary，初始及至少3次元数据转换 | 每代成功且结果等于该代owner；pending清空/事件改变不得保留上代值；不是只测codec |
| typed查询 | 新install真实GetCargoMapSummaryV2服务一次完整请求/每上述实际代；无运动ROS允许 | typed身份和业务字段与facade一致，零未知异常；必要字段类型正确，不只mock返回 |
| 缺字段/损坏 | 三字段分别删除、分别错误类型=6例，加checksum坏1例 | 无半代READY；具名拒绝，旧pinned有效身份不漂移；不可填默认成功 |
| 版本边界 | 新读旧v1、新v2缺键、未知版本、v2新FULL恢复各1例 | 前三类拒绝或有界resync；新FULL恢复后业务全等；旧v1不伪装兼容 |
| 不可变性 | 三字段嵌套输入修改、重复读、前后两代各1例 | 已发布/持有代内容和digest稳定 |
| 邻接回归 | 既有wire/replica/export/facade/summary/surface/distance/catch-up相关测试完整文件清单 | 零新增失败；新增合同无skip；旧schema fixture须按用途保留负对照，不只改expected绿测 |

全部为功能门，不作时延PASS宣称，不重跑A7/A8/A9全矩阵。B后续统一测新增payload的传播/资源成本，不以修复字段少而豁免。fixture组合不可达则用明确标记的codec DTO单测，真实路径必须实际可达；缺真实路径不得用DTO替代全链。

每个门都是有限确定性样本的AND，错误身份/半代/默认伪造允许数0；负对照必须拒绝。它证明协议覆盖，不外推长期可靠率。测试应行为化：真实函数和端口执行，不做源码字符串存在断言。

## 4. 证据与版本绑定

证据目录 `docs/status/evidence/occ_owner_b1/occ_summary_wire_fix_20261028/`。首次评分前记录实际pytest精确文件清单/命令（运行`pytest --collect-only`核对）；真实无运动路径命令、环境、source/install/IDL hash、dirty状态必须留manifest。每例输入/期望/实际、schema/完整identity/原始错误/对象hash、测试退出码留存；失败保存全部本例对象和请求再停，不盲重试。fixture总量有限，单例<=16MiB、总<=512MiB；超额停并具名说明，不删失败。无需新增RGBD全量捕获。

完成提交产品SHA+tests+报告，独立新install执行v2逐文件manifest与IDL对照（IDL应未变）。由reviews核验后更新B产品/install绑定；此前B0da4086b仅历史身份，不能用于summary组评分。不要静默cherry-pick到B，记录修复SHA和B脚本分离的版本关系。不全重开A，不宣称B通过，不修改master生产默认。

## 5. 明确不纳入本单

writer先close后reader停RESYNC_REQUIRED是独立生命周期观察，不在本次三字段修复范围；记录实际state/reason/head/owner时刻即可，未证实前不改health/close/resync。

commit时刻不得换成covering offer：covering可能是后续合批代，将低估前序提交等待。B需真实每笔提交线性化时刻与同笔完整identity；offer可以另记export_enqueue_t。若只能取得同笔提交前后夹逼，报告区间/上界并标正式指标not_evaluated，不能偷换测量口径。本单不授权为此修改store或新增计时产品hook。
