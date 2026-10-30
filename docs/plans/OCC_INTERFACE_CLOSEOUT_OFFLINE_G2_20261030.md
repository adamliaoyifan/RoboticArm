# OctoMap接口取证离线补齐与关闭裁定 gen2

- owner: eng/cursor/grok-4.7
- parent: OCC-QUERY-INDEX-20261025
- subtask: OCTOMAP-INTERFACE-DATA-COLLECTION
- generation: 2
- tool_base_revision: 57d8c0bfe23fee54920c89c06a2b6bdcb97de976
- product_revision: 4c8f1862ec357cb01616f4b04be636227461cf44
- supersedes: gen1后续交付与关闭裁定；保留旧Claim/Result和全部历史证据
- prior_collection_plan: de5f214cca1e587e3934b3d438bd4d72eb8b6381
- resolution_target_revision: 9b7d465b01183ac1f5aa189c4c970c8c3659eca5

## 1. 目标与授权边界

用户要求针对关闭审查缺口给出执行计划并派发Cursor。本代修复离线工具与交付：让现有诊断结论可追溯、可复算，明确原取证合同每条是否满足，不能再用工程done/pass代替证据完整。

不启动observer/ROS服务、积分/查询benchmark、Gazebo或硬件；不重跑1cm、不启动2cm、不修产品或探针行为、不改历史成绩/门限。不把不可恢复的数据补造为0或PASS。可执行静态检查、解析、纯离线判官、合成行为测试，以及冻结二进制的`--help`/非法参数验证（不得进入工作负载）。重放脚本仅生成和验证，不授权执行测量。

复用 `/tmp/elfin-occ-octomap-perf-cursor-g1`；禁止新worktree、覆盖未提交修改或操作Claude任务。先记录HEAD/dirty及产品src/include/config与固定SHA差异；只改测试工具、tests、docs，独立提交工具SHA。gen1已完成，无需等待不存在的运行进程；实际若发现本任务遗留进程，记录PID/命令/owner并有界处理自己的进程，不推断或杀其它任务。

实施预算3小时，新增证据≤1GiB；历史大数据只读引用/哈希不复制。单离线子进程最长15分钟；重IO/CPU步骤使用同一perf锁，与其它计时战役串行，锁等待不计预算。到期交付已完成检查与明确阻塞，不启动替代实验。

## 2. 输入与输出身份

只读输入：

- gen1根 `docs/status/evidence/occ_o5_o7_interface_collection_20261030/run01/`（所有spec、server/client events、ledger、health、queries、O6/扰动原始文件及报告）。
- 历史正常/过载根 `docs/status/evidence/occ_o6_o7_20261029/run03/o7/`、`closeout_g2/`、原O6失败组warm 3+30。
- 原计划de5f214c与本轮审查 `docs/agents/reviews/2026-10-30_1737_occ-interface-closeout-audit.md`、`docs/status/evidence/occ_interface_close_review_20261030/run01/FINDINGS.json`。
- gen1工具57d8c0bf及诊断patch、独立build/install、已有manifest引用的输入配置。只在有历史记录支持时声明“运行时加载身份”。

新根 `docs/status/evidence/occ_o5_o7_interface_closeout_g2_20261030/run01/`；存在则新runNN，拒绝覆盖。开始与结束都核对所有读取的历史文件SHA256，证明无变动。当前文件hash只能证明当前工件，不可反推缺失的运行时hash。若只有路径无运行时摘要，标identity_unproven并降级相关结论。

## 3. 执行顺序与逐项验收

### S0：完整性目录与合同检查表

从原计划第3–7节逐条建`REQUIREMENT_LEDGER.json`，每项含requirement_id、原文/位置、mandatory或optional、所需字段/样本、原证据路径/hash、检查命令、结果与原因。结果限定 `satisfied / recoverable_offline / unrecoverable_missing / not_applicable`；原可选middleware序列化/资源归属unknown与必需RSS时间线等分开，不能因名字相近混为可选。

目录核对A、P、四个B臂、C正常/过载含恢复、六次D、E Off/Stages及预演的实际输入/查询/事件/退出状态。只证明产物存在不等于内容通过。建立`ARTIFACT_INVENTORY.json`，列size/hash/来源/时间身份及capture_complete、replay_possible、limitations三字段。

### S1：修复重放命令生成与运行环境记录

修工具command model：argv数组与env字典分开保存；通过正确shell quoting生成脚本，不用JSON字符串拼shell。探针ON/OFF使用`ULD_OCC_INTERFACE_TRACE=1/0`，路径和run ID使用对应env；禁止把`trace=on`当额外argv。保留原COMMANDS.sh只读，新文件明确是修正后的replay recipe而非当时实际执行的命令日志。

两类入口分开：`OFFLINE_COMMANDS.sh`可直接执行复算/校验；`REPLAY_COMMANDS.sh`默认只显示，不执行负载，显式标历史1cm重放不在当前授权范围。重放输出必须传新的绝对证据根，拒绝原run根及其子目录、路径逃逸、已有文件覆盖；原输入路径可只读复用。记录ROS/diagnostic setup、LD_LIBRARY_PATH/PYTHONPATH/ROS_DOMAIN_ID等必要环境、cwd与实际二进制路径/hash，缺项列未验证。不得把当前环境猜测写成当时环境事实。

补齐真实`--help`、工具自测、manifest/schema检查、逐阶段核账、query/health复算、报告生成完整argv。不存在的离线CLI先实现和自测再冻结，不把计划中的参数写成已支持。

验收：每种生成命令用假执行器捕获argv/env/cwd/output验证（包含空格、引号、美元符号、反引号路径）；ON/OFF参数集合都正确，无shell展开副作用。可用真实probe --help和与先前缺陷同型的非法参数检查证明parser边界，但不进行积分。运行状态分别记`argv_env_validated`、`help_validated`、`workload_not_executed`，不能因此声称runtime replay passed。

### S2：按计划集、阶段截止重建输入账

基集必须来自冻结spec计划项而非ledger.sent。用phase+index/计划时刻关联计划记录与实际ID；不能凭最近时间匹配。缺少稳定映射时报告ambiguous，不自动补身份。为warmup、normal、overload、recovery独立输出planned/sent/callback/accepted/rejected/completed/receipt及重复/unknown。

在发送截止、排空截止、恢复截止分别重建互斥状态：未发送、已publish未进callback、callback已进尚未接纳、明确拒绝、已接纳未完成、完成未见客户端回执、客户端已见成功终态；异常/重复单独列但不得重复计入总数。明确拒绝即使有回执仍只属于拒绝桶。按sensor/ID和事件状态去重，QUEUE_FULL两计数交叠不求和。

截止点资格分exact/bounded/unavailable：实际事件/冻结绝对origin可证才用exact；仅凭planned相对时间、首个发送或下一phase开始不得编出精确origin/drain结束。bounded时逐ID给最早/最晚状态及上下界，不在区间中随便选一个点；无依据列unavailable。最终账可成立不代表每个截止账成立。若当前C窗口没有满60s明确结束，给实际活动跨度与边界可证范围，不能以120帧推断满窗。

回放源事件核对报告中的360成功/13拒绝/227入口缺口，只报告实算结果，不能硬编码这些数。分别保留gen1新增227与历史77，禁止互相填补。health原文计数作为交叉检查，说明异步采样/缓存滞后，只有能证同一边界才能作严格等式。

交付`PHASE_BOUNDARIES.json`、`INPUT_ACCOUNTING_BY_DEADLINE.json`、`ID_GAPS.json`。验收为计划集无静默漏项、互斥账覆盖100%、每个歧义/缺失具名、每项等式有来源；不可恢复截止点必须阻止原严格合同coverage PASS，而不是阻止本代离线报告生成。

### S3：查询内容、生命周期、关联与时钟校验

逐链核对计划/实际send/result/timeout及late、acquire/ROI/Surface/release状态；选中identity与两响应identity完全相同，检查handle关联、epoch/revision/stamp/config/geometry/clock。校验ROI/Surface长度、cursor/done与实际请求范围一致；数组只存hash而原数组缺失时明确无法复核元素级内容，不能将hash存在当内容正确。

对相同完整snapshot identity、相同查询参数的重复内容hash比较；不得把不同observed_revision的合法新身份误判，亦不得只按map_revision忽略其他identity。release未返回/失败不能记链成功；超时后迟到结果/handle回收不可证明时具名unknown。列每类实际n，n<30不得报告合格P95。

独立核对server/client关联：对handle链和唯一serial interval映射说明成立前提、callback enter/exit配对、PID/TID、相同时间域；存在多个候选/缺exit必须拒绝，不能直接取下一条exit补配对。TRACE_VALIDATION的schema/sequence检查与语义关联检查分别输出。

store/repository/read/array-fill段按嵌套关系计算，确认没有把overlap重复相加；对2个慢Surface逐条列事件来源、服务线程CPU、writer重叠及callback residual。核查`store_lock_release`等探针标记与真实RAII释放的先后，必须说明是精确释放还是释放前标记，禁止对标记间隔声称超出精度的因果。内容hash在计时外并不等于对下一请求零扰动，列其位置与限制。

交付`QUERY_VALIDATION.json`、`TRACE_JOIN_VALIDATION.json`、`QUERY_SEGMENTS_RECHECK.json`和Q1结论支持范围。所有缺失/错误进入分母，不删除失败请求。

### S4：窗口、health、资源与捕获完整性

对gen1所有窗及历史9个正常窗逐窗计算可得的计划/实际发送和查询lateness、health相邻接收gap、首尾覆盖。精确score_start/end缺失时只给可证明区间与未覆盖边界；不能用首末health反推计分窗。原门仍为health最大gap严格<1s，阈值相等失败；不跨窗合并。

列出已有进程CPU/线程CPU/RSS/峰值/1Hz时间线。不得从两次查询间CPU采样生成1Hz序列、用当前/proc补历史RSS、用RSS代替512MiB视图归属。若未采集则`unrecoverable_missing`，如实保留原计划不满足项。

逐输入/config/geometry/TF/generator/patch/build/install工件重建来源账：运行时既存hash可比对、当前可hash但历史加载不明、路径/工件丢失分三类。诊断二进制与patch的可复建关系需说明，不能只有两个文件名就声称完整身份冻结。

核对详细失败消息、前后5秒ring、完整轻量时间线、重建所需输入前缀、stdout/stderr、进程退出与清理记录。能由现有事件导出timeline不是原始响应数组的替代品；缺原文/缺退出检查不能事后编造。交付`WINDOW_HEALTH_RECHECK.json`、`RESOURCE_COVERAGE.json`、`IDENTITY_PROVENANCE.json`、`CAPTURE_COMPLETENESS.json`。

### S5：O6/恢复/扰动复算与最终裁定

只读复算已有30条Off/Stages按input hash/index配对、nearest-rank P95、逐样本ApplyBatch/write-loop占比和stage闭合。确认“主导”不误写成相对旧产品退化；输入不能匹配则拒绝配对。历史517.297808ms与新诊断值分栏，绝不改判。

复核六次R0/R1首查的选中age/2秒门/服务linearization与writer完成；如果仅有age没有now或stamp，则明确只能核age判断，不能声称完整原始操作数证据。三次R0、三次R1分别列，不合并成功率。扰动P每项10对、AB/BA各5顺序及内容摘要复核；Acquire1.104、repeat5.66仍受限，不择优删除。

最后生成旧计划逐条完成表及 `CLOSEOUT_DECISION.json`：

- `offline_supplement_delivery`: pass/blocked（本代工具、检查和完整报告是否交付）。
- `original_collection_contract`: satisfied/not_satisfied（所有原必需项必须有证据才可satisfied；已知不可恢复缺失时为not_satisfied）。
- `diagnostic_findings`: 每项supported/qualified/unsupported及证据/限制。
- `product_acceptance`: not_pass；分列历史O5/O6/O7和新2cm目标状态，不能用目标变化抹掉证据缺口。
- `unrecoverable_items`: 所需数据、为何不能离线恢复、影响哪项结论、是否只需在未来2cm计划中采集；不擅自执行。

## 4. 行为自测门与执行纪律

每类至少一正一负对照：命令argv/env与shell特殊字符；旧证据覆盖/逃逸拒绝；planned但未sent；重复ID/重复receipt；QUEUE_FULL交叠；阶段边界exact/bounded/missing；仅最终账闭合不能掩盖截止pending；错identity、内容hash不一致、坏分页、release失败；callback多候选/缺exit；跨时钟相减拒绝；health阈值相等和缺端点；n29不足/n30取第29；当前文件hash不能冒充运行时hash；缺RSS/原文不能自动通过；原始证据只读与输出新根。

所有行为自测通过才冻结工具并运行正式离线复算；失败保存输入/expected/actual/stdout到新根，不重写历史。可修工具重跑离线检查，但每次输出新子目录并记录tool SHA，不择优隐藏错误。最终对工具提交做必要格式/语法检查与原相关行为tests回归；只测试受影响工具，不引入产品测试矩阵。

正式离线命令由owner实现CLI后冻结在OFFLINE_COMMANDS.sh，顺序S0→S1→自测→工具commit→S2–S5→判官。脚本默认输出新目录、遇缺失继续生成缺项而非静默略过，真正解析/关联不可信则blocked并保留输入。

## 5. 关闭与派发

| ID | Owner | Generation/base | Scope | Acceptance | Required evidence |
|---|---|---|---|---|---|
| OCTOMAP-INTERFACE-DATA-COLLECTION | cursor/grok-4.7 | 2 / 57d8c0bfe23fee54920c89c06a2b6bdcb97de976 | S0–S5离线工具修复、复算、证据裁定 | 全部规定检查均执行并留可核结果，未观测项明确而非伪造；不新增测量 | 工具commit、自测、身份/只读hash、所有表、FINAL_REPORT、角色记录、Result |

最终报告必须首先写“本代离线补齐完成状态”和“原取证合同是否满足”，明确两者不同。即使全部离线工作完成，若原必需RSS/原文等确实缺失，允许本代工程Result pass但必须显著写原取证not_satisfied；这是本代有限补齐任务的合同，不是对原要求的豁免。工具无法完成检查/原始输入无法解析且不能形成可靠账时本代blocked。

按agent_start领取gen2，agent_complete关闭。旧gen1 Result不删除，追加supersede/复核结论链接。无需为离线工具修复再次询问用户；需要新运行/产品修改/新2cm评分时只列后续候选，不自行执行。禁止自动触发公平比较或生产迁移。

## Pointers

- `docs/plans/OCC_O5_O7_INTERFACE_COLLECTION_G1_20261030.md`
- `docs/agents/reviews/2026-10-30_1737_occ-interface-closeout-audit.md`
- `docs/agents/discuss/2026-10-30_1553_occ-o5-o7-interface-collection-g1.md`
- `docs/plans/OCC_RESOLUTION_TARGET_2CM_20261030.md`
- `.cursor/rules/debug-evidence.mdc`
