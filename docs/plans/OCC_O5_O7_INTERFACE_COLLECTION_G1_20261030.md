# O5–O7 接口取证计划：先形成全景，再决定修复

- owner: eng/cursor/grok-4.7
- parent: OCC-QUERY-INDEX-20261025
- subtask: OCTOMAP-INTERFACE-DATA-COLLECTION
- generation: 1
- product_revision: 4c8f1862ec357cb01616f4b04be636227461cf44
- tool_base_revision: 0142027624e5014c997bba0a31c6c14a53a7fdac
- depends_on: OCTOMAP-O6-O7-EVALUATION generation 2 done/pass（工程交付，不是产品PASS）
- status: dispatch-ready；plan_revision 由派工线程绑定本文件提交

## 1. 授权与边界

用户要求明确收集数据、制定计划并派发 Cursor；本代只实现取证工具、行为自测、非计分实验和分析，不决定/实施产品修复，不重新认证 O5/O6/O7，不改原成绩、门限、负载语义或恢复语义，不开启双后端公平比较、生产迁移或完整装箱验收。其它已授权任务保持各自状态，不擅自暂停/恢复；重负载统一排队。

复用 `/tmp/elfin-occ-octomap-perf-cursor-g1`，不新建 git worktree。先保存 git status、产品源 diff、编译选项和已加载二进制/库路径与SHA256；不得覆盖其它agent修改。固定产品 Release、OctoMap1.9.8、discretize=false，当前x86主机，不启动Gazebo、不接机械臂。记录governor/CPU拓扑/线程/亲和性/外部负载，不调整governor择优，不外推Orin。

允许修改测试、诊断适配器和文档；必要产品事件探针只在现有worktree下单独的诊断源码副本及独立build/install中应用，保存完整可审查patch和源身份。探针不得更改锁范围/顺序、队列/QoS、executor、缓存、参数或响应语义，不替换冻结产品install。所有插桩结果标 `not_scored`。优先复用现有 profiling 和 `sustained_run.py`/`surface_diag.py`，以单一职责模块扩展，不建无法复用的一次性benchmark。

总实施预算6小时；其中新增实验最多120分钟（进程实际运行含预演/扰动实验，等锁不计）。持 `/tmp/elfin-occ-backend-perf.lock` 后串行实验，释放锁后分析。单RPC超时5秒，启动30秒，排空30秒，单进程上限10分钟。预算耗尽保留逐项not_evaluated，不缩样、不自动增负载或择优重试。启动/取证故障只允许修工具后重新预演，旧目录不可覆盖；评分历史不受影响。

## 2. 要回答的问题与证据判据

| ID | 问题 | 必收证据 | 可形成的结论 |
|---|---|---|---|
| Q1 | ROS查询为什么比进程内查询慢？ | 服务/registry/store锁等待、surface hit/build、响应数组组装、线程CPU、客户端全程、输入重叠 | 分段解释慢请求；不能把进程CPU或未分段残差叫作积分/序列化 |
| Q2 | 77帧无回执的同类缺口在哪？ | publish前后、subscription callback进入、epoch/TF/convert、enqueue结果、worker开始/结束、receipt发布/接收、完整health与DDS可用事件 | 逐ID定位最后已证边界；未进入callback只能记“publish至callback之间”，没有DDS事件不能断言DDS丢弃 |
| Q3 | 恢复首查STALE是何种时序？ | 首个恢复帧接纳/完成、查询linearization、选中snapshot stamp/age/max_age/allow_stale、首查结果 | 区分旧数据按约拒绝与已具新鲜数据仍错误拒绝；不改原FAIL |
| Q4 | O6失败组核心fusion成本在哪？ | 原始输入前缀、off/stages配对、各融合阶段、changed keys/blocks/概率摘要 | 指向真实慢段，不能移用其它fixture的归因 |
| Q5 | 现有“通过项”证明是否完整？ | 正式O5覆盖映射、query内容/身份/分页/release、发送调度、health端点、缓存分类、资源归属边界 | 明确可由旧证据补证、必须新增观测、接口不可观测三类 |

“定位”要求相同run/request/input关联上的边界事件支持，不以相关曲线宣布因果。每个问题最终为 `localized / partially_localized / not_evaluated` 并附证据；机制假说须列至少一个替代解释与区分它的观测。没有触发异常不等于问题解决。

## 3. 事件数据契约（先实现、自测、冻结，再运行）

每条事件包含 `schema_version, run_id, phase, event_seq, event, mono_ns, pid, tid, thread_cpu_ns, request_id/chain_id/input_id, status`；不适用字段明确null。输入身份用sensor_id+observation_id+input_stamp_ns；记录payload SHA、frame、TF identity、geometry/config/product/tool SHA。查询含handle、epoch、map_revision、observed_revision、last_integrated_stamp、geometry/config/clock_domain。

所有阶段时延使用同主机 `CLOCK_MONOTONIC`/可验证同域steady clock，记录校准方式及采样误差；ROS stamp只用于数据身份/新鲜度，不与单调时钟相减。请求关联若无IDL字段，用客户端请求序号、服务请求header sequence/GUID或可审计sidecar映射；不可凭最近时间关联，禁止为取证修改公共IDL。线程CPU必须取实际服务/worker线程，Python调用线程与进程CPU另列。

必须采集以下边界：

1. **发送**：planned deadline、publish_begin/end、unique ID、实际输入hash；publish返回不标业务accepted。查询记录planned/actual start、各RPC send/result/timeout、chain end；迟发、未发、迟到响应全部保留。
2. **入口**：callback enter/exit；TF lookup begin/end及status；epoch读取前/后；点云解码/convert begin/end；enqueue begin/result。明确实测调用顺序，不为探针重排C++参数求值。记录收到raw/Z有效点数（可复用receipt，避免双重过滤）。
3. **writer**：enqueue时间、worker dequeue、integrate start/end、store lock attempt/acquired/release、receipt status/身份/所有原计数；callback与worker completion关联。探针记录锁release不得移动原释放点；无法安全插桩时说明边界上限。
4. **回执/health**：receipt publish begin/end和客户端接收原文；health完整原文及接收时间（waiting/in_flight/submitted/queue_rejected/adapter_rejected/integrated/rejected/duplicates/age等），含阶段两端至少各一条；发布异常、解析错误、trace dropped counter独立留存。queue_rejected和adapter_rejected可能交叠，不能求和为两次拒绝。
5. **查询服务**：callback enter/exit、repository mutex attempt/acquired、store mutex attempt/acquired（区分snapshot/diagnostics）、handle get完成、snapshot选择/age判定；ROI/Surface begin/end；surface cache身份/hit/build、输出列数；响应数组组装begin/end。无改行为方式取不到的middleware序列化/客户端反序列化分别unknown，不将残差当成它们的精确值。
6. **内容与生命周期**：完整请求参数、各响应status/identity/数组长度/cursor/done/内容hash；release send/result/status；超时late response与handle最终状态。ROI和Surface内容hash规范固定（顺序、浮点位表示、NaN/无值规则），在计时区间之外计算并报告成本；固定snapshot重复读摘要必须一致，跨版本不要求相同。
7. **资源**：进程CPU/RSS每秒；各阶段峰值、线程CPU；已存在的tree估计、blocks、active handles/revisions、cache identity。无可靠共享归属时read-view/cache bytes保持unknown，不用RSS/块数伪造512MiB结论。记录instrumentation buffers本身开销。

## 4. 固定实验矩阵与执行顺序

先读历史，不重复有效评分。顺序 A → 工具自测/预演 → P → B → C → D → E。所有输入/配置/TF/hash及完整展开的COMMANDS.sh在首个正式诊断前冻结；工具修正则新工具版本、新run并标受影响旧run无效。

### A：历史覆盖与失败复算，无新负载

读取原始O5合同（`d7af0b70fc92c30dcde50e5386f935d36447e1f8:docs/plans/OCC_OCTOMAP_OBSERVER_G1_20261026.md` O5–O9及计时定义）、ApplyBatch代表证据、144组ledger、9窗和过载原始ledger、closeout_g2。

交付接口/指标映射表：每项实际调用、计时起止、负载与输入多样性、样本数、内容检查、首建/变更重建/同revision重复分类、是否正式覆盖。复算9窗计划/实际发送、查询调度、health接收间隔及端点；缺score边界不可猜测，应列可证上下界和缺失字段。核对O5物理ROI与实际16³范围，判定差异而非自行重新定义。对77个历史ID逐个保留unknown，不用新实验填写旧终态。历史报告和源码均记录SHA256。

### P：取证有效性与扰动门

先做非计分短预演：每个必需事件路径至少发生一次并完整join；trace丢事件=0，时间顺序可解释，hash和进程清理通过，否则不进入下列实验。

四个代表操作：30k integrate、snapshot acquire、4096cell ROI、全ULD Surface，各固定1cm倒角0箱输入。每项10对ON/OFF，以AB/BA各5对平衡顺序；Surface每对均相同fresh-process状态与首次/重复次序，不混入不同缓存状态。两版本同一源码/配置，唯一差别探针开关；保存内容摘要一致性。P95 nearest rank，n=10取第10；逐项ON/OFF P95比≤1.10作为低扰动参考，内容/身份必须一致、trace无丢失。此处10对是诊断资格，不代替O9正式30样本门。

若>1.10，仍可报告事件关系与高扰动限制，禁止把插桩绝对值或它与历史差值作为性能结论；有明显事件丢失/行为差异则停止依赖实验修取证，保留失败。不得通过多次重试择优通过扰动门。

### B：查询共享锁与缓存，四臂

1cm倒角、30k、outside=.5/cross=0，0箱和20箱，复用closeout同一可校验点云/TF。每臂新进程、3帧输入预热；每链同一handle acquire→ROI→Surface→release。

每组D0无并发写入、D1独立2Hz写入；均3条预热查询+30条测量查询，1Hz计划发起。顺序0箱D0/D1、20箱D1/D0；不得择优重复。max_age=120000ms只为稳定snapshot诊断，明确不同于历史2000ms，结果全部not_scored。D1发送覆盖33秒（66帧），结束后30秒排空，所有66帧留终态或unknown。

每条分解registry/store等待、内部读、响应组装，记录实际服务线程CPU与同时writer活动区间、cache identity/hit。分首次、revision改变、同revision重复；各类不足30标描述性结果，不声称该类P95资格。计时总段与分段必须解释重叠/残差，不加重叠CPU成墙钟。验证重复内容与release终态。

### C：入口到回执全链，正常与过载各一次

1cm倒角20箱、同固定30k点集、outside=.5/cross=0、同时间戳TF lookup可用；每窗新进程，3帧预热并确认worker与客户端账本均清空。先正常2Hz60秒120帧、1Hz60查询，排空30秒。再新进程10Hz60秒600帧、1Hz60查询，停止写入排空最多30秒，接着原样2Hz30秒60帧/1Hz30查询恢复，再排空30秒。max_age=2000ms，恢复第一写与第一查同t=0，保留原行为。

所有阶段均记录绝对monotonic起止，不以末次publish代替窗口结束。输入线程独立，不等待上一帧。查询必须逐条记录计划时刻，若无法准时发起如实记录lateness，不用数量推算节拍。

各截止点输出互斥账：计划未发送、publish完成尚无callback、callback已进入尚未接纳、明确拒绝、已接纳处理中、完成但无客户端回执、客户端已见终态；unknown具名。不把最后一个stage观测当成后续失败根因。强过载覆盖证据为明确QUEUE_FULL/队列中至少一条在另一帧integrate开始至结束期间持续等待/直接观测middleware积压或丢弃事件；微小queue_ms和in_flight=1不算。未触发即not_evaluated，不升频重试。

### D：恢复新鲜度对照，不改原恢复评分

固定20箱同输入，fresh process 3帧预热并排空，停止写入30秒。两臂各3次，AB/BA/AB次序：R0同t=0发首个恢复帧和acquire；R1等首个恢复输入INTEGRATED回执后立即acquire。两臂都max_age=2000/allow_stale=false。每次首查后继续2Hz写入与1Hz查询10秒，记录20帧/10链（首查是第1链，不额外加一次）。R1以回执为query origin，输入仍按原发送origin固定10秒，结束后排空。

至少每臂3条首查全事件：选中身份/age计算操作数、writer完成、服务线性化点。R0出现STALE且选中的stamp确实超龄是合法拒绝证据；R1仍STALE须检查所选数据是否新鲜后再判语义。R0未出现STALE不能宣布历史问题消失。两臂不是原O7验收，原恢复FAIL不变。

### E：O6失败组核心阶段归因，一组一次

复用 `run01/o6/groups/r01_b00_cuboid_c30000_o50_x50/warm` 的完整3+30帧输入及顺序，先校验每帧hash。Off与现有Stages模式各一个新进程累计窗口（顺序Off→Stages），保持无ROS并发。每个测量帧按input hash/index配对，保留全部状态和原外部计时、preprocessing/preparation/fusion、ApplyBatch/WriteLoop/InnerUpdate/Prune/BlockBatch/Finalization等已实现阶段、changed/occupancy_changed/rebuilt blocks、raw/accepted点数、内容摘要。

分别报告30样本nearest-rank P95与max及Stages/Off P95比；比>1.10不能用Stages绝对值判门，仅保留受扰说明和可验证结构。分段按单样本核算，不相加各自中位数；若现有Stages无法再细分，只标unknown，不扩展算法优化。历史517.297808ms保持，新增Off不能覆盖它。

## 5. 工具行为自测与停止规则

行为测试必须有正负对照：事件乱序/缺失/重复ID、publish不等于accepted、QUEUE_FULL双计数去重、completion与receipt分别缺失、health payload解析/端点缺失、query计划迟发、late response/release失败、相同identity内容不一致、分页缺失、stale边界age==max通过和age>max拒绝、ROS与mono时钟分离、线程CPU与进程CPU分离、缓存状态不同不得混池、无真实积压不能通过过载覆盖、统计n=29不冒充30、超时清理及证据flush。禁止只断言源码字符串。

计数守恒和跨事件关联要求100%；有缺失必须在unknown/dropped表中逐项显现，不允许静默剔除。身份/内容语义错误、崩溃、死锁或取证丢失先保存证据并停止依赖运行，不能盲目续跑。单纯性能慢允许完成本窗。每例输出子进程PID、退出码、清理结果，冻结进程列表，禁止误杀其它agent。

## 6. 证据粒度与交付

新根目录 `docs/status/evidence/occ_o5_o7_interface_collection_20261030/runNN/`，不覆盖audit或closeout目录。每例详细消息≤512MiB、全代≤8GiB；空间不足先落manifest并具名停止。完整轻量事件与输入生成manifest持续保存；异常前后各5秒详细请求/响应/TF/health/receipt ring buffer落盘。所有重建输入保留校验hash与生成器/seed/参数，无法完整重放明确capture_complete=false。

| 边界 | T1必留 | T2触发及内容 | 重放/检查 |
|---|---|---|---|
| 发送/TF/入口 | ID/hash、计划/实际时刻、callback/TF/convert、计数 | 无callback/拒绝/超时：前后5秒点云生成描述、TF原文、QoS及中间件事件 | COMMANDS.sh对应case的generate/replay argv |
| enqueue/worker/receipt | 接纳与拒绝、锁/队列事件、完成、publish/receive | 未完成/缺回执/重复：该ID上游输入前缀和所有可得阶段、health | 按ID导出timeline与deadline账本的完整argv |
| snapshot/查询/release | chain与RPC ID、handle身份、锁/CPU/内容摘要/分页/结果 | 慢、STALE、内容不符、释放失败：前后5秒请求响应和选中snapshot输入前缀 | 固定handle读取或按前缀重建的argv，不能重用过期handle声称可重放 |
| O6融合 | 原始hash、阶段计数/时延、内容摘要 | 超限/差异：失败帧及之前全部必要输入、stdout/stderr、profile | 原manifest和新out的完整argv |
| capture/cleanup | 序号、dropped、磁盘、PID/返回码 | 丢事件/超时/空间不足：保留现有日志和缺口表 | validate/cleanup check argv |

交付文件：`COMMANDS.sh`（先冻结真实路径与完整argv，不留占位符）、`MANIFEST.json`（产品/工具/诊断patch/build/install/配置/输入/环境身份）、`EVENT_SCHEMA.json`、`events.jsonl`、`messages`触发包、`TRACE_VALIDATION.json`、`PERTURBATION.json`、`INPUT_ACCOUNTING.json`（阶段截止快照）、`QUERY_SEGMENTS.json`、`RECOVERY_ORDER.json`、`O6_ATTRIBUTION.json`、`O5_O7_COVERAGE.md`、`UNKNOWN_ITEMS.md`、`FINAL_REPORT.md`。工具--help、自测、预演、每臂、复算的确切命令都进入COMMANDS.sh。

FINAL_REPORT按Q1–Q5逐项：实测→源码机制→替代解释→尚缺观测；明确源样本数/分位数/输入边界/复现步骤。只提出“可供决策的候选修复范围与所需证据”，不得把候选变成实施承诺。分别列O5/O6/O7历史验收状态与本代取证完成状态。

## 7. 派工与任务关闭

| ID | Owner | Depends | Base | Generation/plan | Scope | Acceptance | Tests/commit |
|---|---|---|---|---|---|---|---|
| OCTOMAP-INTERFACE-DATA-COLLECTION | cursor/grok-4.7 | OCTOMAP-O6-O7-EVALUATION gen2 | 工具0142027624e5014c997bba0a31c6c14a53a7fdac，产品另固定4c8f1862 | 1；派工线程精确plan_revision | A–E取证及判读，无产品修复 | 全必需产物、关联验证、逐项证据/unknown与历史状态分离 | 第5节行为测试、P扰动门、工具/探针独立提交、工程角色记录与Result |

使用agent_start领取；诊断工具和可撤销探针提交并记录工具SHA，评分产品SHA不得冒用工具HEAD。关闭用agent_complete。Result pass仅表示本代取证按冻结矩阵完成、必需事件可核验且证据局限如实报告，不要求产品通过或故障一定复现；任何未执行必需臂/关键探针无法采集或关联验证不通过，Result blocked并列缺项，不能仅交一份unknown报告就宣称完成。可选middleware序列化和资源归属unknown不单独阻塞本取证任务。有限样本仅证明该有限窗口的观测，不估计长期成功率。

## Pointers

- `docs/agents/reviews/2026-10-30_1534_occ-o5-o7-interface-whole-picture.md`
- `docs/status/evidence/occ_o5_o7_interface_audit_20261030/run01/AUDIT.json`
- `docs/agents/discuss/2026-10-30_1355_octomap-acceptance-closeout-g2.md`
- `docs/status/evidence/occ_o6_o7_20261029/closeout_g2/FINAL_REPORT.md`
- `docs/status/evidence/occ_owner_b1/occ_apply_evaluation_20261029/run01/FINAL_REPORT.md`
- `.cursor/rules/debug-evidence.mdc`
