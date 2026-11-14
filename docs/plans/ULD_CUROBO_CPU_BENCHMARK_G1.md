# ULD cuRobo / CPU benchmark G1 — 新建测试草案

- status: draft
- created_at: 2026-11-14T08:43:39+08:00
- owner: codex/gpt-6
- implementation_revision: b77091ad45323edd1d18824813155d76f8aebdc1
- dispatch_ready: no
- benchmark_result: not_evaluated

## 1. 来源、授权与范围

本文件是用户于 2026-11-14 授权“重建并发布新草案”后的新方案，**不是恢复的旧计划**。
已查询 13 个仍存在的已注册 worktree 的计划目录、ULD 研究文档及本文件路径的 Git
历史，未找到交接所称完整旧计划。检索清单见主工作区
`docs/status/evidence/uld_upper_layer_v1/curobo_cpu_benchmark_publication_g1/worktree_plan_search.json`。

已知意图是保留 GPU 适配、比较 CPU 与 cuRobo、保留 30 秒预算、采用 P95/P99
统计及可重放证据。本草案的**每请求预算解释、矩阵、样本量、具体门槛及统计规则均为
本次拟定**；发布草案不表示这些值已通过测量，也不授权本次执行性能测试。

本次仅发布文档并运行研究 CPU 单测；不安装运行时、不改产品代码、不启动 CUDA
smoke、Gazebo、控制器或性能评测。后续 benchmark 的实现、资格审查及计分属于新任务。
不认证物理放置、碰撞安全总体概率或生产节拍；GPU 输出仍须经过源模型与控制器认证。

## 2. 固定基线与环境

开发源为 `d4c88854384d1c849635636e2fc47d2e15e1b92d`，其独立 worktree 中
`research/uld_upper_layer/RESULTS.md` 的未提交删除必须保留。新分支
`research/uld-curobo-cpu-benchmark-g1` 从该源创建，rebase 到本地 master
`63dba44c691759a5295d53acf3ec3b822380d13f`；fetch 后的 origin/master
`4f7f7b2045a093a189fdaa72cb6854003ecb6ead` 是此 master 的祖先。
8 个开发补丁经 range-diff 均保持一致，rebase 后代码为上述 implementation_revision。

未来每轮须固定同一台主机、ROS Humble、同一源 URDF/mesh/scene、相同配置和输入。
记录 CPU 型号/核心/affinity/频率策略、RAM、OS/kernel、Python、NumPy/BLAS 线程、
OMPL/FCL、GPU 型号/VRAM/驱动/CUDA/PyTorch/cuRobo、外部仓库 revision 与 dirty
文件清单、环境包锁、资产 manifest/hash、源文件加载路径。固定
`OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1`，保留 CPU affinity
供两侧相同使用；不得未经记录切换核数、功率或 GPU 时钟策略。

CPU 侧为现有 `ompl` 自由关节路径；GPU 侧为 opt-in `curobo_fast`，复用
elfin_e2e 的 `PersistentPlannerSession`。两侧共享源 FCL、retiming、连续区间及控制器
插值认证；约束 Cartesian/接触/推动阶段仍走源实现。分别记录 CPU 主进程、GPU worker、
认证成本。不存在 GPU 成功时静默切回 CPU 的比较。

每轮仅一个 benchmark 进程。启动前检查现有 sim/bridge/其他 GPU 计算，资源繁忙则排队，
不得终止其他人的任务。无 Gazebo 的离线 benchmark 不占用或制造第二个仿真世界。
环境预检和 GPU runtime/RobotAssets 资格不足时记 `not_evaluated`，不模拟 GPU 成绩。

## 3. 输入身份与坐标契约

使用活动 `scene_tf_deploy_update.yaml`，禁止假定 world=base。
ULD 的 container 绝对内底/顶为 0.53/2.01 m，`inner_size[2]` 是高度 1.48 m；
fixture 的实际变换从其声明的场景读取，禁止抄写部署平移常量。
研究 snapshot、support-face、箱中心使用 container 绝对坐标；GPU 世界几何导出到
`elfin_base_link`。工具接触平面保留其具名工具帧。

每个 case 冻结：源 fixture、snapshot、request、所有 world object、payload size/mass、
attachment transform、start/goal 六关节弧度、contact mode、scene/URDF/mesh/配置 hash。
快照仅用于研究 fixture，不将理想真值接入生产感知。两侧输入必须字节一致或具有明确的
规范化 hash；不得各自重新选择更容易的起终点。released payload 必须留在碰撞世界中。

## 4. 拟定工作负载矩阵

尺寸沿用 `evaluate_prior.py:BASE_SIZES`，质量均 8 kg：

| size_id | 长 × 宽 × 高，m |
|---|---|
| base_550x400x250 | 0.550 × 0.400 × 0.250 |
| len_500x400x250 | 0.500 × 0.400 × 0.250 |
| len_600x400x250 | 0.600 × 0.400 × 0.250 |
| wid_550x350x250 | 0.550 × 0.350 × 0.250 |
| wid_550x450x250 | 0.550 × 0.450 × 0.250 |
| hgt_550x400x200 | 0.550 × 0.400 × 0.200 |
| hgt_550x400x300 | 0.550 × 0.400 × 0.300 |

| 层 | 冻结输入与分组 | 两侧唯一差异 | 每组计分量 |
|---|---|---|---|
| F：自由关节链 | base 尺寸；三个场景：原始空腔、冻结原 fixture 下层箱、同 fixture 加已释放箱；各含空工具与携箱两种合法起终点，共 6 组 | ompl / curobo_fast | 每侧 100 个 seed 配对 |
| E：完整首条可执行方案 | 七尺寸 × baseline/prior 两种调度，各尺寸同一冻结 request、起点和 prior index，共 14 组 | 同一调度内仅换自由关节 backend | 每侧 100 个 seed 配对 |
| N：负例与故障资格 | 非法 start、非法 goal、源模型不匹配、缺 runtime/assets、GPU 无效输出、worker 超时各一例 | 预期的明确失败类别 | 每类一次行为验证，不算性能样本 |

F 的每个起终点须有独立源模型区间证书证明存在可行参考路径；E 的几何/目标合法性须
在冻结前检查。不得在看见某 backend 失败后删组。若预检失败，整个组具名
`not_evaluated` 并保留原因，不能总 PASS；修复输入须新 manifest/hash。
F 的两种 attachment 是不同 case；携箱案例必须携带全部 payload 几何。

100 个 seed 固定为 0..99，同 seed 跨 backend 配对。每组 5 对预热，之后 100 对计分；
偶数 seed 顺序 CPU→GPU，奇数 GPU→CPU，全部串行。F 与 E 分开统计，不跨尺寸、场景、
调度或冷暖混池。总计 4,000 个计分请求、200 个预热请求；30 秒上限对应计分执行最多
约 33.34 小时，另加预热、冷启动和取证；这是上限估算，不是本次执行授权。

## 5. 拟定预算、计时边界与冷暖规则

30.000 秒解释为**每个计分请求的端到端墙钟上限**，不是整场 campaign 总预算，也不是
单次 CUDA kernel 时间。使用单调时钟纳秒。F 从已冻结输入的调用分发开始，到源区间
认证后的路径/拒绝终态；E 从请求分发开始，到首条完整且通过现有 retiming/控制器
插值认证的方案/拒绝终态。排队、IPC、world/payload update、IK/搜索、源认证、retiming
及同步结果回传均计入；保存最终日志的耗时另列。首次成功必须是真正通过这些门的方案。

E 固定 `search_budget_s: 30.0`、`solution_count_target: 1`、
`minimum_executable_solutions: 1`；F 固定 `path_attempt_budget_s: 30.0`。
E 中每个子阶段实际可用预算必须裁剪到整请求剩余预算，绝不每段重新获得 30 秒。
父进程在分发后 30.000 秒锁定 timeout 终态，迟到结果不能转成功；之后允许最多 5 秒
flush/回收，不计成完成时间。deadline watchdog 和证据 flush 的行为须先离线验证。

每 backend/层/场景各新 worker 测 3 次 cold（每次独立 reset），cold 预算 300 秒，单列
初始化与首次响应及失败；样本仅 3 不计算合格 cold P95/P99。cold 不是稳态门的豁免项，
报告中不得混入 warm。F 保留同一 FullArmScene/session 进行 warm 请求；E 现有 CLI
每请求重建场景/进程，测得的是已热文件缓存下的请求成本，必须包含其实际 worker 初始化。
除非后续真实实现并单独冻结 persistent E profile，否则不得将 F 的 session 复用收益代签 E。
每次 world/attachment 更新仍计时；不得预先计算待测 seed 的路径。

## 6. 本次拟定的验收门（尚无实测校准）

| 指标 | 拟定门槛 | 最小样本 / 判读 |
|---|---|---|
| F warm 首条源认证路径 | 每侧每组 P95 ≤ 1.000 s；P99 ≤ 3.000 s | 100 个终态，且至少 99 个源认证成功 |
| E 首条完整可执行方案 | 每侧每组 P95 ≤ 10.000 s；P99 ≤ 25.000 s | 100 个终态，且至少 99 个完整认证成功 |
| 所有计分请求 | 无迟到结果记成功；30.000 s 后终态为 timeout | 每个请求均有唯一终态和起止时刻 |
| 正确性 | 源关节/几何身份、端点、limits、FCL 区间及适用控制器证书全部有效 | 每条成功路径 100% 满足，无碰撞漏放 |
| GPU 集成 | 无隐式 fallback；原始 GPU 输出和源认证链可关联 | 缺真实 GPU/runtime/资产资格为 not_evaluated |
| 相对性能 | 同组 GPU/CPU P95 比值 ≤ 0.80，配对 bootstrap 单侧 95% 上界 < 1.00 | 每组至少 99 对共同认证成功，且两侧绝对门已通过 |
| 单位可靠性 | 成功率 ≥ 99/100，单侧 95% Wilson 下界 ≥ 0.95 | 每组每侧独立计算；100 样本只约束此固定工作负载 |
| 可观测性 | 100% 请求 trace/终态/输入身份齐全；失败 replay 必要输入齐全 | 缺失证据为 incomplete_evidence，不能 PASS |

这些绝对及相对门均是本次草案的工程目标，不是既有承诺；可能需要新一代计划校准，不能
在看过同轮结果后改门。CPU 绝对门不达标须如实报告 CPU 失败，不能因 GPU 快而抹除。
每组分别给 CPU、GPU、比较三个 verdict；campaign 总 PASS 是全部组/必需门的 AND，
不是要求所有 4,000 次连续成功。每组允许至多一个失败，单位最低目标为 95%，99/100
的观测要求提供额外约束。100/100 时 Wilson 下界约 0.974，99/100 时约 0.956；健康与
全失败系统会获得不同 verdict。跨组 AND 不宣称总体/长期成功率 ≥95% 或场景泛化。

## 7. 统计及缺样规则

所有请求进入分母；取消、超时、无方案、异常、身份错误都保留，不补样择优。性能门使用
排序第 `ceil(p*N)` 项的 nearest-rank P95/P99（N=100 时第 95/99 项），成功路径取
真实墙钟，任何非成功请求的“达成可执行路径时间”记正无穷。JSON 以 null 加
`unbounded: true` 编码，不写非法 Infinity/NaN。另报全部请求到终态的墙钟分布，避免
“快速失败”造成加速假象；成功子集分布只是辅助指标。

同时报告 N、状态计数、成功率与 Wilson 下界、P50（偶数中间两项均值）、P95/P99/max、
超时数、CPU wall/thread/process 时间、GPU event 时间（同步与 wall 分列）、进程树
RSS/PSS 和 GPU memory。资源每 100 ms 采样，称采样峰，不称瞬时峰。阶段耗时对齐父请求，
重叠 GPU/CPU 时间不能直接相加；未解释时间须显式列出。

相对比较以每组 seed 为配对单位，10000 次有放回重采样，固定统计 seed 20261114，
在同一重采样索引上重算 GPU/CPU P95 比；单侧上界取重采样比值的 nearest-rank P95。
仅共同成功对用于辅助配对统计，必须同时公布被排除的 seed 和两侧无条件分布；共同成功
少于 99 对不得通过比较门。时间非有限、≤0、缺字段或身份不匹配不能形成合格比值。
任何 mandatory 组少于 100 终态、warmup 未完成、资格未知或缺必要证据，判
`not_evaluated`；已有硬失败仍保留 FAIL，不被缺样改写。bootstrap 不建立跨场景置信保证。

## 8. 后续实施与精确执行程序

本次没有 benchmark harness。现有 `evaluate_prior.py` 锁定三个 seed、默认 1200 秒，
`curobo_smoke.py` 只有两次请求；二者均不能直接充当本草案的 100 样本比较工具。
`request_planner.py` 的内部计时不能替代本草案父进程的端到端 30 秒 deadline。
后续任务必须先实现独立 harness 的预算、会话生命周期、统计与取证；禁止为本次文档
发布伪造不存在的 CLI 或运行结果。以下是后续必须逐项实现的确定程序：

1. 从干净、冻结 revision 的独立 worktree 启动，保存 T0；固定两侧配置，仅 backend 与
   必需 runtime 路径不同。将 6 个 F case、14 个 E request、prior index、所有配置、
   seed 列表和输入 hash 写入不可变 manifest，实际路径必须全部存在并可读。
2. 在非计分 rehearsal 中验证 N 六类故障、deadline 终态、worker 死亡回收、T2 落盘和
   两侧真实加载路径；每次 rehearsal ≤300 秒墙钟，总启动 rehearsal ≤1800 秒。
   计分前失败不消耗 seed/预算、不改输入 hash，单列启动失败率。
3. 真实 GPU 资格复用现有 adapter：1095 球模型及完整 validated RobotAssets bundle、
   上游 pinned runtime、源 FK 两端 0.1 mm/0.01 deg 一致性、源区间/控制器认证全部
   具备；缺任一项停止 GPU 计分。不得用 CSV 或 mock owner 代替完整资产/设备证明。
4. 按 F 六组、E 七尺寸（表中顺序）各 baseline→prior 的确定顺序运行。每组 cold
   三次，然后五对 warmup，再 seed0..99 交替 AB/BA。F 使用
   `path_planning.joint_path(scene,start_q,goal_q,config,contact_tool)`，随后保留其源
   认证证据；E 使用下列现有 CLI，每请求独立输出目录，并由 harness 计时和监管。
5. 计分源码/输入/配置/资产/环境任一变化立即结束本轮；修复后新 run_id/hash，不覆盖旧轮。
   失败先 durable T2，再回收本任务进程；只重放诊断，不能替换失败 seed。资源污染单列，
   整组结果不用于合格计时，不挑出污染样本继续计算成功成绩。
6. 离线从原始 request JSONL 重算所有门、样本数与统计区间，生成逐组和 campaign verdict，
   报告 not_evaluated 项、缺失 artifact 和物理范围。本轮不安排连续成功补跑。

现有 E 入口的单请求命令模板（变量均须由冻结 manifest 提供；本次不执行）：

```bash
source /opt/ros/humble/setup.bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  ELFIN_E2E_ROOT="$e2e_root" /tmp/uld-upper-python/bin/python \
  research/uld_upper_layer/request_planner.py \
  --request "$request_json" --scene-config "$scene_yaml" \
  --config "$backend_yaml" --selection-policy "$selection_yaml" \
  --execution-policy "$execution_yaml" --seed "$seed" --evidence "$case_output"
```

在此命令之外的父 harness 才负责 30 秒全请求 deadline、T2 与进程回收。上述所需输入
目前尚未冻结，故 `dispatch_ready: no`；本文件发布不能被当作可立即启动的评测派工单。

## 9. 成功证据、失败 dump 与保留规则

后续 run 根仅在主工作区：
`docs/status/evidence/uld_upper_layer_v1/curobo_cpu_benchmark_g1/<run_id>/`。
每个 case 用 `layer/group/backend/seed` 唯一寻址，禁止在 satellite worktree 开第二份证据。
遵循 `.cursor/rules/debug-evidence.mdc`；dump 单位为 case × 阶段边界。

| 边界 | 常开 T1 / 触发 | T2 payload | 有界窗口 / 格式 | 预算与检查 |
|---|---|---|---|---|
| 输入→源 scene/IK | hash、frame、object count、阶段时刻；身份/端点失败 | lossless request/snapshot/config、source model hash、异常 | 当前 case 全部阶段摘要，之前最近一个请求摘要；JSON/JSONL | 每 case 64 MiB；按同 request 重放源合法性 |
| scene→GPU worker→返回 | session/world/attachment identity、IPC 时刻、reason；超时/设备/输出错误 | 实际导出 request、raw q/status、12 KiB log tail、worker 环境 | 当前请求分发到终态≤30秒，终态后≤5秒 flush；JSON+log | 每 case 64 MiB；同 scene/start/goal 调 joint_path，比较首差异 |
| 候选→FCL/retiming/控制器证书 | candidate ID、输入输出 hash、耗时、证书判定；拒绝/超门 | 原始与修正 path、limits、失败 interval/pair、retimed path、所有证书 | 当前失败候选及其直接上游输入/下游终态；JSON/NPZ | 每 case 128 MiB；同 path 复算区间/控制器插值 |
| 请求→统计/资源 | 唯一终态、wall/process/thread、RSS/PSS/GPU sample；缺样/超时延门 | 原始请求记录、配对 seed、资源触发窗、统计输入 | 前5秒/后1秒资源环形缓冲，或 case 起止截断；CSV/JSONL | 每 case 16 MiB；nearest-rank 与 bootstrap 离线重算 |

T0 每轮必须含 revision、dirty 文件清单、完整命令/依赖/硬件、case/hash/seed、开始结束
时间、各 verdict、artifact 清单及所有本任务进程回收状态。T1 每个计分 case 必留；T2
遇阈值失败、无方案、异常、crash、timeout 或 invariant violation 必留。共享输入按 hash
去重存一次，单 case 总额≤272 MiB，全轮配额 64 GiB；达到配额先结束计分并标 incomplete，
不得覆盖旧失败。T3 仅下一次具名诊断启用，不用其耗时认证性能。

T1 取证扰动资格：F/E 各取一个冻结代表 case、30 对 trace ON/OFF，AB/BA 交替；
两侧分别要求 ON/OFF P95 ≤1.10，30/30 trace 完整、至少29对共同成功，失败 replay
完整。成功不足、分母无效或不可重放时为 not_evaluated；先修取证再计分。此为额外的
前置资格运行，**不包含在 4,000 个计分请求内**，本次也不执行。

失败 manifest 必写首次失败阶段、request identity、窗口、已保存/缺失文件及 replay
是否可行。只有最终 summary 或只有“超时”文本是 incomplete evidence；先补捕获再重试。
成功需保留每条证书及原始时序，不仅保留汇总百分位。未来 harness 必须提供从该目录
加载原始输入、在同 revision 复算指定 F/E case 的 replay/inspect 入口并行为验证后才能派工。

## 10. 本次文档发布的验证与结论边界

已在干净 `b77091ad45323edd1d18824813155d76f8aebdc1` 运行：

```bash
source /opt/ros/humble/setup.bash
ELFIN_E2E_ROOT=/home/adamliao/work/elfin_e2e \
  /tmp/uld-upper-python/bin/python -m unittest discover -s research/uld_upper_layer/tests
```

结果 296 tests / 0 failures / 0 errors / 0 skips，25.076 秒。只证明现有 CPU 行为回归
无新增失败；不是上述 F/E/N 性能或真实 GPU 资格结果。原 GPU runtime/完整资产资格仍
未评估。主证据为
`docs/status/evidence/uld_upper_layer_v1/curobo_cpu_benchmark_publication_g1/`。

提交只新增本文件及本任务角色记录；继承的 GPU 适配等 8 个开发提交保留于分支历史。
本草案自身不更改既有 `ULD_UPPER_LAYER_V1.md` 的评测结果或其他任务结论。
