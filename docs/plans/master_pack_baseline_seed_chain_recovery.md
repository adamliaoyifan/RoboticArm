# Master pack 基线恢复与 seed-chain 实验接续

- 日期：2026-10-13
- 计划版本：G1；派发线程的 `plan_revision` 固定到包含本文件的 Git 提交。
- Parent：`MASTER-PACK-20261013`
- 代码基线：`b1de4cb9f9ea17e4b3aae277a4306574284ca812`
- 授权：用户指定第一项交给 `cursor / grok-4.7`；第二项暂不派发。
- 本计划不批准容量、稳定性、实机安全或 S6/S7 验收。

## 1. 两项实验现状

### BASELINE：master pack 基线恢复

Claude 因 API 限流暂停（用户说明）。原工作目录为
`/home/adamliao/work/elfin_master_b1de4cb`，不是纯 master 可复现基线：
其 `BASELINE_OVERLAY.md` 包含部署资产和代码修复。18 次启动/运行尝试发现
QoS 局部导入遮蔽、frame-runtime CLI 缺失、安装身份、位姿身份/格式、观察位
收敛/zone、inch20 目录/模型资产和 YOLO 等阻断。启动失败不是有效装箱样本。

最后保存的一轮为 0/3，三条均 `YOLO_NOT_READY`，未进入放置。
可见箱体的诊断帧据记录在 `/tmp/cam_diag_frame.png`，接手必须检查是否仍存在；
尚未证明是权重、prompts、节点接线还是其它感知差异，不能先定性为模型回归。

旧 4f53c9b 混合树曾放下 1/6，但 cargo map 存在 world 数值冒充 container 坐标、
零体素写入仍成功的问题。MoveIt 当时有 `placed_0_0_0`，不是所有碰撞场景都盲。
master 已合入 typed caller 补完；放箱后 occupied 的在线证据仍缺，不能先记已修复。

证据入口：
- `docs/agents/eng/2026-10-13_1550_slot-b-review-reverification.md`
- `docs/agents/eng/2026-10-13_1330_surface2d-placed-box-invisibility.md`
- `docs/status/evidence/base_container_tf/2026-10-13_master_b1de4cb_baseline/`

### SEED-CHAIN：朝向与种子链优化（冻结待派发）

工作树包含四个有向 yaw、按自身朝向取 Atlas prior、seed-chain 注记/加分。
hullshadow 两轮均 0/3；wrist-yaw 一轮 0/3，有一次到 descend，不能声称完整收益。
最新 seed-chain 保存结果为 0/3，全部 `YOLO_NOT_READY`，算法尚未在线受测。

本次复核 seed-chain 5 个单测通过；它们不验证物理路线。
`hindsight_chain_table.json` 的 12 条全部 `same_branch=false`，包括 `PLACED`。
跨两个目标位姿的关节距离阈值不是路径连通性证明；同样 false 也不是不可达证明。
现有实现只加分、没有硬拒绝，但其区分能力与排序收益未建立。

证据入口：
- `docs/status/evidence/base_container_tf/2026-10-13_seed_chain/hindsight_chain_table.json`
- `docs/status/evidence/base_container_tf/2026-10-13_seed_chain/pack_run/`
- `src/luggage_planning/luggage_planning/seed_chain.py`

## 2. 任务与权限

| ID | Owner agent/model | 依赖 | 基线 | 验收 | 状态 |
|---|---|---|---|---|---|
| BASELINE | cursor / grok-4.7 | none | b1de4cb9f9ea17e4b3aae277a4306574284ca812 | 第 4 节全部门通过；提交与证据齐全 | G1，dispatch_ready=yes |
| SEED-CHAIN | 未分配 | BASELINE | 后续固定 BASELINE 通过提交 | 须另定冻结请求矩阵、比较门槛与计划修订 | dispatch_ready=no；不建 OPEN 行 |

BASELINE owner 负责实现、离线测试、演练、冻结验收、提交、证据与关闭，不再拆测试角色。
最终两箱集成验证属于 BASELINE 的最后一道门；SEED-CHAIN 未来若派发，必须另含整链回归。
本次不启动第二项、不让其修改参与基线的评分/朝向/Atlas 算法。

## 3. BASELINE 工作边界与操作步骤

### 3.1 领取、隔离与资产固化

1. 从派发线程运行 `scripts/agent_start.sh`，身份固定为 role=eng、agent=cursor、
   model=grok-4.7、cli=cursor。主工作区 docs/agents 是唯一 mailbox。
2. 建立独立 worktree（建议 `/tmp/elfin-master-pack-baseline-g1`），基于上述代码提交。
   不重置主树、不复用另一人的 index、不覆盖旧证据。不从旧 script 自动 stop 别人的 sim。
3. 保存主树和旧 worktree 的 revision、tracked diff、untracked 清单和文件 SHA256。
   逐项审计旧 `BASELINE_OVERLAY.md`，只带入本任务必需的启动/typed driver/感知修复和资产。
   不整体复制 +161 行 driver 或整套脏树而不做归属审核。
4. 固化 `scene_tf_deploy_update.yaml`、atlas runtime/npz/yaml/URDF、robot poses、
   inch20 catalog/model/纹理、YOLO/CLIP 权重与 prompts。配置和资产自述哈希必须匹配实际文件。
   外部大权重可留安装路径，但必须提供 SHA256、恢复步骤；不可只依赖 /tmp 文件或 shell PYTHONPATH。
5. 修复并测试 QoS 构造、frame-runtime/peer 参数传递、真实安装树身份、位姿 identity、
   inch20 spawn 资产解析。保留 master 现有规划行为，不带入 seed-chain/wrist-yaw 排序补丁。
6. 清洁环境构建，核对实际执行模块、配置、模型与冻结源码。计分前提交全部相关代码与配置；
   实验 worktree tracked dirty=0，无未声明 untracked 代码/配置。证据文件不算行为污染。

### 3.2 离线定位检测与地图提交

- 先将可用诊断帧复制到新 evidence 并算哈希；缺帧则记录 missing，不用相邻帧冒充历史帧。
  用相同权重、prompts、阈值及颜色/尺寸预处理分别运行模型原始输出和生产 segmenter/授权链。
  逐帧保存 bbox、置信度、类别、mask、几何/来源、授权理由和时间戳，定位首个差异。
  现有 replay_evaluate 可复用，但不把裸 RGB 冒充完整 RGBD 几何回放。
- 若原输入不足，只允许先补一次非计分取件台静止观察，采完整同戳 RGB/depth/info/TF/检测/掩膜。
  不启动 n=3 或 20 箱来捕获这一个前置失败。
- 输出机制必须分为 observed、source-established、hypothesis；不通过降检测阈值、目录尺寸回填、
  GT 输入、伪造 MEASURED_SUPPORT 或跳过授权来恢复链路。
- 验证 mapper 的零体素写入失败且不产生成功 revision/台账；driver 检查 add.success 与 revision。
  合法增箱 revision 必须单调前进且响应与读取快照一致；存在并发更新时不得仅用 expected+1 误判。
- 生产下一箱占用来源必须符合 real_scenario.md：实时深度或已验证的测量提交。
  仅计划位姿写 ledger 的路径只能算诊断，不能签 BASELINE 通过。

### 3.3 演练及冻结两箱

- 环境固定 ROS Humble、Gazebo headless、domain 7、deploy_update yaw-180 场景、bbox_fill backend、
  Atlas strict identity、inch20、seed=0、yaw_mode=discrete、size_mode=catalog（仅 spawn）。
  明确本箱型不是 A-touch 的 0.55x0.40x0.25 carryon，不计算对 51 的容量效率。
- 起点 pickup_observe_sim，保留 goto-timeout=60、auth-max-attempts=8、auth-max-elapsed-sec=8。
  不改变生产碰撞、fraction、奇异性、交接和授权门槛。记录解析后的全部默认预算/控制参数。
- 参考旧证据的 `run_baseline.sh` 编写新 evidence runner，保留同一 launch 参数；必须替换 WS/OUT、
  改为两箱，删除启动前无条件 stop_sim，使用当前 sim-lifecycle 的排队/所有权检查。
  参数允许更改的仅上述路径、两箱数量及明确已测试的运行使能件；其余差异写入 manifest。
- 演练：先静止观察，再单箱及地图提交。启动/接线演练总活跃墙钟上限 90 分钟（排队不计），
  每个不同失败先离线归因和补捕获，禁止对同一未定位失败盲重启。到限报告 blocked。
- 就绪至少记录 10 秒：/clock 发布者恰为 1；相机与推理各有至少 10 个递增 stamp 的有效样本，
  记录实际频率而不据此声称性能达标；观察位到位、frame/atlas/install 身份全部通过。
- 冻结后唯一一次计分运行：依次两只 inch20，任何一箱失败立即结束，不用替补箱凑 2 个。
  driver 使用 `--frame-runtime --frame-peer pack_eval_driver --observe-pose pickup_observe_sim
  --sequence-ids inch20 --max-boxes 2 --goto-timeout 60 --seed 0 --auth-max-attempts 8
  --auth-max-elapsed-sec 8 --scene-tf-config <冻结配置> --out <新目录>`。
  若 driver 不具备首失败停止，先在演练阶段加该选项及行为测试，再冻结；禁止计分中修代码。
- 计分整次上限 20 分钟。到限保留边界证据、退出并拆除自有进程；不扩大预算或再次计分。
  首个箱请求开始前的启动失败不消耗计分样本，但受 90 分钟演练预算约束。

## 4. BASELINE 验收、测试与停止规则

| 门 | 工作量/指标 | 通过条件 |
|---|---|---|
| B0 离线启动行为 | driver 构造/CLI/安装身份/poses/catalog 5 类回归 | 每类至少一个真实调用正例和一个相关失败例；全部通过，不能只 grep 源码 |
| B1 感知边界 | 同一完整输入重复 2 次；另有 missing mask、hold-only、height invalid 三类负例 | 两次目标关联和授权结论相同；有效输入能形成 measured 授权；负例零次抓取提交 |
| B2 地图边界 | 合法 box、零体素/出界、add失败、pending、确认后消费五类 | 合法 box 写入非零体素/列；非法写入不成功；提交失败不记 committed；未确认零次下一箱规划；确认后消费相同 revision/content |
| B3 安全负例 | 同一机器人/载荷状态，障碍插入与移除各一次 | 插入已放箱/occupancy 障碍报碰撞，移除后对应检查通过；保留接触对象与场景快照 |
| B4 两箱整链 | 同一冻结提交，1 场 x 2 箱 | boxes_packed=2、committed=2、无 verdict failure；两箱所有执行段成功，释放/退出/返回完成；第一箱 confirmed 后才规划第二箱 |
| B5 地图交接 | 每箱放置前后及下一请求的 map/scene | 每次真实占用更新 occupied列>0、max_height>0且 revision前进；第二箱请求包括第一箱障碍；frame/content/hash、确认 token 能贯通，无静默 add 失败 |
| B6 证据/退出 | 一个独立运行目录 | capture_complete=true，所遇失败 replay_possible=true；driver rc=0（通过时），拆除自有 sim/bridge 残留=0 |

通过为 B0-B6 的 AND。这是一次两箱功能集成见证，不是成功率估计；本任务不设可推断的长期
单箱成功率门。若单箱独立成功率为 p，仅两箱 AND 的概率为 p²，无法据一次结果区分例如
0.95 与 0.99，故不签可靠性。负例用于证明门能拒绝坏输入，不用成功样本替代负例。
缺测量、缺授权或未到达某阶段均 not_evaluated，不用 0/0 通过。离线环境问题单列，不算产品失败。

聚焦回归全部通过；受影响包测试须与 b1de4cb 按失败 ID 比较，新增失败=0。
若完整场景在规划/执行阶段失败，保存证据并报告 BASELINE blocked；不要在本任务调 Atlas、
seed-chain、TF、箱型、安全门或控制增益。通过才调用 agent_complete --outcome pass；
未通过以 blocked 关闭并写出首个未满足门，不把“排障结束”记为产品通过。

## 5. 证据与交接

新产物根：`docs/status/evidence/master_pack_recovery/2026-10-13_g1/`。
每次启动独立子目录；禁止覆写两项原证据。manifest 记录 base/test/plan commit、dirty 数、
命令、seed、配置/模型/安装哈希、overlay清单、时间、阶段计时、输入来源和退出状态。

| 边界 | T1 / 触发 | T2 内容 | 窗口与检查 |
|---|---|---|---|
| 启动/身份 | 阶段起止、PID、topic publisher和频率；超时/不匹配触发 | launch stderr、解析参数、安装模块路径/哈希、TF/时钟状态 | 本次启动全阶段文本；manifest 核对 |
| 传感器到授权 | 帧/request/instance/source/count/reason；拒绝或超时触发 | 同戳 RGB、depth、info、TF、检测、mask、过滤云和授权输出 | 在诊断中测得端到端最大延迟 L 后，前窗=max(2s,2L)，后窗=1s；确认同戳可 join |
| 放置到认证执行 | candidate/edge IDs、场景revision、reason、耗时；任一拒绝触发 | 完整请求候选、q、附着、PlanningScene/ACM、MoveIt请求响应、轨迹/实际关节 | 完整失败调用加前2s后1s；用现有 replay 入口或只读 JSON 检查器核对 IDs |
| 释放到下一箱 | commit返回、体素/列数、height、revision/token；失败/不一致触发 | 释放前后测量、add请求响应、surface、下一规划的场景/输入 | 完整转换及前2s后1s；验证地图/场景关联，不依赖 GT 代填 |

T0/T1 常开；T2 每种失败签名保留首个完整窗口。JSON/JSONL + PNG + 压缩 bag/点云；
每启动最多 1 GiB，总 8 GiB，到限停止采集与实验并报 incomplete，不淘汰首个失败证据。
演练报告采集开销（处理时间与丢帧），不能未测就声称零开销；必须帧关联缺失=0 才进入计分。
若 8 GiB 不够，先报告阻断，不临场删证据。GT 只在独立 reference 子目录评分。
每个目录提供 `run.sh`、`inspect.sh`（只读核对本表字段）及 `summary.json/md`；失效回放不得伪造输入。

提交仅属于本任务的代码/测试/文档；输出最终 commit、测试结果、证据指针与首个未满足门。
BASELINE 的代码提交应能从清洁 checkout 恢复上述行为。handoff/role notes 始终写主树 docs/agents。

## 6. TODO

- [ ] BASELINE：领取、保护原脏树、保存 overlay 和资产哈希。
- [ ] BASELINE：离线定位 YOLO_NOT_READY，修复真实根因及边界回归。
- [ ] BASELINE：闭合零体素写入/driver响应/测量确认/下一箱场景消费。
- [ ] BASELINE：完成非计分启动、静止观察和单箱地图交接演练。
- [ ] BASELINE：提交冻结行为，完成一次两箱功能集成；失败保留证据并停止。
- [ ] BASELINE：提交结果、角色笔记、关闭 mailbox，并给出可复现入口。
- [ ] SEED-CHAIN（暂停）：保存当前补丁，不改变基线，不运行在线实验。
- [ ] SEED-CHAIN（未来）：补同分支但碰撞/异端点大关节差但连通的反例；定义指标含义。
- [ ] SEED-CHAIN（未来）：在 BASELINE 通过后，冻结请求矩阵、重复数、误判/排序收益和耗时门槛，
      另写高代计划并获授权；此三项不是本次派发内容。
