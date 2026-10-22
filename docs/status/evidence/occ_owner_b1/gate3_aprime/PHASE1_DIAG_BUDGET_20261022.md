# 阶段一功能诊断（180s 预算入搜索器）——2026-10-22

用户三阶段目标（本日 /goal）：①有界放宽只定位保持故障（180s 真实传入搜索器，
客户端等待=预算+30s 通信开销，分段计时 准备/搜索/认证/总墙钟，一次有效尝试）；
②修复后 n=3 功能验证（同配置、不降门、单独报告 prior 采用）；③再评估收回 60s。

## 实现轮（commit a7a47551）

- `diagnostic_route_budget_s` 参数（selector，默认 0.0=冻结 n×60s 不变）→
  `build_production_route_producer(route_budget_override_s=…)` → **RouteProducer.budget_ns**
  （搜索器内强制，非客户端等待）。launch 选项同名；驱动 `--select-budget-s`
  镜像之（客户端等待=服务预算+30s）。transit_diag_run.sh 单旋钮
  `DIAG_ROUTE_BUDGET_S=180` 同时喂两侧。
- 分段计时：`_stage_timings` 新增 `prep_ms`（请求入口→候选环入口）；
  search=总−prep−certify−revalidate 残差。失败时驱动把 selector
  `rows_json`（逐候选/逐边失败原因）与 `stage_timings_json` 存入 trial extras。
- 测试：override 三路径入搜索器 + prep 计量（factory 5 绿）；sampler/anchor/
  cleanup 15+5 绿。test_planning_request_capture atlas fixture 失败经 stash A/B
  证实为 HEAD 上既有（与本次无关）。

## 尝试史（boot #5=114850、#6=120028、#7=scene 修复后）

| boot | 结果 | 归因 |
|---|---|---|
| 5 (11:48) | 图门死：camera_bridge/waypoint_generator 进程已 ready 但 DDS 图不可见 + 7 peer 自报心跳断流（PEER_LEASE_EXPIRED 永久毒化） | 启动期 SHM 端口锁竞争 boot flake（同 attempt-2 族），未到 Select=非有效尝试 |
| 6 (12:00) | **有效尝试**：180s 预算实测进搜索器（log: `route budget 180s for 1 box(es) (DIAGNOSTIC override of frozen 60s)`）→ 177.6s 耗尽 | 见下方根因 |
| 7 (12:3x) | scene 修复后重跑 | 待记 |

## Boot #6 的决定性证据（180s 仍耗尽，去向完整留存）

- `stage_timings`: total 177626ms / **prep 26ms** / search 177600ms /
  certify **0** / revalidate **0**；`by_reason`: IK_UNAVAILABLE 175442ms
  (98.8%)、EDGE_BUILD_FAILED 2158ms。
- 逐候选行：110 候选全部 `IK_UNAVAILABLE`（"no IK goals refined"，各 ~1.6s）
  +2 EDGE_BUILD_FAILED；atlas 失配（atlas=b40c85ae vs current=603f9b65，
  与 10-20 两 boot 相同）⇒ seeded=0。
- ik_refine 尝试：tool_down 种子系 pos_err **0.915m** 收敛失败；
  离线重放完全复现（0.915369 一致）⇒ 确定性几何错误，非运行态/算力。
- FK 定位：图/portal 锚 TCP x=−0.905（正确）；**槽 TCP 目标 x=+2.03（镜像）**。
  正确值 = 容器 [−1.5,0.6,0] yawπ ∘ slot[0.53,0.14] = [−2.03,0.46]——
  与 10-20 prodcap/run2c 成功 boot 的槽锚 FK 完全一致。

## 根因（闭环）

驱动进程未获 `--scene-tf-config`（transit_diag_run.sh 从 hold_diag_boot.sh
改编时遗漏；生产 runner `scripts/g3/run.sh:376` 有传，且 :85 注释明言场景 yaml
须是 "the ONLY yaml the launch AND the driver see"）→
`resolve_scene_tf_config_path` 静默回退包内 `scene_tf.yaml.example`
（容器 [1.5,0,0]、单位旋转）→ `container_transform_fields` 产出错误变换 →
全部槽目标镜像到不可达侧 → 全候选 IK 失败。

**结论：attempt 3/4（60s）与 boot #6（180s）的 PLANNING_BUDGET_EXHAUSTED
均为本脚手架缺陷，非生产规划墙**（证实用户纠偏"两次超时不能证明与旧吞吐
同根"）。G3 计分轮 5-8s/候选的墙是独立问题，未被本轮证伪。

修复：transit_diag_run.sh 驱动调用补 `--scene-tf-config "$SCENE"`（一行，
仅诊断脚手架；产品/规划策略/安全门零改动）。

## 遗留观察（记录不擅改）

`place_smoke_driver` 在无参/无环境变量时静默回退 example 场景，且其
frame 身份哈希（603f9b65，来自运行链）与几何配置（example）不一致却无门
拦截——建议产品侧改为显式拒绝或身份绑定（另行裁定，本轮不动）。

## 产物

- /tmp/jcc_runs/transit_diag_20261022_{114850,120028}（及 #7）
- 分段计时/逐候选行留存于 driver_out/dumps/box_00_pending/trial.json extras

## 增益修复验证 + n=3 尝试（阶段二）——2026-10-22 下午

**根因修复链**：transit_diag_run.sh 补 `--scene-tf-config`（ac2a92d3）后 boot
#7（transit_diag_20261022_123250）transit 完整执行
（DIAGNOSTIC_COMPLETE_CARRYING，认证 10 边全过），保持窗判定
**physical_response**：JTC 输出冻结（cmd spread=0）→ 插件换算忠实
（实测 K=10.2=gain 0.1×100）→ 物理仅给出指令速度的 ~1%，10s 漂移
0.12/0.27 rad。溯源发现 gz_ros2_control 0.7.20 参数送达缺陷：插件自身
节点先于 SDF `<parameters>` 解析创建 → yaml `gz_ros2_control:` 段从未生效
（**历次 boot 运行时均为 0.1**，而 elfin_controllers_sim.yaml 自基线起声明
0.8，注释明言 0.1 带不动摄像头+面板负载）。vendor patch 02（参数随节点
构造传入）后 **runtime=0.8 首次生效**。

**0.8 下的保持窗物理**（n3_functional_20261022_130429 捕获，真保持窗
197.86→201.17）：K=−80.0 精确；j0/j1 误差 0.062/0.074 → 1.6s 内先漂离至
0.088（施加 +7 rad/s 仍被约束——接触保持）→ 199.38 释放瞬态（v 峰
1.57 rad/s）→ 199.7 起**误差 0.0000、位置冻结、零漂移**。物理收敛 1.7s
完成（0.1 时永不收敛）。增益修复物理有效。

**n=3 尝试 A（130429）**：增益门过（0.8）→ 箱1 transit 执行 →
**HANDOFF_SETTLE_TIMEOUT**：settle 门证据流（controller_state 订阅）在
sim 199.3 断供（采样停在 144 条），门盲等 6s 超时；驱动 spin 线程
（executor）同刻死亡——回调异常逃逸杀线程，守护线程 traceback 被
interpreter teardown 截断丢失。已加 executor_crash.txt 崩溃留痕
（观测性补丁，零语义变化）。物理真值（插件流）与 JTC feedback 按 stamp
对撞 diff=0.00000——数据面无分歧，纯属驱动侧回调线程死亡。

**n=3 尝试 B（133336）**：增益 0.8 ✓、executor 无崩溃；箱1 INIT 规划
180s 耗尽于**生产规划墙本身**（G3 计分轮同签名）：32 候选×~5.6s，逐候选
留存 NO_FEASIBLE_ROUTE×10 / ATTACHMENT_EXPIRED×16 / CERT_SERVICE_REJECTED×4 /
IK_UNAVAILABLE×1，certify 1.78s（个别候选已到认证）。目标侧正确
（x=−2.03，场景修复保持）。附注：ATTACHMENT_EXPIRED 部分为 180s 预算与
attachment max_age=budget 的交互（长预算下搜索尾部过期），60s 下更少。

**prior 报告**：start_placement_prior:=false → prior 未进链路，
**未采用**（按裁定不算 prior 验证通过）。

## 决策点（待用户）

1. 生产规划墙（阶段二前置）：G3 四选一菜单（按箱重发预算／Select 期间
   降竞争／sampler 种子质量／接受单箱见证）。180s 不足以穿过 32 候选。
2. executor 回调崩溃（间歇）：已留痕待复现；如再现即有完整 traceback。
3. settle 门证据断供若再现（回调修复后），下一步查 controller_state
   传输路径（QoS/CPU）。

## n=3 补充尝试 C/D/E（2026-10-22 午后）——全部 infra/CPU flake，零箱运动

| 尝试 | 死因 | 分类 |
|---|---|---|
| C (13:44) | `GOTO_FAILED: PEER_LEASE_EXPIRED`（boot 期租约断流永久毒化，readiness 门拒） | boot flake（今日第 2 次） |
| D (13:53) | `graph_wait_failed`（图发现缺节点 90s+30s，同 transit boot#5） | boot flake |
| E (14:02) | `BUILD_FAILED: CERT_SERVICE_TIMEOUT /validate_route_v2 5s 无应答`（pick 认证服务饥饿） | CPU 竞争服务饥饿 |

**有效搜索窗补充**（B 的 rows 复盘）：冻结契约 `ATTACHMENT_MAX_AGE_NS=60s`
（place_targets），首次 `ATTACHMENT_EXPIRED` 在搜索累计 ~43.5s（拾取龄 ~16.5s）
——**任何预算下有效候选窗 ≈43s**，之后 136s 全部在过期轮替中空烧。B 的
CERT_SERVICE_REJECTED 实为首边 PORTAL_TRANSIT 的 RRTConnect error −6
（2s 单边预算内未建成）——随机性，prodcap 同配置穿过。

**今日 boot 账（10 次）**：infra/CPU flake 7（lease×2/graph×2/spawn 挂×1/
认证服务饿×1/executor 回调崩×1）、真内容 2（A settle 门断供、B 规划墙）、
干净成功 1（transit #7）。下午重载加剧。

## 最终决策点（待用户，无进一步自动动作）

1. **规划墙**（B 证据）：~43s 窗内需 RRTConnect 建成 ~10 边并过认证；
   四选一菜单（重发预算已被 180s 诊断配置覆盖，实际选项=降竞争/种子质量/
   单箱见证）。
2. **环境稳定性**：7/10 flake 使任何验证尝试成为抽签；需环境侧动作
   （错峰/绑核/降载）或接受多轮重试成本。
3. attachment 60s 时效为冻结数据新鲜度门——按裁定不以放宽换通过。
