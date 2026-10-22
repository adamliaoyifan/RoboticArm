# 会话总结：三阶段目标执行（180s 功能诊断预算）——2026-10-22

分支 `agent/eng/integration-prior-g1`（worktree /tmp/elfin-master-pack-baseline-g3）。
证据全账：`docs/status/evidence/occ_owner_b1/gate3_aprime/PHASE1_DIAG_BUDGET_20261022.md`。

> 2026-10-22 executor 复核更正：本总结中“attempt A executor 同刻死亡、settle
> 证据断供”和“带箱保持根因闭环”的结论被原始时间戳反证。144 条样本持续
> 到门截止前 17.3ms，截止时物理误差仍 0.127389rad；首次收敛在失败清理删除
> 箱子之后。历史异常栈已截断，类型未知。增益参数送达修复成立，但不能据此
> 宣称带箱保持已通过。后续以 `executor_root_cause_20261022/` 证据为准。

## 一、阶段一（有界放宽定位保持故障）：完成

- **配置落地**（a7a47551）：`diagnostic_route_budget_s`（selector）/`--select-budget-s`
  （driver）单旋钮两侧同喂，180s **实际传入搜索器**（运行时日志
  `route budget 180s (DIAGNOSTIC override of frozen 60s)`）；客户端等待=预算+30s
  显式通信开销；分段计时 `prep/search/certify/revalidate/total`；失败留存逐候选/
  逐边行。生产默认 0.0 不变，非计分。测试：override 三路径+prep 计量 5 绿。
- **用户两点纠偏均被证实**：60s 两次超时与旧吞吐墙不同根；采集频率差异确不能
  排除影响（真因另有其处，见下）。

## 二、挖出并修复的三个真缺陷（证据闭环，ac2a92d3）

1. **诊断脚手架**：transit_diag_run.sh 漏传驱动 `--scene-tf-config`（生产
   run.sh:376 有传）→ 驱动静默回退 `scene_tf.yaml.example`（容器 [+1.5,0,0]
   单位旋转）→ 全部槽 TCP 目标镜像到 x=+2.03（真值 −2.03；10-20 prodcap
   成功锚 FK 反证）→ 110 候选全 IK 失败。**attempt 3/4（60s）与 boot#6
   （180s）的超时全是它**，非生产规划墙。
2. **产品（vendor）**：gz_ros2_control 0.7.20 参数送达缺陷——插件节点先于
   SDF `<parameters>` 挂 context 创建，yaml `gz_ros2_control:` 段从未生效 →
   **position_proportional_gain 历次 boot 运行时都是 0.1**（yaml 基线起就
   声明 0.8，注释明言 0.1 带不动摄像头+面板负载）。patch02（节点构造即带参）
   后 **runtime 0.8 首次生效**。
3. **驱动（间歇）**：executor spin 线程被回调异常杀死 → settle 门证据流断供
   盲超时；traceback 曾被守护线程 teardown 截断，已加 executor_crash.txt
   崩溃留痕（3a89c52b，观测性补丁）。

**保持故障根因闭环**：保持窗三层判定 **physical_response**——JTC 输出冻结
（spread=0）→ 插件换算忠实（K=gain×100 精确）→ 物理不跟随。gain 0.1：施加
1.4–3.6 rad/s 仅得 ~1%，10s 漂 0.12/0.27 rad 永不收敛；gain 0.8：K=−80.0
精确，**1.7s 内误差→0.0000、位置冻结零漂移**（含 199.38 释放瞬态 v 峰
1.57 rad/s 的接触-释放过程，全窗捕获在案）。

## 三、阶段二（n=3 功能闭环）：未通过，两次真尝试+三次 infra flake

| 尝试 | 结果 | 实质 |
|---|---|---|
| A (13:04) | HANDOFF_SETTLE_TIMEOUT | settle 门证据断供（executor 回调崩）；物理侧其实已完美收敛 |
| B (13:33) | PLANNING_BUDGET_EXHAUSTED@箱1 INIT | **生产规划墙本身**：32 候选×~5.6s；NO_FEASIBLE×10/ATTACHMENT_EXPIRED×16/CERT_SERVICE_REJECTED×4（实为首边 RRTConnect error−6 随机失败）/IK×1 |
| C/D/E | PEER_LEASE_EXPIRED / graph_wait_failed / CERT_SERVICE_TIMEOUT(5s) | infra/CPU flake，零箱运动，不消耗尝试 |

**硬约束（B 复盘）**：冻结契约 `ATTACHMENT_MAX_AGE_NS=60s` ⇒ **有效搜索窗
≈43s 与预算无关**（拾取龄 ~16.5s），180s 其余 136s 空烧于过期轮替。放宽预算
的杠杆到此为止；43s 内需 RRTConnect 建成 ~10 边并过认证（prodcap 同冻结配置
穿过两次——随机性）。

**prior**：start_placement_prior:=false → 未进链路，**未采用**（按裁定不算
prior 验证通过）。

**环境**：今日 10 次全栈 boot，infra/CPU flake 7 / 真内容 2 / 干净成功 1
（transit #7），下午重载加剧。launch 已有局部 OPENBLAS_NUM_THREADS=1
（sim_world.launch.py:939/1018），全栈级线程钉死未做。

## 四、待用户裁定（未再自行动作）

1. **规划墙**：四选一（"按箱重发预算"已被 180s 诊断配置覆盖；实剩 Select
   期间降竞争 / sampler 种子质量 / 接受单箱见证）。
2. **环境稳定性**：flake 率使验证成为抽签——错峰 / 绑核 / 降载 / 接受重试成本。
3. executor 回调异常：留痕已就位，复现即有全栈；如采纳可按 ST09-I
   reason-loss 模式给订阅回调加具名防护（本轮未动，避免掩盖）。
4. attachment 60s 时效为冻结数据新鲜度门——不以放宽换通过。

## 五、产物索引

- commits：a7a47551（180s 预算入搜索器+分段计时）→ ac2a92d3（根因链修复+
  vendor patch02+PROVENANCE）→ 3a89c52b（n3 runner+executor 崩溃留痕）→
  fc8499c9（C/D/E 证据）
- 运行数据：/tmp/jcc_runs/{transit_diag_20261022_114850,120028,123250,
  n3_functional_20261022_130429,132415,133336,134400,135317,140256}
- 工具：scripts/g3/n3_functional_run.sh（增益验证门前置，0.8 不生效零运动中止）
- 阶段三（60s 收回、~400s 路线效率）：未开始，按框架排在阶段二通过之后
