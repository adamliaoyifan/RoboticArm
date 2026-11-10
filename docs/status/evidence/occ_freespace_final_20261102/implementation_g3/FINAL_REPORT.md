# OCC-FREESPACE-IMPLEMENT gen3 — 复核四组 A–D 一次补齐

- owner: eng / claude / glm-5.3（Q-20261102-4，2026-11-02 16:30 领取；复核 745740ae / review 2026-11-02_1621）
- worktree: /tmp/elfin-occ-freespace-impl-g3，分支 agent/eng/occ-freespace-impl-g3，自 1181a52e（gen2 证据头；候选基线 ca28c885）
- freeze: FREEZE.json head 7e2b7a25（候选工具提交 0e3dd151；其后仅证据路径提交允许演进；clean；6 二进制 + 15 工具/静态库/typesupport/
  消费者测试/g2+g3 测试/audit 回放 + 6 输入帧 + 2 fixture 按实际字节 + COMMANDS 自身）
- 正式评分未启动：score 默认关闭（实现完整、有界同源路径重演过）；S3 仍不开放
- 证据根: docs/status/evidence/occ_freespace_final_20261102/implementation_g3/

## 修复前红基线与修复后回放（复核 D 组要求）

- audit_baseline.py 先按复核 audit.py 原样回放：8 坏例在 gen2 判官下全部
  reasons=[]（误过复现，RESULTS.json 首版留档后覆盖）。
- 修复后回放：**3 个足额 formal 对照 reasons=[]，8 个坏例全部具名拒绝**
  （exit 0；tests 入口每次全跑）。对照夹具本身就是合同级（300/240、
  600+120/60/75+60 含首Wait链、18000/14400+1000/100+12桶），不再是复核
  指出的"不足合同夹具"。

## A 逐记录独立判定（判官）

- R/L 查询循环的无条件 break 全部删除——**每条记录**过状态/释放/完整五字段
  身份/数值健全检查；语义答案必须带物化候选且 canonical 摘要能从原始候选
  **重算一致**（不再只查字符串存在）。
- P 比对回执 **ID 集合**（timeline 与 sender 集合精确相等）+ 每行 receipt_count==1
  （重复终态保留并计数，不按 ID 折叠到最后一条）；e2e 从 publish/回执到达钟
  **重算**并要求非负一致；查询负时延（ended<planned/ended<started）逐条具名。
- L 严格门：18000/14400 **精确等于**（90% 放宽删除）；10min 预热后 110min
  按 120s 窗 AND（每窗恰 300 输入/240 查询 + 350/500、250/500、20/50 全门）；
  RSS 按真实时间（时间戳严格单调、12 个计划桶**全部在场**且每桶≥600 样本、
  首/末评分 10min 中位比较，替换按值排序的十分位）；1000read/100reset
  **逐轮**所有权回基线（自适应采样取 min：瞬态并入查询/integrate 的双
  retained 视图自然落出，泄漏对全部样本免疫），不接受自报轮数或句柄≤N。
- R：排空按**回执到达时刻 vs 精确截止**重算（截止后成功=未按时排空，具名
  drain_late），runner 自报清单与重算不一致也是 finding；首 Wait 与恢复首输入
  planned 时刻必须相同；首 Wait 的租约必须完成候选链（链记录全套字段）并释放。

## B 真实调度与租约闭环（runner）

- R 恢复三线程同 t0 并发：发送 75 条、首 Wait（**其租约直接分页走完整候选链
  并 finally 释放**——不再拿走租约不用）、其后 59 条完整查询按 t0+0.5s(i+1)
  同表（正式合计 60 含首Wait，替换 recovery_inputs//2=37 的错误数量）。
- 排空期 60 条查询有自己的 2Hz 时刻表；drain_deadline 由 drain_plan 计算。
- L 逐输入发布/终态 + 逐查询计划/起止**同源采集**（P/H 门可验）；1000 读轮
  分布在 110min 评分窗（与输入流并发，不再停止后补测→不再必然 STALE）；
  100 reset 轮在预热期——**真实可观测生命周期**：活图上取真实租约、跨 reset
  持有、finally 显式释放（产品 release 恒 OK/幂等，gen2 的 bogus-handle
  "错误路径"是虚构的已删除），refill 后所有权回基线证明未靠 TTL 滞留。
- R/L formal 绑定**固定 20 箱帧**（frames_manifest 逐帧 sha+box_tag，判官
  必查 tag==20；混合/短帧集不能冒充正式工作量）。
- P 每窗独立文件 p_bNN_wN.json 含 window_id（九窗集合判官）；**阶段失败
  停止依赖**（P 败→不跑 R；R 败/预算尽→不跑 L，skipped 记原因）；预算由
  调度器 --budget-minutes 在阶段间强制执行；异常时 failure.json 前缀拆卸前
  落盘。调度事件序列有**离线可验的单一来源**（p_window_plan/recovery_plan/
  drain_plan，阶段与测试共用；启动顺序不再花长窗验证）。

## C 冻结、磁盘判官与失败证据（COMMANDS）

- require_freeze：五组齐全非空、逐条按字节校验、**commands 条目必须等于
  当前脚本自身字节**、HEAD 实测等于冻结 head、工作树实测 clean（不信
  freeze.dirty 自报）——判官分支再改脚本而沿用候选标签不再可能。
- judge 入口：覆盖 rehearsal **与 scoring** 两目录；P 按九窗集合判
  （judge_p_matrix 要求 3×3 恰好一次）；scoring 目录的 R/L 必须 formal 全绿
  （short marker 不允许）；failure.json 前缀在场即 finding；**O9 从逐对
  samples.yaml 原始样本独立重算**（判官自带 o9_document_problems 重实现：
  status/缓存状态机/roi_cells/哈希在场），P95 比值、比值与 result 一致性、
  verdict.performance_certified、逐对 freespace/内容摘要 ON=OFF——改 ratios、
  改 verdict、删一对、换摘要都会红。
- verify 负例：未授权 score 拒；篡改二进制 SHA 拒；篡改 commands SHA 拒；
  篡改 head 拒；**删一窗 + 伪造 summary + 谎报 ratio 的整棵证据树拷贝，
  磁盘判官必须红**。
- 预算/失败停止在调度器执行（见 B）；replay 保退出码真实执行产品。

## 验证

- 离线：g3 36 过（足额对照绿 + 复核 8 坏例红 + 字段×位置(首/中/末)×故障
  矩阵 + 调度计划 + 九窗 + O9 重判正负例）；g2 对齐后 18 过（audit 注入 +
  full_query 五故障 + score 守卫）；全套 100 passed / 3 skipped；行为二进制绿。
- F 矩阵（g3 重跑）：**360/360**。
- 短接线预演（新调度器 + 逐记录判官）：P×3/R/L 全部仅 short_sample 标记，
  **wiring_ok=true**（此前 R 的 wait 链 latency、L 的所有权/ACQUIRE_STALE
  问题均已闭环——瞬态双 retained 视图经自适应采样取 min 消除，泄漏不可藏）。
- 有界 score 预演：campaign wiring_ok + O8 bounded（非计分、不可认证）。
- 冻结后四操作 O9（head 0e3dd151）：见 ACCEPTANCE（verdict pass /
  performance_certified=true / 30 对零失败，含 acquire 的全链比值全 ≤1.10）。
- judge 入口从盘重判（360 案例 + rehearsal 各阶段 + O9 原始样本重算）problems=[]；
  verify 守卫与磁盘判官负例全拒；replay 保退出码。

## 诚实边界

- 正式 P/R/L/O8/O5/O6 未启动；有界路径不构成正式结论。
- 短样本 R 首 Wait 的 ≤2s/年龄属 formal 门；时间线证据保留。
- 输入 stamp 采用计划周期步进；与真实采集时间戳的严格一致由 S3 formal 输入
  manifest 绑定。
- L 读轮在满负载下偶发 ACQUIRE_STALE（无租约、诚实记录）为合法可观测；
  所有权硬门不受影响。正式 L 若出现该形态，判官按记录如实出具。
