# Stage 2c：生产 pick 前序诊断（2026-10-20）——设计未达成 + 意外第三次复现

按用户裁定执行（单次满足边界的非计分诊断；未满足记无效不补跑）。
三次尝试全部为**脚手架/接口级无效**（pick 条件从未被修改）；第三次
尝试中 driver 意外完整跑完 pick→place→transit 并**第三次复现**
HANDOFF_SETTLE_TIMEOUT（全程在诊断捕获下）。签名与 r10/r11 一致：
目标 [-0.8543,-2.3921,2.1368,...]，j1/j2-only 误差（窗末 0.059/0.117），
transit→transit 交接 6s 超时。

## 新事实（本轮证据价值所在）

1. **路线时长跨 run 非确定**：同 seed 0 同栈，三次计分级复现的 transit
   实际时长 = **111s（2c，RTF 0.985 实测）/ 121s（r10）/ 166s（r11）**，
   终点相同。合并版建路的 staging/梯级探索受时序影响，路线本身是
   非确定的——"同 seed 冻结"只冻结场景，不冻结路线。settle 失效与
   166s 那条特定腿**无绑定**（111s 腿同样失效）。
2. **排除链收紧**：2a/2b 用同一条 166s 冻结腿在无 driver 上下文中
   完美保持 + 本轮 driver 自排路线（111s 变体）复现失效 ⇒ 机制进一步
   收敛到 **driver 的在场上下文与其自排路线的组合**，而非轨迹内容、
   末端姿态、move_group 路径（均已被 2a/2b 隔离排除）。
3. **PEER_LEASE_EXPIRED 中止路径会删除携带箱**：用"杀 placement
   planner 使 place 首步服务不可用"来停循环的设计无效——被阻塞的
   服务调用突破 2s 墙钟租约 → 租约中止路径删除箱体（第一次尝试实测）。
   production 租约=每 peer 2s 心跳（runtime_readiness.py:85-107）。
4. pick_retreat_eval_driver 的 v2 goto 路径**不完整**（_last_joint_state
   由子类注入；该 driver 生产从未走 frame 链）——它不是"生产 pick"的
   可用载体；生产 pick 载体=pack_eval_driver（第二次尝试实测崩溃）。
5. **生产 pick 的确定性得到实证**：pack driver 的 pick+retreat 落点与
   冻结腿首 knot 最大差 **0.00032 rad**；杯位 [0.565, 0.592, 1.427] 与
   r11 tf_trace 完全一致；retreat 抬升 1.319 vs r11 1.320。

## watcher 脚手架教训（第三次尝试触发器失效根因）

- `rclpy.spin_once` 单回调 × `box_pose()` 子进程阻塞（~2s/次）→ 订阅
  队列饿死：joint_states 实际捕获 ~5Hz（连续但稀疏），**on-change 型
  action status 事件全丢** → SIGSTOP 触发器（要求 ACTIVE goal）从未
  满足 → driver 未被冻结、自然跑完全程（反而带来第三次复现的完整捕获）。
- 修正方向（未实施）：box_pose 移入独立线程/以 /world/pose/info 流
  替代 CLI 轮询；触发条件用事件而非轮询。

## 额度与判定

- 三次尝试均记**脚手架无效**（parser 旗标缺失 / v2-goto 崩溃 / 触发器
  饿死），pick/边界/腿条件零修改。设计意义上的"满足边界的单次 2c"
  仍未执行；第三次尝试的实际产物=一次带诊断捕获的**完整生产复现**
  （非计分、max_boxes=1、失败中止）。
- 判定按用户规则收敛：下一单一变量=**driver 在场上下文**（其在场时
  自排路线失效 vs 2b 冻结腿完美保持），其后全栈并发。
