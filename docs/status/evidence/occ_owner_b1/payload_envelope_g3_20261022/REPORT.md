# G3 10% 载荷包络与闭环复核 — 2026-10-22

用户要求的相对膨胀已实现。功能闭环仍未通过：有效运行尝试1箱、装入0箱，
未生成可认证携带路线。结果证明缩小包络打开了部分边规划分支，但不能证明
当前候选姿态与搜索预算足以完成装箱。

## 冻结配置与实现

- 计划 `7336bfcb`；实现 `99989272`；离线比较脚本 `63af7842`。
- 名义测量尺寸和偏心 `T_tool_box` 保持不变。
- W/D尺寸各总增长10%，两侧对称；XY额外位置范围从各100mm降为0。
- H尺寸只在远离吸盘的一侧增长名义高度的10%，5mm Z位置误差也只叠加
  在该侧。对当前工具朝下、箱体竖直合同，该侧是世界 `-Z`；接触侧不动。
- yaw连续误差仍为 `+/-5deg`，因此其联合外包会让最终W/D超过名义1.10倍。
- 无法区分吸盘近/远高度面的附着关系具名拒绝；roll/pitch倾斜仍拒绝。
- G2槽位包络诊断旁路仍只跳过槽位闭包预筛；全臂碰撞、路径认证、link2禁止
  和放置后验证均保留。

live_03旧输入的名义尺寸为514.327 x332.526 x217.541mm。G2包络为
849.686 x684.423 x327.541mm；G3为595.487 x413.696 x244.295mm。
G3工具Z近侧面为约-0.000002mm（数值残差），远侧为244.295mm；在当前
工具朝下姿态，新增高度全部指向世界 `-Z`。

## 离线复核

使用旧失败案例同一20个保存的中间关节状态、同一展开URDF和native FCL：

| 模型 | 与link1--4相交的状态 | link1 | link2 |
|---|---:|---:|---:|
| 名义测量箱 |16/20|11|16|
| G2固定裕量 |20/20|20|20|
| G3百分比/单向Z |19/20|18|19|

这说明包络缩小释放了1个旧姿态，但旧姿态族仍高度受限。这里只检查保存状态，
不代表连续轨迹、世界碰撞或候选集外无解。原始结果与G2既有16/20、20/20
计数一致，作为回放一致性检查。

## 测试与构建

聚焦包络和路线测试113项通过，覆盖10%尺寸、接触侧不动、反向box-Z、连续
yaw极值、配置拒绝、槽位旁路仍保留轨迹碰撞门。`st09_contracts`、
`luggage_planning`、`luggage_gazebo` 普通安装构建通过，源/安装配置一致。

第一次live_01误用symlink安装，驱动因规划节点报告源码路径而在0箱尝试时
具名 `WRONG_OVERLAY` 中止。该确定性接线问题改为普通安装后，节点路径逐项
核对位于worktree `install/`，冻结revision和配置未变；该次不计产品案例。

## live_02闭环结果

- revision `63af7842210daeff5109f21372f9e3ab95073344`，配置SHA256
  `01102a7a922b7ec5dc62b42ba1d8846e9e07bd8735c4bd048e90ad8a2d47ef10`。
- gain0.8、domain7、inch20、seed0、selector预算180s、槽位预筛旁路true。
- 1箱尝试、0箱装入，终止 `PLANNING_BUDGET_EXHAUSTED`；总wall207.0s。
- 选择总计180716ms：准备28ms、搜索180688ms、认证0、重验证0。
- 791槽位中尝试117，未尝试674。118行含63 `EDGE_BUILD_FAILED`
  （均为staging_endpoint）、49 `NO_FEASIBLE_ROUTE`、3
  `ATTACHMENT_EXPIRED`、1 `IK_UNAVAILABLE`、1预算耗尽及1汇总行。
- 54次route build进入边规划；0携带段输出，0路线进入最终认证。
- 与G2的332槽位全部在中间姿态门前失败不同，G3有49个槽位越过该门并进入
  路线构建，但边规划/重采样仍未拼成完整路线。较少的117个已试槽位是因为
  MoveIt边规划消耗了预算，不能解读为回退。
- 驱动executor正常停止，`failed=false`；所有本轮Gazebo/bridge进程已清理。

## 证据与边界

`live_02`保存原始选择request、response CDR、解码JSON（含所有行和精确关节）、
manifest、包络配置、物理附着、规划失败dump、joint/plugin流、生命周期、日志和
teardown。约126MiB，未超过1GiB。运行时未单独重抓展开URDF；response记录的
URDF语义hash仍为 `3899e2b396650a61e9d0f56d61c2cce1`，与G2捕获一致。
因此新运行可做行级归因，但本报告不声称新的独立URDF字节捕获。

复现命令：

```bash
PHASE2_SKIP_SLOT_ENVELOPE_CHECK=1 PHASE2_RUN_OUT=<fresh-dir> bash scripts/g3/n3_functional_run.sh
python3 scripts/g3/replay_percent_envelope_collisions.py \
  docs/status/evidence/occ_owner_b1/payload_envelope_g2_20261022/live_03 \
  --config src/luggage_planning/config/payload_envelope.yaml \
  --output <output.json>
```
