# 规划验证验收边界（用户指令 2026-10-19 冻结；主目标=多箱放置合理＋轨迹无碰撞）

## 碰撞两类（不统一放宽）

| 类型 | 处理 |
|---|---|
| 已定位的**吸盘—当前载荷**接触区重叠（简化包络产物；UNIFIED_GEOMETRY.md 全链实测） | 显式仿真近似（接触凹区 box_recess 已入库）；不以消除其物理弹射为主线；动态弹射研究暂停 |
| **箱—容器、箱—邻箱、机械臂—环境、非允许自碰撞** | 严格检查不变；不设通用 ~10mm 容差；~10mm 不能一律当误判，也不能据此断言实机无问题 |

## 三个验收结果（运动学验证模式 `--release-mode kinematic`，默认）

1. **placement 合法**：箱在容器内、支撑成立、与已放箱满足声明的几何约束。
2. **完整路线合法**：携带进入/下放/释放后退出全查完整机器人＋载荷；释放箱不得从场景消失。
3. **多箱状态正确**：下一箱使用上一箱**实际采用**的放置状态（地图提交实测/声明位姿）；旧槽位或错误占用不得继续规划。

### 运动学模式语义（隔离接触求解器伪影）

- 按认证轨迹运动箱体；在**声明的释放终态**解除附着并作为障碍保留：一次声明式
  set_pose 到槽位支撑面位姿＋新鲜位姿流回读确认（≤5mm），**不** delete/重生/
  自由落体/弹射；释放驻留不适用（无动态体）。
- **不等于**把穿墙轨迹"瞬移成成功"：释放终态、支撑条件、相位转换、占用提交
  照查；全臂碰撞检查照旧（MoveIt 认证链未动）。
- **证明**：规划与状态链可行性。**不证明**：实机接触安全、吸附可靠性、最终
  落点精度——以后在 dynamic 轨道（`--release-mode dynamic` 保留）单独验。

## 基线与不变项

- **基线恢复自 r5**（唯一有效 n=2 PASS；VERDICT.md）。40mm 门、规划预算、
  占用合同、感知尺寸、placement、bitmask 方案全部不动；不扩大通用碰撞容差。
- r7 根因（跟随单 tick set_pose 50ms 超时被锁存为丢载）已按监测语义修复：
  连续 3 次失败才发布原因、成功即清除；真 detach/吸附门不受影响（宽限只作用于
  传输瞬态锁存，不触碰任何碰撞/物理检查）。

## 如实保留的未知

现有证据**不能**说明问题是"多箱本身造成"——槽位、姿态、接触关系、运行负载
都可能随箱数改变。本边界不追其物理根因，此未知保留在案。

## 下一次计分运行（骨架，待授权）

- 模式：kinematic（显式 `--release-mode kinematic` 记入 manifest 参数）；
  判据=上表三结果＋原 40mm 门/占用合同；判权=用户裁定箱数（多箱 n 待定）。
- 预检沿用 preflight_r7 模式（位姿流/服务/清扫归属）；运动开始即计额度。

---

## 记录修正（用户指令 2026-10-19）

1. **实施偏离声明**：独立集成分支 `agent/eng/integration-prior-g1` 是
   实施选择，非用户指令顺序；用户指定的工作分支是当前 G3 分支
   （agent/eng/master-pack-baseline-g3）。集成分支保留用于验证；
   合回 G3 分支未经用户下达，须待其明示。
2. **回归口径修正**：此前"零回归"准确表述为——**已执行测试中零新增失败
   ID**（对照 ce361117 逐项复现）；同时明示：4 个 prior-node 语义测试
   显式 skip（记为**能力缺口**，不计入"prior 功能全过"）、depth-view
   通道停用、install-st09 环境测试未执行、2 项组合批伪影、st09 B/I
   进程隔离套件未并入组合运行。1697 通过不能替代缺失能力的验证。

## prior 实际接线检查（n=2 前置门，进行中）

已关闭（离线只读链路检查 7 测全绿，test_prior_occ_chain_offline.py）：
- **GetCargoVolumeV2 可用且数据出自 OccupancyMapStore**：身份绑定改为
  store 快照（epoch+revision+content_digest 请求钉死/响应回显；裸读
  mapper revision 与仅初始化镜像已废除）；错 revision/epoch/digest、
  跨快照（快照钉死后地图再变更）、未确认态全部具名拒绝
  （SNAPSHOT_IDENTITY_MISMATCH / NO_CONFIRMED_MEASURED_VOLUME）。
- **prior 可从 owner 体积生成**（真实 store→volume→ConfirmedSurface→
  build_private_snapshot 链；碰撞端口为离线桩——链路与身份合同已证，
  碰撞正确性不在此证）；准备时长独立字段（elapsed_s/cache_hit）；
  失败具名回退（adopted=false+reason）合同钉死。

**尚未关闭（n=2 前必须）**：实际采用路径——`place_prior_node` 未在
sim launch 启动；pack 计分驱动（PlaceSmokeDriver）无 prior 客户端接线
（现仅 motion_occ_eval_driver --placement-prior 可达）。需把 prior 客户端
按同一 opt-in 接入 pack 路径＋launch 启动 sidecar，形成**新冻结版本**，
n=2 才能同时报告"两箱流程＋prior 是否真正采用＋准备/规划分别耗时"。
