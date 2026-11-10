# 感知、Attachment 与 planning 包络可视化及碰撞重放

## 结果

使用保存的完整精度关节角和 URDF 碰撞网格，离线复现了历史 G2 与当前 G4
候选姿态的箱体—一、二号臂碰撞。没有启动仿真、重新搜索或执行携带轨迹。
产品代码和裕量配置未修改。

| 几何来源 | W×D×H，mm | 相对位姿来源 |
|---|---|---|
| 感知名义盒，蓝 | 514.327×332.526×217.541 | 保存请求的 measured dimensions / T_tool_box |
| Attachment，橙 | 595.487×413.696×244.295 | 保存请求的 conservative 几何 |
| 建路器输入，绿虚线 | 同 Attachment | 请求及 route_producer 的 PayloadSpec 映射；与橙色重合 |
| 实际 scene 快照，紫 | 同 Attachment | GetPlanningScene CDR 的 object.pose × primitive_pose |
| 历史 G2 包络，红 | 849.686×684.423×327.541 | 历史离线证据中保存的包络；不是当前配置 |

绿线是按保存请求及源码映射呈现的建路器输入，**不是新捕获的 PayloadSpec 运行时 dump**。
紫线则直接从真实 scene 快照解码，不能用绿线代替。当前请求 geometry digest 为
`9181a3b23f1256a75fb07f2058fc5f437745e387e158074eadaddc85903f001c`。

## 图示

- [三层几何局部对照](envelope_comparison.png)：工具 XY / XZ，单位 mm。
- [当前失败候选](g4_collision.png) / [交互三维](g4_collision.html)：
  state 1，蓝=感知、橙=Attachment、绿虚线=建路输入、紫=scene、红点=FCL 接触样本。
- [抓取起始姿态](g4_attach_start.png) / [交互三维](g4_attach_start.html)：与候选碰撞区分。
- [原 G2 slot_index=332 碰撞姿态](original_g2_collision.png) /
  [交互三维](original_g2_collision.html)：历史 state 11，保留当时的红色旧大包络。
  slot 332 对应多个 staging 姿态，图只展示其中一个，不表示发生过执行撞击。

HTML 可旋转、缩放、点击图例隐藏图层；需保留同目录的 plotly.min.js。
HTML 机械臂显示网格抽稀、坐标保留 6 位小数；FCL 使用完整网格和完整精度，
PNG 使用完整网格。XZ 投影相交本身不能证明三维碰撞，红点来自 FCL。

## 定量重放

两次数据分别有 20 个唯一候选关节姿态，另检查各自抓取起点（共 21 个姿态）。
同一关节姿态会被多个槽位复用，不能把候选行数当作独立碰撞点数。

| 数据 | 名义盒碰撞姿态 | 当时包络碰撞姿态 | 抓取起点 |
|---|---:|---:|---|
| G2 live_03 | 16/20 | 20/20，历史大包络 | 无碰撞 |
| G4 live_02 | 16/20 | 19/20，当前保守包络 | 无碰撞 |

以上碰撞均涉及 link1/2；本轮几何重放实际检查 link1..6 的全部 collision mesh，
未应用 ACM 豁免，没有修改线上允许碰撞规则。没有检查底座、传感器、货柜、
物理真值箱体及节点之间的连续运动，不能宣称整条路线合法或物理覆盖通过。

当前 state 1 的部分 FCL 接触样本（world，m）：

| 碰撞对 | X | Y | Z |
|---|---:|---:|---:|
| Attachment / link1 | -0.322366 | 0.614451 | 0.955957 |
| Attachment / link2 | -0.200287 | 0.592869 | 1.075523 |

接触样本每个 mesh/box pair 最多 16 个，非全部交线。完整坐标、关节角、槽位对应见
[g4_replay.json](g4_replay.json)、[original_g2_replay.json](original_g2_replay.json)
及 [逐姿态碰撞表](state_collision_table.csv)。

## Z 方向核对

工具坐标 +Z 在这些朝下姿态中对应 world -Z。名义盒工具 Z 范围约为
[0,217.541] mm，当前保守盒为 [0,244.295] mm。近吸盘面不动，远端增长
26.754 mm（10% 高度 + 5mm）。近端数值约 -0.000002mm 是浮点/原始变换误差。
因此当前包络没有靠向上增长来制造这些碰撞；名义盒本身在 16 个候选姿态就碰臂。
W/D 最终增加不止 10%，因为还包含绕吸附点 ±5° 的连续 yaw 外包。

## 新发现：尺寸一致不等于附着位姿一致

在 live_02 Select 前的完整 scene 中，pickup_box 的 header.frame_id 与 link_name
均为 suction_contact_frame。按 object.pose × primitive_pose 解码后，中心在工具系约为
[0.575029,-0.582682,1.077473] m；Attachment 中心约为 [0,0,0.122147] m。
两者中心相差 **1.258103m**，存储坐标轴旋转差约 179.028°。
长方体存在轴符号等价，不能只用旋转差判断几何误差；中心偏移已经足以证明不一致。

这是直接观测。源码机制是 planning_scene_client.attach_pickup_box() 丢弃 xyz/quat，
走 move_world_object 语义；vacuum backend 传入的保守位姿没有被此调用直接采用。
仅凭这些证据，尚不能确定原 world 物体为何得到该偏移，也不能把快照问题等同于
全部建路/认证过程都用错位姿。route_producer 的 carrying scene 会从 Attachment
构造 PayloadSpec，后续可能替换场景，因此图中分开显示这两个来源。
此处需要后续修复并做场景级位姿一致性验收，不能继续只比较尺寸。
此外，原报告的两处 nominal-height 锚点输入遗漏仍未修改。

## 输入、版本、复现

- 调查基线：c3dc2e59；G4 live_02 运行版本 6ebaac7c。脏态与环境沿用各 run manifest。
- 当前输入：../live_02/selection_boundary/place-v2-2/request.json、planning_scene.cdr、
  ../live_02/robot_expanded.urdf、selection_response.json。SHA256 保存在 g4_replay.json。
- 历史输入：../../payload_envelope_g2_20261022/live_03/ 的 request、URDF、候选响应和
  offline_anchor_payload_collisions.json；未用当前策略重算历史包络。
- FCL 探针源：../../payload_envelope_g2_20261022/collision_viz/contact_probe.cpp。

在集成 worktree 根目录复现（离线，不消耗新的闭环尝试）：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
c++ -std=c++17 docs/status/evidence/occ_owner_b1/payload_envelope_g2_20261022/collision_viz/contact_probe.cpp $(pkg-config --cflags --libs fcl) -o /tmp/payload_contact_probe
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 scripts/g3/visualize_attachment_envelopes.py --output /tmp/attachment-envelope-replay
```

检查历史 16/20 与 20/20 结果的断言已通过；scene 与 Attachment 的尺寸精确相等
断言已通过。已人工查看三层对照图、当前碰撞图及历史 slot 332 图。
这验证了离线姿态碰撞复现，未重新执行完整搜索或新增功能闭环。
