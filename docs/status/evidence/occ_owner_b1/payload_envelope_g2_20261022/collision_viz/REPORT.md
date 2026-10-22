# G2 已保存中间姿态：Z 裕量消融与碰撞可视化

2026-10-22。离线分析基线84fe9c8d；live_03运行版本752e1835。
本轮只回答用户的碰撞方向问题并制图，没有修改在线配置或运行仿真。

## 结论

Z向上膨胀不是这组候选全部失败的决定因素。移除全部Z尺寸/位置裕量后，
20/20保存的中间姿态依然与二号臂相交；原始测量箱体自身在16/20姿态已相交。
Z尺寸裕量仍为每面50mm，加上位置误差5mm，目前上下各增长约55mm。
这并不意味着所有裕量都可删除，也不证明候选集外无解。

| 离线模型 | 与任一被测臂段相交的姿态 | 一号臂 | 二号臂 |
|---|---:|---:|---:|
| 原始测量箱体 |16/20|11|16|
| 完整G2包络 |20/20|20|20|
| 仅删除Z位置5mm，保留Z尺寸50mm |20/20|20|20|
| 删除全部Z尺寸/位置裕量，保留XY和yaw |20/20|16|20|
| 仅保留Z尺寸50mm及位置5mm |17/20|15|17|
| 完整包络仅裁掉向上55mm，下侧保留 |20/20|20|20|

“向上”针对保存姿态的根坐标系+Z；脚本断言所有工具+Z轴均指向下方
(dot < -0.999)，据此裁切工具包络的负Z面。这里的消融用于归因，不是合法新包络。

## 图和数据

- `collision_examples.png`：S01原始模型也相交；S10原始模型通过但包络相交。
- `all_20_staging.png`：全部20个唯一关节配置。蓝色测量箱，红色完整包络，
  橙色一号臂，黄色二号臂，灰色其余臂段，品红点为FCL代表接触点。
- `ablation_slots.png`：逐姿态消融矩阵及332槽位的容器坐标XY分布。
- `slot_state_mapping.csv`：逐槽位到保存关节配置ID的映射。
- `ablation_contacts.json`：完整精度关节、映射、每对碰撞体的接触点坐标及穿透深度。

332个尝试槽位中，331个EDGE_BUILD_FAILED共享20个唯一中间关节配置；
剩余1个NO_FEASIBLE_ROUTE行未保存这类中间点，不能为其补造接触点。
槽位目标位置不是接触点，图中分别展示。所有图均是未执行的离线候选姿态。
接触点为FCL网格/盒体算法输出，每碰撞体对最多16个，并非完整接触流形或撞击时序。

## 复现与验证边界

从worktree根目录执行：

```bash
c++ -O2 docs/status/evidence/occ_owner_b1/payload_envelope_g2_20261022/collision_viz/contact_probe.cpp -o /tmp/payload_contact_probe $(pkg-config --cflags --libs fcl)
PYTHONPATH=.:src/luggage_planning:src/luggage_description python3 docs/status/evidence/occ_owner_b1/payload_envelope_g2_20261022/collision_viz/replay_and_plot.py
```

输入为live_03原始request.json、selection_response.json、robot_expanded.urdf及
URDF引用的本地碰撞网格。固定G2模型参数在脚本内具名列出。20×6种模型对link1–4
精确网格使用native FCL碰撞检测；link5/6保留原有接触豁免，只在图中显示。
未包含世界碰撞、全轨迹连续检查或可达性验证。绘图完整使用碰撞网格，没有以示意
包围盒替代臂段碰撞计算。原始16/20与完整20/20结果断言匹配先前独立复核。
生成后人工查看代表图；脚本正常结束。没有改变产品/规划参数。
