# 统一几何参考：感知面 × 抓取接触面 × 物理碰撞面（2026-10-19，离线）

任务（用户冻结范围）：在同一箱体坐标系下列出三个表面与 TCP 接触平面的对应，
用 r5/r6 实录尺寸与位姿计算；判定修哪一侧；**未施加任何补偿，未跑在线复验**。
分析工具 `scripts/g3/unify_box_geometry.py`，数据
`unified_box_geometry.json`（同目录）。

## 对应表（inch20 loafbrr，共底参考系；r5/r6 世界 z 一致到 ±0.7mm）

| 面 | 盒系 z（相对模型原点） | r6 世界 z | 来源 |
|---|---|---|---|
| 碰撞底＝真实底（mesh z_min） | −0.1100 | 0.5826 | STL AABB＝目录尺寸 |
| **lid 平台面＝感知顶**（顶带众数 1mm bin） | +0.0985 | 0.7911 | STL 顶点直方图（可观测参考） |
| **物理碰撞顶**（全外框 AABB 冠） | +0.1100 | 0.8026 | release.sdf `<box> 0.35×0.53×0.22`（tier manifest primitive_collision_size） |
| **吸盘接触平面**（tf suction_contact_frame） | +0.1078 | 0.8003 | tf_trace after_descend |
| p04 板底最低点（接触流实测） | +0.1084…+0.1100 | 0.8010…0.8026 | contact_stream 流形 z |

关键判读：
- **共底成立**（用户的第一区别点）：可观测高度 0.209＝lid 到 AABB 底；两参考
  共底，顶差＝lid_offset＝**11.5mm**（非 5.75mm 共心情形）。spawner 公式
  `observable_top = origin + size_z/2 − lid_offset` 与此一致。
- **lid 冠区不是空的**：AABB 冠下有 9.6% 顶点（387/4047）分布在整个顶面
  （XY bbox ≈ 全顶面）——圆穹浅拱，仅在冠线达到 +11.5mm。
- **吸盘停在 lid 上方 8.6–9.3mm，从未接触真实箱面**；它"陷入"的 1.6–2.9mm
  全部是 AABB 碰撞盒虚构的冠部平板（把圆穹填平）。
- 接触流无 depth 字段（Fortress 对 ODE 不填充，protobuf 文本省略默认值）；
  穿透量由几何重叠实测：**swap 初态 p04 板底低于碰撞顶 1.6(r6)/2.4(r5)mm
  ＝释放初态非法穿插**（用户测试项 4 的违例）。
- 接触仅 p04（collision_4）单一 lump，无侧管/邻件；18/22 条 @1kHz＝swap 后
  2–20ms，与"动建体时刻弹射"签名吻合。

## 判定：修碰撞资产侧（不修触底目标，不加硬编码补偿）

1. **触底目标（lid 语义）正确**：感知顶＝lid 平台是相机可测面；实测吸盘
   停在 lid+9mm（触底从未压真实箱体）。把触底加深 11.5mm 会真的压箱；抬高
   则悬空——两个方向都错。
2. **碰撞盒把圆穹填成 11.5mm 冠部平板是唯一虚构物质来源**：箱体释放碰撞体
   `collision_kind="box"` 按全外框 AABB 取值（tier manifest
   `primitive_collision_size [0.35,0.53,0.22]`），把顶面声称到冠线。
3. **附加合同违例（用户第 3 项）——swap 尺寸跳变**：carry 期碰撞＝mesh STL
   （真实圆穹顶），释放体碰撞＝AABB 盒；swap 瞬间碰撞顶从"局部穹面"跳到
   "+11.5mm 平板"。穿插正是这个跳变的直接后果。
4. 已存在的设计债（无需新修，仅记录）：吸附门 `contact_gap_min=-0.02`
   （"attach often sits ~1 cm into the cube"）与 bitmask 方案
   （carry 0xFE 隔离 p04、释放 0xFF "collides with p04 again"，均为设计内）。

## 修复方案（落在错误的一侧；待实现）

**释放/放置碰撞盒顶面＝lid 平台、共底锚定**：
`<box> 0.35×0.53×0.2085 </box>`＋collision `<pose>0 0 −0.00575</pose>`
（XY 保持全外框——侧鼓是真实物质；只削顶）。来源单一化：由
`resolve_observable_reference`（mesh 实算）产出每 tier 的
`lid_offset`，tier manifest 登记高度与底锚；`suitcase_sdf` box 分支与
`_placed_primitive_size` 消费同源。效果：
- 释放初态：吸盘（lid+9.3mm）与碰撞顶（lid+0）净空 9.3mm——**按构造无穿插**，
  对下降深度不敏感；
- swap 尺寸跳变消除（mesh 穹顶 vs 平台面差 ≲11.5mm 仅剩穹拱局部差）；
- 静置高度不变（共底，底驱动 resting）；规划/占用用可观测几何，不受影响；
- 代价（明示）：叠箱场景下穹拱区欠声明 ≤11.5mm——当前流程无叠箱（地面槽位），
  记录为已知边界。

不变项：触底语义、bitmask 方案、40mm 门、占用合同、规划预算。
测试范围（用户第 4 项）：共心 vs 共底参考判定、多 tier（inch22 lid_off
12.0mm）、实录检测误差情形、释放初态零穿插断言、支撑面独立核对
（settle z＝floor＋底半高）。
