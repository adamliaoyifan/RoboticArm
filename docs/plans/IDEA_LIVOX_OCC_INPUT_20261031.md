# IDEA：OCC 几何输入切换 Livox Mid-360S（未派工、未授权执行）

- date: 2026-10-31
- status: idea（用户 2026-10-30/31 讨论定调：**当前 focus 保持 D555 深度相机**；Livox 为后续升级方向）
- motivation: 深度相机运动中深度质量差（运动模糊/主动立体失配，物理性）；用户希望资源允许时**运动中持续观测**（更覆盖），Livox 拖影不严重、畸变可消除
- parent campaign: OCC-QUERY-INDEX-20261025（gen7 收口后的方向讨论）

## 目标形态

```
Livox Mid-360S → 占据几何（运动中连续积分，去畸变后）
D555 相机     → 语义/标签（label/instance 层需要彩色；停稳快照）
                ↑ 双传感器分工，对应 store 内部占据/标签分层
```

## 已核实的关键事实（2026-10-30/31）

1. **SDK 无内置去畸变**：livox_ros_driver2 只发布逐点时间戳点云 + 内置 IMU（ICM40609, 200Hz）。包时间戳=包内首点时刻。
2. **官方参考实现**：Livox-SDK/livox_cloud_undistortion（IMU 基）——基于老 driver1，需移植适配。
3. **IMU 坑**：内置加速度计单位是 **g 不是 m/s²**（DLIO issue 实测），SI 算法会算错。
4. **推荐方案：FK 去畸变**（传感器装臂上，动源=臂，运动学精确已知）：
   - 残差 ~0.5mm / 100ms 窗（1ms 插值 × 0.5m/s）= 0.03 个 2cm 格；无积分漂移、无单位坑、静止时退化为恒等
   - 工作量 ~百行（向量化 slerp+平移插值），joint_states ~1kHz 插值
   - 误差预算对照：未补偿 3–10cm（0.3–1m/s × 100ms）不可接受；FK 补偿后亚格
5. **连续积分的资源账**：瓶颈是 OCC 写路径 145ms/笔，不是 CPU/内存（派生内存实测 71.6MiB/预算 512）：
   - 10Hz 连续 = 145% 占空比（不可行）；5Hz = 73%+并发查询（勉强）
   - **前置条件 = 写路径优化战役**（序列化移出关键路径 + 派生索引增量维护）——与 gen7 遗留的 distance 2cm 稳态 52–57ms→50ms 门属同族，一次优化两处受益

## 已有底子 vs 缺口

| 已有 | 缺 |
|---|---|
| 冻结外参 elfin_base_link←livox + T_livox_to_camera（桌面 ICP 冻结件） | 实时驱动接入（本仓无 livox driver 包） |
| colorized.las 彩色化扫描 + eval 工具（dsim_livox_metrics / lidar_camera_calib_export） | 逐点时间戳链路 + FK deskew 组件 |
| 5Hz 频率预算兼容（10Hz 隔帧取用） | DDS QoS 设计（~6MB/s；D555 大帧丢包前科） |
| — | 双传感器时间对齐（D555 标签 ↔ Livox 几何 stamp 配对） |

## 决策记录

- 2026-10-31 用户：Livox 讨论落盘为 idea；后续以其输出替换相机几何输入；**当前继续 focus D555 深度相机**（motion_gate 停稳积分模式不变）。
- 若启动：建议派工顺序 = FK deskew 组件（可先离线用已有 .las 数据验证）→ 驱动接入+DDS QoS → 写路径优化战役（与 distance 主线合流）→ 连续积分灰度（先 5Hz）。
