# D455 替换 D435 / D435i

Status: discussion only. Not architecture. Not dispatched.

## 目标已改为 D555 PoE（2026-09-08）

**不要按本目录去实现 D455。** 实机装的是 **D555 PoE**（SN `419222302385`），
用户已确认对齐目标是 D555，不是 D455。本目录的 D455 参数、官方 TF 和选项
A/B/C 降级为**参考资料**；其「选项 A：不上 D455」的结论已无意义，因为 D455
从来不是装在机上的机身。

D555 实测规格与本目录若干前提冲突（数据来自 D555 Datasheet v1.1 与
`20260902_183500_eef_livox_d555/NOTES.md`）：

| 项 | 本目录原假设 | D555 实际 |
|---|---|---|
| 机身 | D435 90 mm 口袋，D555 共用 | **167 × 42 × 48 mm，337 g**；支架本就是按 D555 设计 |
| Min-Z | 只讨论 D455 的 52 cm | **~26 cm (VGA)**，满分辨率 ~52 cm，理想 0.6–6 m |
| 接口 | USB，`realsense2_camera` | **PoE RJ45 + SafeDDS**，无 USB 取流选项 |
| 数据流 | 仿真 640×480@30 | 实测稳定 **640×360@15**；896×504@30 会**掉 DDS 设备** |
| 发行版 | Humble / 22.04 | 快照 bring-up 是 **native ROS 2 Jazzy** |

**机械安装没有问题**（支架按 D555 设计，已确认）。问题在模型和数据来源：本
目录与 `realsense_d435.urdf.xacro` 注释把支架描述成「D435 90 mm 口袋」，这
对一个 D555 支架是错的；而 `eef_mount_adapter` → `camera_link`
（`0.013 0.097 -0.021`）出自 `eof_mount_stack_tune_gui.py` 的**目视调参**、
且是按 D435 机体原点调的，不是标定值。D555 的机体原点在同一支架里位置不同，
所以该外参需要重新标定，`camera_link` → `d555_link` 的单位阵也应改为**定义**
而非调出来的关节。

待决问题在 `docs/agents/discuss/2026-09-08_1643_d455-sim-parity-and-min-z-conflict.md`
（`Q-20260908-2`）。相机机身决策排在 F3（`Q-20260908-3`）之后。

## 以下为 D455 参考资料（不再是目标）

Intel 没有 D455i 这个独立料号。带 IMU 的型号就是 **D455**（Bosch BMI055），对应关系是 D435 → D435i。本目录把官网参数、官方 TF、以及「腕部 D435 换成 D455」的替换路径收在一起。

当前栈仍是 **D435 仿真 + D555 PoE 真机**。在机械支架、近距 Min-Z、以及 `pickup_observe` 工作距确认之前，不要把 URDF 或架构改成任何新机身。

## 结论（先读这个）

腕部相机 **不能** 把 D435/D435i 当 D455 即插即用。

1. 壳体：D455 长边 **124 mm**。（原文把 `arm_realsense_v1.3` 描述为 D435 **90 mm** 口袋并称 D555 共用；2026-09-08 更正：支架是按 D555 设计的，D555 为 167 × 42 × 48 mm。若要上 D455 需按实际支架重新核算。）
2. 近距：D455 满分辨率 Min-Z ≈ **52 cm**，推荐 0.6–6 m。`pickup_observe` 把 `camera_depth_optical_frame` 放在世界 **z = 1.9 m**、光轴朝下；平台约 z = 0.86 m。0.80 m 大箱顶面距相机约 **0.24 m**，会落在 D455 盲区里。D435 仿真 clip 是 0.105–3.0 m，这才覆盖该姿态。
3. 真机单元（2026-09-02 快照）用的是 **D555 PoE**，不是 D435i，也不是 D455。机械安装已确认（支架按 D555 设计）；待办是机体外参重新标定。

若仍要上 D455：先改支架并抬高/后移观察位，使箱顶工作距 ≥ 0.6 m，再改 URDF、yaml、launch 和真机预处理 overlay。话题名保持 `/camera/...`。

## 文档

| 文件 | 内容 |
|---|---|
| [official_params.md](official_params.md) | 官网/简报参数，官方 ROS TF，深度点云 / RGBD / `camera_link` |
| [replacement.md](replacement.md) | 替换选项、文件清单、仿真保真度、验收 |

规范行为仍以 `docs/architecture/sensor_data_pipeline.md` 为准。本目录只讨论尚未采纳的相机型号更换。
