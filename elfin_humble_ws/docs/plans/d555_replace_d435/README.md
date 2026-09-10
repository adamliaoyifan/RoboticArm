# D555 替换仿真 D435

> **Discussion notes, not the executable plan.** Datasheet/TF facts stay
> here. Implementation of the Gazebo depth-image path is
> [`d555_sim_depth_pipeline_execution.md`](../d555_sim_depth_pipeline_execution.md)
> (DSIM-1…INTEGRATION). HB-1/2/3 already ran. Camera-body question
> `Q-20260908-2` is closed.

Status: discussion / reference. Not architecture.

对齐目标是已装的 **RealSense Depth Camera D555 PoE**（SN `419222302385`），不是 D455，也不是 D435i。本目录是 D555 的官网参数、TF 约定、以及「仿真 D435 → 真机 D555」的替换讨论。

`docs/plans/d455_replace_d435/` 是误按 D455 写的参考，实现不要跟它走。

机械支架按 D555 设计，安装本身不是问题。未决的是机体原点、流格式、DDS 传输和 `pickup_observe` 近距。

## 结论（先读这个）

仿真不能把 D435 `rgbd_camera` 改名成 D555 就算对齐。

1. **光机**：D555 用的是与 D455 相同的 **D450** 模组（95 mm 基线，RGB 在左右 IR 之间）。`camera_link` 仍是左 IR / 深度原点。官方 **没有 D555 URDF**；壳体 167×42×48 mm、337 g，不能抄 D435 或 D455 的安装孔偏移。
2. **近距**：VGA Min-Z ≈ **26 cm**，满分辨率 ≈ **52 cm**，理想 0.6–6 m。`pickup_observe` 下 0.80 m 箱顶约 **0.24 m**，VGA 仍可能进盲区。
3. **流**：稳定实测 **640×360@15**（16:9）。仿真是 640×480@30（4:3）、clip 0.105–3.0 m。896×504@30 会掉 DDS 设备。
4. **传输**：PoE RJ45 + SafeDDS，jumbo 9000，无 USB 取流。话题实测前缀是 `/camera/d555/...`，不是架构表里的 `/camera/...`。
5. **TF**：机内光学 TF 由 `realsense2_camera` 发布。有报告 D555 driver 把 `infra2` 的 Y 符号发反。HB-1 要 dump 真树，不要用名义 URDF 当真值。

活体事实收集见 [`d555_hardware_bringup_verification.md`](../d555_hardware_bringup_verification.md)（HB-1/2/3 **已完成**）。机身决策已关闭：仿真改靶走 DSIM，不要再按 `Q-20260908-2` 排队。

## 文档

| 文件 | 内容 |
|---|---|
| [official_params.md](official_params.md) | Datasheet v1.1、产品页、D450 模组 TF、点云 / RGBD / `camera_link` |
| [replacement.md](replacement.md) | 仿真 D435 换成 D555 的选项、文件清单、验收 |

规范行为仍以 `docs/architecture/sensor_data_pipeline.md` 为准。
