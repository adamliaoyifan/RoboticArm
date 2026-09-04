# Todo 1 验收记录：YOLO 语义节点与 cargo 点云

日期：2026-08-27
launch：`ros2 launch luggage_gazebo sim_world.launch.py gui:=false use_rviz:=false use_semantic:=true`
位姿：pickup_observe（FJT 执行 SUCCEEDED），箱子 pickup_box_0001_gen
（GT 0.731×0.403×0.288，中心 (-1.000, 0.000, 1.004)，yaw 0）

## 交付物

| 文件 | 说明 |
|---|---|
| `luggage_perception/semantic_segmenter.py` | +`SegmenterOutput`/`update`/`copy_output`（深拷贝快照，`instance_map` 属性弃用） |
| `luggage_perception/scripts/semantic_segmenter_node.py` | 新：预处理 RGB → mask/overlay/instance + stats；`require_backend` 启动拦截 stub；`max_rate_hz` 限频；模型按 share/models 解析（杜绝联网下载） |
| `luggage_perception/scripts/semantic_point_filter_node.py` | 新：**精确 (sec,nsec) stamp join**（云×mask），无 slop、无 TF、无 fallback_to_raw；内参取自在线 camera_info；`extrinsics_source: identity\|config` |
| `ros_message_adapters.py` | +`mask_msg_from_array`(mono8)/`instance_mask_msg_from_array`(mono16)/`mono16_array_from_msg` |
| `config/semantic_segmenter.yaml` | 重构为 ROS 参数格式（两节点节），删 `sync_slop` |
| `CMakeLists.txt` | 装两个节点 + yolov8s-world.pt 到 share/models + CLIP vendor 到 share |
| `sim_world.launch.py` | `use_semantic` 参数（默认 false），detector 随之切换输入 |

## 验收结果（对照 plan 第 7 节）

| 项 | 结果 |
|---|---|
| mask stamp 与 color stamp 一致 | ✓（节点按构造继承 header；探针 74/86，差异为探针侧 best-effort 丢帧） |
| cargo stamp ⊆ 云 stamp，帧率 ≥ 80% | ✓ **91%**（向量化的 filter 下） |
| backend 非 stub | ✓ `bbox_fill:yolov8s-world.pt`，GPU 7.4 ms/帧 |
| cargo_count ≪ raw_count | ✓ 110,392 / 307,200 |
| DetectLuggage 返回 perception estimate | ✓ **5/5 稳定**（同一稳态场景逐次一致），conf=1.00 |
| 停 segmenter | ✓ cargo_points 停发，detector 报 `DETECT_STALE_CLOUD` + GT fallback，未发生整片点云被当 cargo |

## 检测精度（供 Todo 2 基线，不是本 todo 门槛）

| | 检测 | GT | 误差 |
|---|---|---|---|
| 中心 | (-0.974, -0.042, 1.004) | (-1.000, 0.000, 1.004) | **xy 4.9 cm**，z 0.0 cm |
| 尺寸 | 0.674×0.313×0.288 | 0.731×0.403×0.288 | 高度精确；w -5.7cm，d -9.0cm（欠估） |

Todo 2 的 tol_xy=0.03 目前差 2cm，尺寸欠估源自低阈值 bbox 裁边 + mask 不对称；
plan 已预留调参迭代（conf 阈值 / prompt / min_points）。

## 实测发现（已修入代码/配置）

1. **俯视深蓝箱在 CLIP 空间不像 suitcase**：裸类名 prompt（含 worldv2 m/s 检查点）
   对箱体检出为 0；场景描述性 prompt "luggage on a platform viewed from directly
   above" 才命中（箱体 conf 0.04-0.07，吸盘面板反而 0.28-0.35）。
   处置：阈值降到 0.04 让两者都进 mask，检测器 RANSAC 按点数优势（箱顶 ~6 万
   vs 面板 ~1.2 万）选中箱顶平面--实测 5/5 落在箱子上。
   遗留：mask 含面板行；真机常规视角下裸类名有效，此为仿真俯视特化。
2. **`filter_points` 纯 Python 循环 30.7 万点/帧 ≈1.5s**：发布延迟把云年龄推过
   判龄窗口（DETECT_STALE_CLOUD）。已向量化（11 条测试全过），链路延迟恢复正常。
3. 检测器 `cloud_max_age_sec` 1.0 → 2.5（launch 内）：语义链在原始链路上多了
   预处理 4-6Hz + YOLO + 点滤波三级，实测年龄峰值 1.9s。
4. **CLIP vendor 在安装树下找不到**（dist-packages 布局比源码树多一层）：
   ultralytics 回退联网 AutoInstall 卡死。`_setup_clip_vendor` 改为向上逐级探测
   share 位置，vendor 目录随包安装。
5. 模型 checkpoint 安装到 share/models 并由节点解析，裸文件名不再触发联网下载。

## 遗留 / 移交 Todo 2

- 检测 xy 误差 4.9cm vs 3cm 门槛、尺寸欠估：调 conf 阈值、prompt、min_points
- 面板混入 cargo mask：长期解是 robot_self_point_filter（plan 刻意后置）
- 预处理 4-6Hz 仍是链路速率上限（Phase 5 C++ 化依据）

---

## 复测（2026-08-27 下午）：suitcase 网格资产

新资产：`suitcase_loafbrr`（拉杆硬壳）、`suitcase_vintage`（帆布复古）——单位 AABB
网格 + `mesh_meta.json` 原生尺寸，spawner 经 `suitcase_sdf()` 缩放生成 visual、
box collision 与惯量。本次 spawn：`visual=suitcase_loafbrr 0.590×0.430×0.295`。

### 链路指标（全部维持达标）

cargo rate 87%（≥80%）、backend `bbox_fill:yolov8s-world.pt` 7.9ms/帧、
cargo 109,645 / raw 307,200、检测 5/5 感知路径（间隔 ≥5s 时）。

### 检测精度（对比纯色箱，同一流程）

| | suitcase 网格 | 纯色蓝箱（上午） |
|---|---|---|
| 中心误差 xy | **0.6 cm** (-1.000, -0.006, 1.006 vs GT) | 4.9 cm |
| 中心误差 z | 0.15 cm | 0.0 cm |
| 高度 | -0.3 cm | 0.0 cm |
| 宽/深 | -6.5 / -5.3 cm（欠估） | -5.7 / -9.0 cm |

真实网格让 mask 更贴、中心误差从 4.9cm 降到 0.6cm（远低于 Todo 2 的 3cm 门）。
宽深欠估 ~5-6cm 依旧来自低阈值 bbox 收边，是 Todo 2 调参项。

### Prompt 结论（网格上复测）

裸 "suitcase" 在俯视图上仍为 **0 检出**（连 0.04 都不触发）--视角问题与纹理无关，
是 CLIP 对俯视箱体的固有盲区。描述性 prompt 仍必需；箱子 conf 从纯色箱的
0.054 升到 **0.204**（网格确实更像箱子），面板混淆依旧（0.23-0.24）。
当前配置（描述性 prompt + 阈值 0.04）在两种资产下都工作。

### 新发现：连续调用触发 STALE（GIL 饿死）

背靠背 detect 调用（间隔 1s）第 2 次起报 `DETECT_STALE_CLOUD`（检测器视角云龄
3.51s，而流实测龄 1.0-1.3s）：`estimate_box` 的纯 Python RANSAC 在 11 万 cargo
点上跑 ~2-3s，期间持有 GIL，订阅回调无法更新 `_latest_cloud`。间隔 ≥5s 全过。
修复方向：向量化 `estimate_box`（与 `filter_points` 同法）；Todo 2 的采样流程
（spawn→goto→等 geometry_ok→detect）天然有秒级间隔，不阻塞。

---

## 性能修复（2026-08-27 晚）：解码向量化 + estimate_box 体素

背景：上一节记录的 GIL 饿死（背靠背 detect 报 STALE）。四段计时证实瓶颈分布
与预判一致：sensor_msgs_py 逐点解码 ~1s 是最大头，_refine_rectangle 161 次
percentile 循环次之，RANSAC 本体（numpy 内核）只占小头。

改动：
- `luggage_detector_node`：解码改 `adapters.cloud_points_from_msg`（向量化，
  保留 isfinite 防御）；新增 `voxel_size` 参数（默认 0.01，0=关）；
  每次检测输出一段 INFO 级四段计时（read/tf/voxel/ransac/refine）
- `luggage_box_estimator`：新增 `voxel_downsample`（首点法，避免质心收边）；
  `estimate_box(voxel_size=, timing=)`，体素在 ROI+高度带裁剪之后、拟合之前
- 新增 4 条测试：体素开/关中心尺寸不变性、低大平面+高箱顶仍选高、
  面板混合双侧边界（25 点高平面不翻转 / ~2500 点高平面按高度规则翻转——
  真实边界是"提出概率×高度占优"，不只是 min_inliers）、粗体素饿死计数制
  置信度。套件 229 全过

实测（suitcase 场景，同一帧）：

| 段 | 修复前 | 修复后 voxel=0 | 修复后 voxel=0.01 |
|---|---|---|---|
| 解码 | ~1s（估） | **4.2 ms** | 2-5 ms |
| TF | ~1-2 ms | 2.5 ms | 1-2 ms |
| 体素 | — | — | 6 ms（82929→2358 点） |
| RANSAC | （合计 2-3s） | 280 ms | 24-32 ms |
| refine | （合计 2-3s） | 251 ms | 20-27 ms |
| **单次总计** | **~2-3 s** | ~540 ms | **~60-70 ms** |

- **背靠背 1s 间隔 ×3 detect：3/3 感知估计**（原 bug 场景修复；且 voxel=0
  的 540ms 也已低于判龄窗，解码修复本身即消除了 STALE）
- **精度 A/B：voxel 开/关结果逐位一致**（center (-1.000,-0.006,1.006)、
  size 0.525×0.377×0.292、conf 1.00）——首点体素对无噪仿真云的
  percentile 边界完全无扰
- `_refine_rectangle` 向量化按计划暂缓（体素后仅 20-27ms，远低于 100ms 阈值）
