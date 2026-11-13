# 四项开发与整链验证记录

- code_revision: 4e9b9622367d33020fbcd0bef4661468418e53a8
- plan_revision: 6c83aeda659b2a39fca64fb960fedf7678f88869
- environment: ROS Humble / install-production / 单 Gazebo 完整模型
- scope: 连续放置准入、观测几何稳定、LiDAR 结构回波处理、成功轨迹复用
- functional_result: PASS（三箱 floor / floor / stack）
- physical_payload_qualification: FAIL（第 2、3 箱真实外形部分超出载荷包络）

## 实现及边界

| 项目 | 实现 | 验证与限制 |
|---|---|---|
| 连续目标准入 | `selected_pose_admission.py` 对原始目标 XYZ 和四向 yaw 直接检查包络边界、部分体素覆盖、支撑和同源已知空闲 XYZ；compute/search 与目标复核调用同一谓词。目标不必重新落入枚举格点。 | harmless 2.108 mm 变化、四向平地/叠放通过；小尺寸变化造成真实穿透、进入未知或破坏支撑仍拒绝。未降低原有边界余量。 |
| 抓前几何稳定 | `StationaryGeometryWindow` 按同箱/代次累计已观测侧向范围和底部；保留原始采集时间，输出近期中位数和跨度。当前顶面仍取本帧实测接触平面，吸盘支持区域不从箱体包围盒推导。vacuum-on/attached 结束此轮静止历史。整链继续使用冻结模型，吸附后不改变抓取关系。 | 处理重复性和遮挡缩小；不能保证未观测外轮廓的绝对准确性。新发现的物理包络不足见后文。没有实现“忽略所有小于 2 mm 的变化”。 |
| LiDAR 结构回波 | 新增显式标定 CLI 和 `StructureAssociation`，绑定 ULD 几何、传感器 frame、安装身份和实测空结构样本。原 XYZ、时间、射线终点全部保留，仅追加 `structure_terminal` 标签；O 对这些端点跳过 occupied 更新。 | O 原生输入门验证两点中一个标签使 occupied hit 从 2 变 1，普通障碍保留。不得把标记端点强制置 free，不清空后方，不删除 ULD 碰撞模型。未覆盖/歧义/标定失配保留原回波。当前默认未加载标定，真实 LiDAR 数据资格尚未完成。 |
| 成功路径缓存 | `SuccessfulPathCache` 仅在成功、运动终止、无剩余载荷时保存原关节路径与时间。键绑定机器人/ULD/场景身份、起始关节、抓取和释放工具位姿。当前候选重新绑定当前几何、观测与地图并完整认证，拒绝则回到正常规划。 | 三次实际执行均触发持久化，证据中保存了对应的三份缓存提案。确定性测试覆盖命中、尺寸变化后认证、拒绝与回退、损坏及身份不匹配。不保存执行许可；尚未测得实际重复任务中的命中率/提速。当前仅支持精确端点匹配。 |

入口与参数：

- 新逻辑接在现有默认 `joint_operation` 链路，未新增一条旁路 planner。
- `successful_path_cache_directory` 默认 `~/.ros/luggage_successful_paths`；空值关闭。
- LiDAR 用 `operation_lidar_structure_calibration_file`、`operation_lidar_structure_sensor_mount_id` 显式配置。无标定时保留普通回波。
- 标定工具安装为 `calibrate_lidar_structure.py`，输入实际采样的 `points_container`、`origins_container` NPZ 和明确的结构平面/来源身份。未使用箱型目录或仿真真值作为在线输入。

## 保留旧目标的实证

`replay_diagnostic53_admission.py` 读取历史 diagnostic_53 的原始同源 O 页面与目标，校验原始 SHA。箱深刷新增加约 2.108 mm，时间间隔约 1.716 s。

- 旧逻辑：32 个原目标全部因重新枚举成员资格失败而拒绝。
- 新逻辑：28 个原 XYZ 原位保留。
- 其余 4 个：物理上仍在 ULD 内，但更新后最小 XY 余量约 19.265 mm，低于已有的一格、即 20 mm 的 hull margin；旧尺寸时约 20.319 mm。
- 因此 4 个拒绝不能描述为真实碰撞；它们确实改变了当前余量计算。没有为这次案例增加特判，也没有擅自删除旧余量。

详见 `replay_diagnostic53_admission.json`。目标准入一次包括 XYZ 前缀构建约 0.036 s；这只是该案例测量。

## 测试和失败修复记录

- 最终受影响 Python 行为回归：139 passed，命令见 `verify_affected.sh`，结果见 `verification_final.log`。
- 原生 C++ observer initial/keyset：2 passed，见 `native_structure_tests_enabled.log`。
- 实际已安装 O 的 ROS 输入验证：1 passed，见 `native_wire_02.log`；输入/回执 CDR 在 `native_wire_02/`。
- 曾发生模块命名冲突：误覆盖原有生命周期 `placement_admission.py`；广回归发现后已从基线逐字恢复，连续谓词移至独立 `selected_pose_admission.py`。
- 旧 H3 测试缺少必需 XYZ 已知空闲证据，在原始 `ed2c47f4` 查询代码下也同样失败；保留 `legacy_hull_baseline_control.log`，未放宽产品准入规则。
- 更新了旧目标复核测试中“偏移 1 mm 即拒绝”的过期断言，同时增加真实越界拒绝断言。尺寸改变后的缓存测试重新组合当前场景、验证当前几何传到认证接口。
- 环境/测试工具错误均保留日志：覆盖 ROS PYTHONPATH、ROS launch 插件导入无关测试、测试夹具字段名和场景版本未同步等；修正后完成上述最终门，不计为产品成功样本。

## 完整模型运行

运行命令、原始输入哈希与 dirty 状态在 `simulation_01/run_manifest.json`。源码与安装对应关系在 `build_01/manifest.json`。

- 冻结 1,965 个源码文件，534 个源码/安装文件一致。结束后安装内容无变化。首次审计仅两个本轮测试文件更新；收尾期间另有并行源码清理，后续审计逐项记录，不混入本轮提交。运行使用的冻结输入另有完整归档。
- 模型 SHA：`51df00b411c9bbf57390a2a256828d195b637becd2b3ab31a5f77a49005624a5`。
- 场景 revision：`413055f4740a733fb8ef18867db86adb`；沿用用户确认的当前 deploy_update 源配置。
- atlas 身份匹配，状态 `REVIEWED_IDENTITY_MATCH_REQUIRES_ONLINE_VALIDATION`，仍逐条在线验证。
- RGB-D / 原生 O，分辨率 2 cm；显式空舱初始化与实际观测来源均有记录。未把请求位置写成已观测障碍。
- 三箱共用 epoch=2；任务模式依次 floor、floor、stack；三次返回 `OBSERVED_OPERATION_COMPLETE`。
- 三箱 prepare 分别约 77.883 / 39.613 / 56.296 s，整个 cycle 约 908.558 s，含记录开销。不是生产时延达标结论。
- 完整 SHA/长度核验：driver 265、prepare 753、execution 5,831、reference 85 个记录，零损坏。
- 仿真和两个参考记录进程正常退出，teardown=0；结束时未发现 Gazebo / O / 本轮规划节点残留。

## 物理包络复核未通过

`review_payload_containment_v5.py` 从实际释放模型的 body + 四个 rim 碰撞体取 40 个顶点，在实际加载期间按相位及约 10 秒间隔的物理姿态样本检查；参考数据仅离线评估使用。

| 箱体 | 参考样本 | 包络外样本 | 最大超出量 |
|---|---:|---:|---:|
| 1 | 7 | 0 | 全部在内；最小采样余量约 3.059 mm |
| 2 | 7 | 7 | 6.630 mm |
| 3 | 15 | 15 | 6.143 mm |

直接观测：三箱真实外形为 0.55 × 0.40 × 0.25 m，冻结测量的长宽约为 0.496–0.503 × 0.359–0.370 m；第 2、3 箱在实际吸附关系下存在外形未被包住的部分。动作成功不能证明该模型合格。

源码建立的机制：既有 `luggage_box_estimator.py` 的长宽来自顶面 RANSAC 内点的矩形拟合；新多帧逻辑只累计已有测量，不能保证顶面等于全箱外轮廓。已有 10% 等包络参数并非本轮新增，但本次证据说明它们不足以保证这些输入的真实覆盖。

仍待定位：顶面/分割裁边、未观测凸出结构、位姿/中心误差各占多少；新历史融合是否改变其中某些误差的分布。本轮第 2、3 箱原始同步感知窗口未完整保留，不能把顶面拟合确定为唯一根因，也不能据此宣称普遍误差界。

捕获的约 99 万条物理接触消息未出现机器人接触对；包括箱子/平台、底层箱/ULD 和第三箱/第一箱 rim 的接触。只能陈述捕获范围，不是无遗漏碰撞证明。最终三箱姿态稳定，第三箱确实由第一箱支撑。

`simulation_01/containment_failure_manifest.json` 明确区分“包络边界可重放”与“原始感知根因重放资料不完整”。不以动作成功覆盖该失败；未调大参数以迎合仿真真值。

## 测时与下一步

一次确定性 1 cm 全空闲 149 × 197 × 148 地图枚举：旧查询约 4.047 s，新查询约 6.156 s，前缀构建约 0.031 s。连续复核增加了完整枚举成本；收益首先是避免旧目标因格点变化被无意义作废，不能声称全局加速。见 `benchmark_placement.json`。

后续优先级：

1. 为上述包络失败补充“原始 RGB-D/TF → mask/cargo cloud → 单帧顶面/外轮廓 → 历史输出 → attachment”的同步窗口，再定位绝对尺寸和中心误差。完整箱体外轮廓与吸盘接触域应分开建模；不足观测需要补视角，而非猜全箱尺寸。
2. 用实际空 ULD LiDAR 采样做结构标定及独立保留样本验证，明确覆盖范围与误抑制，再启用该源；目前没有这项资格证据。
3. 实测重复任务的缓存命中、完整再认证成本和回退成本，再确定近似路径修复/搜索优化是否值得。当前不能给出复用提速比例。

这轮四项代码开发与既定功能闭环已完成；真实载荷几何资格和实机基线尚未完成。
