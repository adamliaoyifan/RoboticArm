# 释放位姿格式修复与一轮闭环验证 — 2026-10-23

## 结论

格式修复已完成：计划 `015a5b90`，冻结产品 `857292cbb61b4a6908319c0421cbaff77c754b57`。54 项相关测试通过，构建及安装/源码一致性通过；原始实抓回放177个位姿全部接受，无解析拒绝。

一轮 n=3 套件已执行并按首错停止：尝试2件、抓取2件、装箱并提交验证1件，第3件未尝试。套件726.1s，退出1，未通过 n=3。没有追加第二轮，也没有在运行中改代码、裕量或5mm门限。第一件成功不构成可靠性提升或物理接触安全证明。

## 修改

`luggage_gazebo.pose_text` 是在线缓存和离线回放共用的单一解码模块。私有 Protobuf 描述符按本机 ignition-msgs8 的 Pose_V/Pose/Header/Time/Vector3d/Quaternion 字段定义构建；protobuf负责字段名、层级、顺序、数值语法及省略的默认零值。

每个 header 重置时间戳；缺少必要结构、无效时间戳、非有限位姿和非单位四元数具名拒绝。原始记录不变；presence_evidence增加解码计数/拒绝细节。保留原有模型身份、freshness/epoch策略及规划/物理链。package.xml明确python3-protobuf依赖，新增行为测试与邻近格式说明。

## 验证与环境

- 命令及验收：`docs/plans/RELEASE_POSE_FORMAT_20261023.md`。
- `offline_tests.log`：格式、在线/离线一致性、freshness拒绝、kinematic释放、原始采集及释放失败契约，共54 passed。
- `build.log`：colcon build --packages-select luggage_gazebo 成功。
- `raw_replay.json`：安装模块与源码一致；上次实抓177个位姿、3个预加载箱体样本解析成功。
- rehearsal_01：无箱子尝试；单clock publisher，相机10.76Hz，gain0.8，正常退出/清理。
- 正式live_01：2026-10-23 10:29:34开始，10:29:47 driver开始，10:41:56 driver退出，10:41:57清理结束（Asia/Shanghai）。ROS_DOMAIN_ID=7，A-prime，seed0，inch20，prioroff，Select预算180s，attachment60s；几何配置hash见manifest。只有预存未跟踪pydeps-frozen链接，产品文件无脏修改。
- 运行命令：PHASE2_RUN_OUT=<此目录>/live_01 ELFIN_RELEASE_CAPTURE_DIR=<此目录>/live_01/release_capture bash scripts/g3/n3_functional_run.sh。

## 两件结果

| 箱体 | 选路 | Select | 认证/执行段数 | 释放 | 闭环 |
|---|---|---|---|---|---|
| pickup_box_0001_inch20 | direct，slot13 | 13.276s | 8/8 | 声明/世界位置误差记录为0，fresh，解码无错误 | COMMITTED→VERIFIED→HOME，549.13s |
| pickup_box_0002_inch20 | 首候选direct失败、staging端点不可用；后候选direct，slot1 | 12.039s | 8/4 | fresh，但声明/缓存位置相差0.051973m，超过0.005m门限 | ABORT_RELEASED，未提交 |

完整机器可读结果：`live_01/metrics.json`，原始suite/trial、request/response/scene CDR均保留。

## 新失败的观测与假设

直接观测：第二件在线解码累计接受1262451个位姿、19878个header，无解析拒绝。set_pose返回ok；被采用缓存stamp729.582s，检查时sim_age12ms、wall_age11.2ms，因此通过既有freshness门，却因位置差51.973mm拒绝。

独立原始订阅器保留的下一条目标位姿stamp729.616s，比被采用缓存晚34ms，与声明位置一致（相对trial中6位小数声明值的差约0.000000638m）。该消息在abort之后到达独立记录器。见第二件release_capture/offline_replay.json、events.jsonl、chunks.jsonl和poses.raw。

源码机制：连续缓存从第一件释放起已运行；第二件set_pose之后读取任意符合freshness的缓存，不要求它晚于此次位姿变更。fresh不等于对应本次set_pose。上述证据高度支持“读到更新前缓存”的竞态解释，但缺少set_pose请求/响应/应用的精确时间，以及被拒缓存的完整pose7，不能声称已经唯一证明根因或箱体持续偏移52mm。后续若验证此假设，应先补齐这些边界记录和命令/样本关联屏障，不应靠放宽误差门限或盲目重跑。

## 证据覆盖与限制

- 两件的boundary replay manifest均标记capture_complete/replay_possible，scene、surface、joint ring、输入输出引用已保存；该声明只适用于原有边界包，不代表释放命令竞态可被完全重放。
- 第一件poses.raw达到8MiB上限，完整解出827个目标样本，末尾partial block对应一次truncated_message，属于落盘上限，不是在线解码错误。pose采集覆盖约391.777–405.826s；不覆盖整段返回/最终settle。记录器90s自停。
- 第二件poses.raw10566字节，完整解出1个目标样本；abort后停止记录器，无字节截断。没有拒绝时使用的原始完整pose7，释放竞态精确重放为not_evaluated/incomplete。
- plugin recorder在600s WALL_DEADLINE关闭，早于第二件释放；plugin_write存在8503条queue drop（其余read/iteration queue drop为0）。不能声称插件流全程完整；原有driver失败joint ring独立保留。
- 两个contacts.raw均为空，contact_absence_proven=false；不得据此宣称没有物理碰撞。
- `live_01/teardown.log`记录只停止owned process group；最终进程审计未发现Gazebo/parameter_bridge残留。
- 原始大bin本地完整保留，并复制到主工作区；Git保留其大小/hash及其他证据文件，避免提交大bin。
