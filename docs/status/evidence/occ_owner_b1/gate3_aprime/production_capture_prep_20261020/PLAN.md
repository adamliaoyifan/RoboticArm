# 单次生产诊断捕获准备：首条 transit + 保持窗

- base_revision: 550862ca
- dispatch_ready: no
- simulation_started: no
- historical_replay_admitted: no
- purpose: capture_new_same_run_inputs_and_responses

## 范围与证据语义

当前缺失的是历史实际输入证据，未发现“轨迹加速度字段有错”。日志字段
未保存与请求数组显式为空不同。`[]` 是合法的原始导数语义，记录和回放时
必须保留；不补零，也不按“缺加速度”拒绝一个已完整捕获的空数组请求。

旧 `trajectory_dump` 省略a，因此不再补猜2c的257点日志。新诊断从正常生产
入口生成并捕获自己的认证轨迹、提交与实际FJT goal；不是历史轨迹重建。
本文件是准备草案，未冻结仿真命令、未启动新运行、也未修改任何规划、速度、
等待、settle或安全门。新增捕获代码目前**未接入生产回调**。

## 已完成的离线捕获器验证

新增 node-layer `ros_boundary_capture.py`，无节点创建、无发送动作：

- 写完整CDR和从该CDR解码的完整字段JSON，保留空数组、全部header、
  原关节顺序、sec/nanosec整数及SendGoal中的16字节UUID；不猜goal ID。
- CDR与字段文件分别哈希；回读验证存储哈希、消息类型、全字段与UUID。
  不用重新序列化的字节作为恒等标准，避免CDR填充字节差异误判。
- 消息记录以临时目录写入、fsync后原子提交。目录不得覆盖；超预算报错，
  不截断消息；错误保持在health中。`recorder_io_complete`不等于整次运行
  `capture_complete`，后者必须经过边界清单和UUID对应检查。
- 12项离线测试覆盖真实ROS类型的空/零/非零加速度、typed admission到
  executor提交的空数组保留、认证请求、ExecuteTrajectory/FJT SendGoal、
  状态及带源stamp反馈、NaN effort、UUID冲突、预算耗尽、文件损坏及并发。
- `selfcheck_final/` 中15条**合成离线**记录已回读验证，不是实际机器人goal。
  其本机同步写耗时中位数约4ms、最大约50ms；仅是本机文件I/O样本。
  故不得把该同步写器直接用于50–100Hz反馈回调并宣称无采集干扰。

## 下一次诊断需要接通的真实边界

| 边界 | 必须保留的原始数据 | 关联与健康检查 |
|---|---|---|
| 认证输入/结果 | ValidateRouteV2完整请求/响应CDR（含knots、sample与scene）、认证摘要 | run/box/request、route digest、edge ID、cert ID、请求序号；全部认证尝试保留，不能只用覆盖别名 |
| Select输出/driver输入 | 完整Select请求/响应、选中首条transit的MotionSegmentV2 | 以edge/leg摘要对应认证；空导数数组也参与字段摘要 |
| executor提交 | wrap前后完整轨迹、live joints、shift向量、实际ExecuteTrajectory goal及UUID、结果 | 保存实际构造分支、原names/q/v/a/effort/ns/header；记录动态生效参数及安装哈希 |
| 实际FJT接收 | FJT action接收边界的完整goal与其原始UUID，accept/reject/result/cancel | 原接收UUID与完整goal组成SendGoal记录；必须来自真实边界，不能用executor副本替代。接收侧hook尚未实现/验证 |
| 两层action事件 | SendGoal、状态、feedback、结果、cancel及UUID对应 | 列明rejected/pending/accepted/terminal；无事件不能当作无其他goal |
| 物理反馈和控制器状态 | joint_states、controller_state原始消息与源stamp；/clock另存 | 高频数据采用独立连续录制，不阻塞生产回调；保存接收mono、录制丢样计数和源stamp范围 |
| 生命周期/收尾 | 原配置、源码/安装哈希、dirty diff、命令、peer/lease/vacuum状态、正常退出事件 | 实验外的模型/配置不得混入；采集失败也要写缺口清单 |

跨边界比较必须逐关节名检查顺序和全部字段，header与相对纳秒分开。
发现转换只报告转换及再验证证据，不自动归罪。每条数据保留来源层级，
不把synthetic、client构造、server接收混称“捕获实际goal”。

## 运行边界和正常收尾（需要先实现并离线验证）

1. 保持原生产pick、attach、退离、Select/认证与首条transit提交过程；诊断
   模式仅一箱、一次首条transit，无放置commit、无第二箱、无两箱计分。
2. 必须在driver正常控制流中识别**第一条**transit的terminal结果，成功后
   进入被动观察60仿真秒的诊断分支；不提交后续腿、不发送额外hold goal，
   peer/lease/正常安全监测继续运行。此分支尚未实现，不能靠外部进程猜时刻。
3. 首条transit被拒绝/中止/取消、触发现有安全退出，或原有等待超时，沿
   原正常错误/清理流程返回，不能为了凑满保持窗屏蔽安全退出。保持窗未达
   的原因与实际覆盖范围明确落盘，不强行判“未复现”。
4. 60秒观察完成后返回显式diagnostic完成结果；先flush并验证必需捕获，再
   调用正常vacuum/detach/clear流程。释放/删除事件属于观察后的收尾阶段，
   必须单列时间；不能在观察窗中删箱。收尾路径需离线行为测试验证一次性
   顺序与异常处理。不得杀planner、SIGSTOP或以服务失联/租约超时停环。
5. 总诊断提议上限900墙钟秒（包含启动、前序、transit和观察），不改任何
   生产execute等待参数；触顶仅通过正常取消/终态确认与收尾退出。具体正常
   取消适配和确认截止时间未验证，故本提议不构成可运行批准。
6. 不论复现、未复现或证据不完整，保留全部数据；不自动补跑。目标腿尝试
   数固定为1，捕获失败不产生额外名额。仿真资源遵守单栈检查与原stop_sim.sh
   teardown；证据flush验证在拆栈前完成。

## 度量、窗口与判读

新run记录revision/dirty、环境、配置/seed、分阶段边界与墙钟/仿真时间。
目标腿全程记录，专门分析其末端2仿真秒及terminal后60仿真秒；样本不足
时保留实际时段。分别报告每关节q-reference和q-endpoint的最大绝对误差、
RMS、末端越程、物理速度、保持段有符号斜率及其有效样本数、最大采样间隔。
既有settle条件（0.01rad、|v|≤0.01rad/s、3样本/0.3s）仅用于离线对照，
不改变在线门，不将action成功等同于物理到达。

- 这是一次条件诊断，不是可靠性或成功率认证；不设“连续通过”聚合门。
- 证据完整且复现：确定本次实际输入及所记录生产上下文下出现异常，之后
  才能拿**本次**确定输入做隔离重放，不自动证明转换层缺陷。
- 证据完整且未复现：只表明本次未触发；生产上下文差异仍开放，不自动签
  根因。不能拿新路线的时长去补旧run的goal。
- 缺任何关键边界、UUID不可关联、数据截断或源时钟不可对齐：
  `capture_complete=false`、`diagnosis=not_evaluated`，只报告覆盖与缺口。

## 存储与最终准入缺口

提议每次boundary预算256MiB、高频流256MiB、元数据16MiB，总计528MiB；
单次数据无论结果均保留，原输入不覆盖、不自动删除。预算不得靠省略字段
满足，任何超限记证据不完整并正常收尾。最终预算须在完整边界接通后的
非运动离线压力测试中验证；本轮仅证明单消息/小批次记录正确。

最终准入仍需：实际认证/executor/FJT hook接通，跨UUID关联验证，高频
采集与丢样健康门、正常单transit终止/收尾的行为测试、冻结启动命令与
有效运行参数、时间盒取消验证。这些尚未完成；**本次草案维持未准入**。
离线序列化通过不能越过这些缺口。

## 可复核的离线命令

```bash
source /opt/ros/humble/setup.bash
source /home/adamliao/work/elfin_humble_ws/install/setup.bash
source /tmp/elfin-master-pack-baseline-g3/install/setup.bash
export PYTHONPATH="$PWD/src/luggage_planning:$PYTHONPATH"
/usr/bin/python3 -m pytest -q src/luggage_planning/test/test_ros_boundary_capture.py
# 输出目录必须为新的证据目录，已保存的selfcheck不得覆盖：
/usr/bin/python3 scripts/g3/verify_trajectory_capture.py NEW_OUTPUT_DIRECTORY
```

已运行证据：`pytest_final.txt`、`selfcheck_final/selfcheck.json`、各条原始CDR/字段/index。
