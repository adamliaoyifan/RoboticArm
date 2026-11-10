# 单次生产诊断捕获：接线及非运动验收

- base_revision: c240a796
- dispatch_ready: no
- motion_diagnostics_started: 0
- historical_replay_admitted: no
- verification_scope: real_ROS_actions_and_services_with_synthetic_backends

## 结果

三项接线已实现，并通过非运动行为测试；尚未冻结或启动单次运动诊断。
`pytest_verified.txt`：107 passed、3 skipped（缺少已有G9/baseline历史证据），
1条xacro/Python版本警告。新增接线/退出测试21项，其余为既有格式、handoff、
启动及执行预算回归。不是生产成功率认证；不从这些测试推断运动故障根因。
`build_final.txt`：luggage_planning、luggage_gazebo两包构建安装通过。
`installed_help.txt`及`installed_launch_args.txt`证明安装后的捕获入口和参数可解析；
只执行help/show-args，未启动launch进程、Gazebo、MoveGroup或控制器。

## 1. 实际FJT与executor边界

新增显式启用的两层action适配器。executor仍调用普通`/execute_trajectory`，
MoveGroup仍调用普通`/elfin_arm_controller/follow_joint_trajectory`，诊断launch
将五个底层action端点全部路由到捕获适配器；关闭时不添加任何remap。
适配器记录真实收到的完整SendGoal（包括被拒请求的UUID），在实际下游
`send_goal_async`处记录完整请求并以相同UUID转发；记录accepted/stamp、
原始状态、feedback、GetResult响应及取消请求。每一跳分别保留自己的UUID。
FJT记录在发送时绑定活动Execute UUID集合；恰好一个才标记关联无歧义，
并发或无父goal均保留为关联缺口，不猜测父请求。

`synthetic_verified/action_success/messages.bin`与`fjt_sample.jsonl`是**合成替身
实际action边界样例**，不是历史goal，也不是机器人/生产MoveGroup实测goal。
普通名称的ROS客户端确实经重映射发送；替身MoveGroup刻意将executor的
positions `[0.1,0.2]`变成`[0.875,-0.25]`，并改变frame_id。断言捕获FJT与
下游替身实际收到的goal逐字段相同，同时不同于executor输入；关节顺序、
空a/effort数组、header sec/nanosec、time_from_start纳秒及UUID全部保留。
Execute UUID=`03`×16，FJT UUID=`07`×16；返回结果各自与UUID对应。
覆盖success、abort、下游reject、正常cancel及执行中capture fault。

认证`ValidateRouteV2`通过实际服务适配器原样转发并捕获完整请求/响应，
以独立service_call_id配对；安装到driver的typed recorder钩子保存Select
请求/响应及PlanMotion请求/响应，并保留已有recorder。诊断driver也将
本地认证client路由到适配器。无边界内容使用认证副本冒充实际FJT。

**适用范围：**这是插入通信跳数的诊断接线；下游拒绝发生在上游接受后时，
上游以ABORTED结束，原拒绝独立保留。该传输/状态语义差异必须进入未来
运行manifest，不能把它当作原生产上下文完全未变。当前未对真实C++
MoveGroup/控制器启动后的连接或同运行整条认证—Select—提交链做运动验证。
其他直接绕过适配器的FJT发送者仅有状态订阅覆盖，不能声称其完整goal已捕获。

## 2. 高频采集与故障

回调仅完成CDR快照、来源/接收时间及有限内存入队；不做磁盘操作、不等写盘。
单独worker写有长度与SHA256的完整CDR帧，关闭时drain/flush/fsync。目录新建
且不覆盖。每进程磁盘预算512MiB、队列4096项且最多64MiB；capture节点与
驱动记录器合计磁盘上限1GiB、排队字节上限128MiB（不含ROS自身队列）。
拒绝超预算/队列满；received/enqueued/written/dropped/failed/pending均报告。
错误具名且粘滞，包括CAPTURE_QUEUE_FULL、CAPTURE_WRITE_FAILED、
CAPTURE_HEALTH_WRITE_FAILED、CAPTURE_DRAIN_TIMEOUT。written仅计完整write返回，最终持久性需close时flush/fsync成功。错误日志和health可见；
采集故障触发正常action cancel，禁止提交下一腿。driver自身磁盘故障通过
匹配run_id的故障topic通知捕获节点，测试覆盖执行中正常取消；typed边界
记录失败时拒绝发起该请求。health也必须匹配本次run_id。

依据sim controller配置update_rate=100Hz、state_publish_rate=50Hz，以及
launch中1kHz /clock说明，使用高于预期速率的混合输入，**一次4秒非运动
负载**：clock1250Hz、joint_states250Hz、controller_state250Hz，总1750Hz。
最终实测总速率1749.98Hz，发布/接收/写入均7000条（5000/1000/1000），
丢弃0、失败0，全部源时间戳与发布清单逐条核对；空数组和完整消息CDR保留。
回调p99=0.045115ms、最大=0.182205ms；
独立200Hz执行探针仅统计实际发布窗口（排除启动/drain），得到799次回调（门槛≥600），最大探针间隙5.414ms（门槛<50ms）；
p99门槛<0.571429ms。定向强化窗口测试1 passed，见pytest_workload_window.txt。
`synthetic_workload_window/rate_1750hz/rate_result.json`包含计数和测量。

另外固定阻塞磁盘worker，验证回调在其仍阻塞时完成，队列满具名且3收/2写/1弃；
ENOSPC开盘故障、中途write故障均具名，计数守恒，中途故障的2条请求均记failed。
所有故障样例保留在synthetic_verified下，不能作为完整捕获。
`dropped`是采集器内部丢弃计数，不能测量未知生产发布者在DDS之前丢失的消息；
合成测试额外知道发布数，所以能证明此负载下端到端没有缺样。生产仍需看
源stamp间隔、发布配置/速率及捕获覆盖。文件顺序允许并发交错，以sequence、
source stamp、UUID对齐，不能把接收wall时间当物理源时间。

## 3. 正常退出

诊断入口使用原PackEvalDriver.run的启动、graph、home、spawn、map确认和
run_trial顺序；唯一安装的trial出口在commit/清箱/重试/下一箱之前返回。
原place循环只在显式诊断钩子存在时分支：第一腿必须为transit且捕获健康，
执行后立即进入诊断出口；不提交insert/descend或额外hold goal。
成功保持采集60仿真秒，继续运行后台回调并检查捕获、vacuum和clock。
hold有300墙钟秒上限以处理停钟（只限制新增诊断观察，不改变既有等待参数）。
transit失败或捕获故障走原`_abort(..., carrying=True)`，保持真空和箱体，落盘证据。
成功也停在DIAGNOSTIC_COMPLETE_CARRYING，等待明确恢复；不自动释放/删除箱体。
这取代此前准备草案中“观察后自动detach/clear”的未实现提议。

行为测试直接调用真实place循环和真实pack循环，并以替身边界提供输入：
成功精确观察60e9ns；transit失败、提交中捕获失败、观察中捕获失败、观察前
不健康、payload lost、停钟和倒钟均保留箱体并正常返回；即使max_boxes=2也
仅调用一次run_trial并在cleanup/commit前返回。未使用SIGSTOP或杀节点停环。
保留已有规划、速度、staging、execute等待和settle/handoff安全门及其回归。

## 已知开发问题与修复

- action基名remap的首次非运动接线测试会直接绕过捕获（无Execute记录）；
  改为五个底层端点，使用普通名称的真实ROS客户端验证后通过。
- 下游CANCELED不能直接映射到未收到取消请求的上游CANCELED；该情况改为
  ABORTED，上游有正常取消请求时仍返回CANCELED，测试覆盖两种情况。
- 现有build是复制安装，首次symlink-install因生成目录冲突失败；保留原始build.txt.gz，
  使用原复制模式构建通过，未删除现有构建目录或改staging。

## 复核

环境：/usr/bin/python3，ROS Humble，ROS_DOMAIN_ID=118；依次source
/opt/ros/humble/setup.bash、主工作区install/setup.bash及本卫星install/setup.bash。
PYTHONPATH前置src/luggage_planning、src/luggage_gazebo。测试清单和命令见
`manifest.json`，发布输入为确定的整数索引与源stamp，无随机规划或仿真。
仅一次有限合成负载的通过，不编码每条生产运动的可靠性或成功率门槛。

离线检查完整FJT字段（不会创建ROS节点）：

```bash
/usr/bin/python3 scripts/g3/inspect_capture_stream.py \
  docs/status/evidence/occ_owner_b1/gate3_aprime/capture_wiring_20261020/synthetic_verified/action_success \
  --boundary /elfin_arm_controller/follow_joint_trajectory:send
```

未来运动冻结仍单列：确定安装/配置/seed/命令、同运行边界覆盖清单和最终
证据审计及正常拆栈顺序。新建捕获未发生，run_capture_complete始终false；
未把格式验证或替身接线测试提升成生产失败捕获，也未推进轨迹/上下文根因。
历史轨迹原样重放继续未准入；无论未来单次诊断结果如何，均不自动补跑。
