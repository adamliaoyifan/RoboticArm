"""Compile the human audit and retained, resolvable source provenance."""
import hashlib,json,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[7];OUT=Path(__file__).resolve().parent
manifest=json.loads((OUT/'manifest.json').read_text())
inputs={r['path']:r for r in manifest['inputs']}
findings=[]

def source(path,anchor):
 if path not in inputs:
  data=(ROOT/path).read_bytes();saved=OUT/'snapshots'/path;saved.parent.mkdir(parents=True,exist_ok=True);saved.write_bytes(data)
  inputs[path]=dict(path=path,sha256=hashlib.sha256(data).hexdigest(),bytes=len(data),snapshot=str(saved.relative_to(OUT)),supplement_time_ns=time.time_ns())
 row=inputs[path];lines=(OUT/row['snapshot']).read_text().splitlines()
 matches=[i for i,line in enumerate(lines,1) if anchor in line]
 if not matches:raise ValueError(f'anchor absent {path}: {anchor}')
 line=matches[0]
 return dict(path=path,line=line,sha256=row['sha256'],anchor=anchor,excerpt='\n'.join(f'{i}: {lines[i-1]}' for i in range(max(1,line-2),min(len(lines),line+4)+1)))
P='src/luggage_planning/luggage_planning/'
K='src/luggage_packing/luggage_packing/'
NODE='src/luggage_packing/scripts/placement_planner_node.py'
def add(id,priority,kind,title,mechanism,impact,next_step,refs=(),evidence=()):
 findings.append(dict(id=id,priority=priority,evidence_class=kind,title=title,mechanism=mechanism,impact=impact,next_step=next_step,sources=[source(*r) for r in refs],artifacts=list(evidence)))

add('F01','P0','源码＋历史原生对照','O 每侧5 cm膨胀无已核实标定依据',
 '新准备端和运行时均默认0.05 m，体积碰撞表示在XYZ尺寸各加2×margin，占用和未知体素都处理。与箱体10%包络可能覆盖相同误差，但是否重复尚未建立误差归属。',
 '已在合成夹具导致吸盘与占用表示重叠；不能据此断言真实机械臂碰撞。',
 '建立误差账本并讨论替代值；先用真实工具模型复现，不直接清零余量或删除未知。',
 [(P+'joint_operation_prepare_node.py',"param('collision_margin_m',.05)"),(P+'placement_prior_snapshot.py','shape.dimensions=[v+2*collision_margin_m')],['release_resolution_02/summary.json','release_resolution_02/contact_source_analysis.json'])
add('F02','P0','源码＋离线行为','整链默认漏掉90°和270°放置方向',
 'enumerate_placements未传allowed_yaws，底层None默认只枚举0° footprint；整链只为已有槽位补π，未补quarter turn。',
 '实际函数对照：0.15×0.30 m箱在0.40×0.20 m平面，默认0个可行候选，显式允许90°得到8个。不是物理不可达，是没枚举。',
 '把四个有向箱体yaw显式传入统一候选接口；检查相应抓取J6、固定G和载箱旋转轨迹，不仅交换长宽。',
 [(P+'placement_prior_requests.py','candidates = generate_candidates'),(K+'placement_solver.py','for yaw in allowed_yaws or [0.0]'),(P+'joint_operation_prepare_node.py',"':pi'")],['probes.json:quarter_turn_required'])
add('F03','P0','源码','ComputePlacement与整链候选的准入策略不同',
 'ComputePlacement向solve_placement传params、allowed_yaws、带placed箱体的constraint回调、atlas hook和same-snapshot volume verifier；整链直接generate_candidates，仅传top_n，随后廉价volume排序。',
 '间距、已放箱重叠/通道、观测授权与atlas语义不能视为等价。原生认证仍会拦碰撞，不能称已绕过最终安全门，但会产生错误候选或错误拒绝。',
 '统一只读候选输入/准入合同，显式传递有效policy和同一O快照；逐项对照两入口结果。',
 [(NODE,'result = solve_placement('),(P+'placement_prior_requests.py','candidates = generate_candidates')])
add('F04','P0','源码','O分辨率存在两套默认：1 cm与2 cm',
 'O配置resolution_m=0.01；ComputePlacement声明volume_observer_resolution=0.02并用它重建surface、nz及volume位置，而不是绑定owner配置；硬件placement组合未覆盖此参数。新整链接口则读取O配置文件绑定。',
 '默认组合可能拒绝地图或错误解释索引/体积，实际线上override未读取，不能声称当前运行必定如此。',
 '用owner同一配置和摘要贯穿compute/prepare/runtime；不再让消费者独立猜分辨率。',
 [('src/uld_occ_observer/config/observer.yaml','resolution_m:'),(NODE,'"volume_observer_resolution", 0.02'),(NODE,'geometry["_resolution"] = resolution')])
add('F05','P0','源码＋尺寸核算','1 cm全ULD读取需要67页，ComputePlacement上限64页',
 '当前几何149×197×148=4,344,244单元；65536单元/页需要67页，老compute读取循环在len(pages)>64时抛错。新ObserverSnapshotPort按count计算上限，没有同一个64页限制。',
 '即使把F04改为1 cm，这个旧入口仍会在完整volume读取阶段失败。尚未为本次审查运行67次原生RPC，结论是确定的源码容量冲突。',
 '按冻结geometry和page contract推导总页数，验证完整性/进度/截止时间；做同尺寸有限分页行为回归。',
 [(NODE,'if len(pages) > 64:'),(P+'joint_operation_observer.py','for _ in range(math.ceil(count/65536)+1)')],['data_recheck.json'])
add('F06','P0','源码','常规硬件placement启动图与默认O消费不配套',
 'hardware_placement_launch启动G mapper、placement和selector；placement默认volume_evidence_source=observer，但这个组合没有启动O。O standalone另需geometry_file/config及真实输入。',
 '若未外部启动O，start_placement本身不足以让默认ComputePlacement可用，服务就绪会失败。不是把G启动了就已接通O。',
 '定义一个明确部署组合：当前scene导出O几何、传感器输入、owner配置、服务就绪、消费者参数；先做不执行运动的启动排练。',
 [(P+'hardware_placement_launch.py','def placement_nodes'),(NODE,'"volume_evidence_source", "observer"'),('src/uld_occ_observer/launch/standalone.launch.py',"DeclareLaunchArgument('geometry_file')")])
add('F07','P0','安装文件实查','常用安装前缀未完整安装新整链接口',
 '检查install与install-mo的14个明确入口/配置：新prepare可执行文件及相应新模块缺失；install-mo的motion_planner与源码不同。attachment YAML字节不同，但解析后的数值完全相同，只能报文档字节差异。',
 '读到新源码不代表ros2 run/launch会运行新链路。隔离测试前缀曾成功，不等于标准硬件部署已更新；当前活跃前缀未知。',
 '冻结单一构建/启动前缀，记录ament解析路径、实际进程配置、URDF/SRDF和生成接口，验证后再进入实机排练。',
 [(P+'joint_operation_prepare_node.py','class '),('src/luggage_planning/CMakeLists.txt','scripts/joint_operation_prepare_node.py')],['install_parity.json'])
add('F08','P0','源码＋覆盖缺口','新整链有服务端，默认执行开关关闭，常规客户端接入未完成',
 'motion_planner的enable_joint_operation默认False；新runtime还要求operation_state_file及O/模型等参数。所查生产Python路径只找到prepare/execute服务端，没有常规driver的完整prepare→最终pick/J6→execute调用。',
 '用户要求的观察箱体后先准备整链、再执行最终pick/place，不能因接口存在就视为默认主链。仓库外客户端未调查。',
 '明确生产入口和身份绑定，增加plan-only整链调用与故障返回，再做执行端集成；保留旧接口兼容但不混淆覆盖。',
 [('src/luggage_planning/scripts/motion_planner_node.py','"enable_joint_operation", False'),(P+'joint_operation_runtime.py',"param('operation_state_file','')")])
add('F09','P0','历史数据＋源码','历史RGB-D相机模型不满足新观测适配器',
 '9/16录包抽查3个标定均有非零plumb_bob畸变；新partition要求D全零的rectified pinhole。对齐RGB-D不等于已经去畸变。',
 'OBS-JOIN单元/安装测试通过不能证明这批原始实测数据可直接接入，更不能靠把D清零消除拒绝。',
 '核对当前相机流和真实K/D/rectification，再决定适配畸变射线还是接收有证据的rectified流；保留原始像素来源。',
 [('src/luggage_perception/luggage_perception/cargo_observation_partition.py','np.any(d != 0)')],['ST-4-OBS-DATA-AUDIT/run_02/manifest.json'])
add('F10','P0','历史数据＋缺失证据','bbox填充mask不能证明完整吸盘覆盖或同箱身份',
 '237 mask中225有精确四源配对；仅抽查的2个非空实例mask均100%填充其轴对齐框。224 DetectionFrame的instance_id全部为空。',
 '矩形ROI不能替代真实顶面轮廓，任务ID也不能证明跨搬运物理身份；不能据此擦除本箱O。并未证明全部mask都是矩形。',
 '分别验证真实顶面有效区域、观测缺失和唯一身份关联；歧义保留O。先修基础射线/数据语义，再决定关联工作规模。',
 [('src/luggage_description/config/suction_contact_model.yaml','boundary_margin_m:')],['ST-4-OBS-DATA-AUDIT/run_02/coverage_assessment.json'])
add('F11','P0','历史原生证据＋覆盖缺口','没有真实全模型的成功完整正向基线',
 '静态atlas资格完整，但最新整链夹具是TCP球体/合成光学观测；退出起点原生拒绝。未证明真实臂/工具、当前O、释放观测及完整退出成功。',
 '不能把部件测试数、五段成功或atlas qualification当作整链/实机准入。',
 '在固定真实deploy_update完整模型上先取得一条不执行硬件的完整正向witness，保留原反例和所有阶段证据。',
 [(P+'joint_operation_planner.py','("place_retreat", "retreat"')],['release_resolution_02/summary.json','ST-0/deploy_freeze_01/native_qualification/result.json'])
add('F12','P0','源码＋用户已重审','5 cm悬空释放尚不支持紧密落箱结论',
 '新PLACE_RELEASE_GAP_M和旧RELEASE_DROP_M都为0.05 m；vacuum-off后自由落下。没有本部署释放/落稳数据证明横向位移、回弹或最终间距。',
 '允许最终贴靠不意味着5 cm落下后落点可控。无力/接触信号时，改成接触压箱也不成立。',
 '按用户决定重审高度/方式，分开验证释放前几何、落稳位置、支撑和退出；不要单独缩高度制造通过。',
 [(P+'waypoint_generator.py','PLACE_RELEASE_GAP_M = 0.05'),(P+'route_producer.py','RELEASE_DROP_M = 0.05')])
add('F13','P0','数据缺口','误差账本和实机异常信号没有资格证据',
 '没有独立尺寸/位姿参考、当前手眼/深度系统误差、执行跟踪、释放误差、多个相同箱体误关联率。用户确认扭矩/电流目前默认不可用，其他碰撞/保护停接口尚未确认。',
 '不能给5 mm/1 cm替代值附加物理保证，不能用无告警证明其他箱子始终静止。',
 '将几何算法验证与物理准确度资格分开；先采现有可获取的边界数据，明确每项误差需要的参考及可用故障状态。',
 [('src/luggage_planning/config/attachment_geometry.yaml','size_growth_fraction_wdh:')],['ST-4-OBS-DATA-AUDIT/run_02/coverage_assessment.json'])
add('F14','P1','源码＋离线行为','ComputePlacement同层5 cm间距与贴靠目标冲突',
 'constraint_verdict的默认同层footprint间距为0.05 m；结构化proposal另有gap_min=0.05。注意整链直接枚举没有接此placed validator，所以不能再称整链实际执行了同一间距。',
 '行为对照中相邻0.1 m箱面恰好接触，无正体积重叠：5 cm规则拒绝；零工艺间距不拒绝。后者尚不是轨迹或实机验证。',
 '明确零额外工艺间距目标，统一两种候选与整链的独立误差检查；保持正体积穿透拒绝。',
 [(K+'placement_solver.py','SAME_LAYER_CLEARANCE_M = 0.05'),(K+'placement_structural_candidates.py','"gap_min": 0.05')],['probes.json:nominal_contact_and_gap'])
add('F15','P1','源码','支撑高度容差5 cm、60%支撑及坡度规则未经当前箱体验证',
 'support_tol=0.05把距peak最多5 cm的高度计为contact；min_support_ratio=0.6；tilt ratio=0.05会补充拒绝缓坡，但这仍非实测完整支撑/刚性接触判据。',
 '容差过宽可能把高度不齐视为支撑，过保守也可能拒绝有效箱面。不能仅按参数名断言所有候选都错误。',
 '构造已知阶差/倾面/遮挡支持面，比较有效接触区域、实际可观测高度与用户允许的支撑方式；不与O膨胀共用一个数。',
 [(K+'placement_solver.py','"support_tol": 0.05'),(K+'placement_solver.py','"min_support_ratio": 0.6')])
add('F16','P1','源码','一格墙面margin与一格occupied-overlap tolerance语义相反',
 'hull_margin=None→resolution是向内收缩；occ_overlap_tolerance=None→resolution则在边缘扫描中只把h>peak+tol判为穿入。这不是同一个误差，且与后续严格原生碰撞规则可能不一致。',
 '可以出现候选阶段容忍一部分边缘占用，而轨迹阶段拒绝；不能把一格容差当成允许真实穿透的授权。',
 '用真实体素边界和独立几何oracle核对一毫米跨边/低台阶案例，明确离散保守性和物理穿透边界。',
 [(K+'placement_solver.py','cfg["occ_overlap_tolerance"] = resolution'),(K+'placement_solver.py','+ occ_tol)).any')])
add('F17','P1','源码＋离线行为','先截断候选，再atlas判定会产生假耗尽',
 'generate_candidates按top_n截断后，solve_placement才调用atlas_annotate；后面的可达位置不会补进来。整链还先取2048，再廉价volume排序，再截到slot_limit。',
 '同图同尺寸、固定假设atlas分类对照：top_n=1失败，top_n=2成功。这证明算法保留顺序问题，不证明真实atlas失败率。',
 '准入/标注后再保留，或分页有界补齐；记录总枚举/被保留/未搜索数量，不能把前缀耗尽说成不可达或箱满。',
 [(K+'placement_solver.py','candidates, atlas_report = atlas_annotate(candidates)'),(P+'placement_prior_requests.py',"params={'top_n':2048")],['probes.json:retention_precedes_atlas'])
add('F18','P1','源码','整链最终仅选首位和最浅槽，外部目标还受内部前缀限制',
 '从最多256默认pool只选pool[0]和min(depth)再补π；request.slots必须匹配这个有界pool，否则REQUESTED_SLOT_NOT_IN_BOUNDED_GEOMETRIC_POOL。',
 '中等深度、另一个支撑层或前缀之外的合法显式目标可在规划前消失；这不是终点真实无解。',
 '保留深度/层/yaw/可达分支的有限多样性；显式目标直接执行同一几何准入，不以入选内部top-N代替合法性。',
 [(P+'joint_operation_prepare_node.py','selected = [pool[0]]'),(P+'joint_operation_prepare_node.py','REQUESTED_SLOT_NOT_IN_BOUNDED_GEOMETRIC_POOL')])
add('F19','P1','源码','每个放置目标只生成两个朝向、每个朝向只返回一个吸附点',
 '目标回推从当前参考tool和其π翻转出发，propose_contact在最多256个中心检查中找到第一个完整覆盖点即返回。之后质量/IK失败没有同yaw下下一个完整覆盖点的联合迭代。',
 '已有完整覆盖、但更容易连接/避J6限位的邻近偏心抓取，可能从未进入IK。两端给候选的框架存在，候选集仍窄。',
 '在原有预算内返回多样的合法点/yaw，记录覆盖、质量、端点分支与连接失败，再决定补哪里；不为每格盲增采样。',
 [(P+'joint_operation_preparation.py','for opposite in (False,True)'),(P+'joint_operation_candidates.py',"return ContactProposal(query_id,tuple(float(v) for v in point)")])
add('F20','P1','源码','整链10 s预算中预留5 s，且全局仅3次分支尝试',
 'prepare入口budget_s<=10；certification_reserve_s默认5；planner总max_branch_attempts=3、每候选最多2。原生端点及连接单次上限各2 s。',
 '有界预算是合理约束，但当前组合在昂贵观测/全量O/两个连接后可能很少剩余搜索时间。没有阶段耗时样本证明该分配符合目标。',
 '记录获取O、感知、IK、连接、认证各段耗时和预算退出原因，按实测分配而非先加超时或宣称无解。',
 [(P+'joint_operation_prepare_node.py',"param('certification_reserve_s',5.)"),(P+'joint_operation_planner.py','attempts >= self.max_branch_attempts')])
add('F21','P1','源码','箱心atlas硬拒绝与偏心吸附目标存在量词错位',
 'ComputePlacement atlas hook在箱顶中心及π等价yaw查询，可对两者hard_reject_safe时剔除整个箱心；允许偏心抓取后，中心吸附不行不能证明该箱心上所有合法吸附点不行。',
 '可达集合可能在几何/规划前被过早缩小。新整链在实际G对应tool目标查询，不能把两入口当成同一消费语义。',
 '对具体吸附候选查询atlas，把箱心结果作为候选排序或待补充信息，除非有覆盖全部允许G的排除证据。',
 [(K+'placement_reachability.py','solver_contact_point(candidate, floor_z, inner_h)'),(K+'placement_reachability.py','all_hard_reject ='),(P+'joint_operation_atlas.py','contact=query(candidate.world_from_settled_box.compose(G))')])
add('F22','P1','源码','新整链atlas默认未接通，丰富指标主要停留在trace',
 '新prepare要求atlas_npz/yaml/qualification/scene四个显式路径；默认全空时在线IK fallback。已接通时保留manipulability、neighbor_confidence、lift_clearance，但augment取前三个branch作seeds，没有利用指标做跨候选调度。',
 '“源码存在atlas”不等于当前请求使用；不能声称已实现充分消费和性能提升。',
 '在有效启动配置中明确receipt绑定，trace报告使用/拒绝原因；先比较指标与原生成功/耗时，再加入调度。',
 [(P+'joint_operation_prepare_node.py','atlas_paths ='),(P+'atlas_proposals.py','manipulability:'),(P+'joint_operation_atlas.py','release=replace(release,branches=release.branches[:3])')])
add('F23','P1','源码＋资格边界','静态空载atlas不是载箱、当前O或动态轨迹的有效性证明',
 '34依赖当前仍匹配；5600格/12098状态的资格是空载静态场景。最大3分支、15 cm及4yaw仍粗；当前update未建立尺寸overlay并不能由此推导任何实箱都不可达。',
 '可作高价值seed，但不能“画一条线”排除所有尺寸/G/高度/yaw的深处位置。旧overlay缺失回落也不能假装载荷检查完成。',
 '保留当前真实G/保守包络的原生refinement和中间连接；根据失败区域有条件补采，不先建立猜尺寸全集。',
 [(P+'joint_operation_atlas.py','refined=refine_atlas_pairs')],['data_recheck.json','ST-0/deploy_freeze_01/native_qualification/result.json'])
add('F24','P1','源码','固定垂直出入场模板尚未利用当前O构造多条通道',
 'candidate_for_contact默认corridor_surface_max=None，place entry为release上方0.15 m，retreat再多0.10 m；预抓取默认0.30/0.25/0.35 m。之后OMPL连接可避障，但局部端点模板本身可能不可行。',
 '真实可行通道可能需要不同高度、方向或portal；末端模板失败不能证明目标无轨迹。',
 '在固定G下由当前O/开口推导有界出入场候选；先从失败碰撞对定位哪一个端点模板阻挡。',
 [(P+'joint_operation_candidates.py','pick_clearances=None, corridor_surface_max=None'),(P+'waypoint_generator.py','DEFAULT_PLACE_CLEARANCE_Z = 0.15')])
add('F25','P1','源码＋历史反例','缩小网格会改变unknown覆盖，全量O转换成本未测',
 'O→native为每个XY竖直blocked run建对象，未知保留；同39帧由10 cm变2 cm后出现20668未知单元和9867个O碰撞对象。新runtime每次验证附近多次capture/转换/比对完整状态。',
 '可同时影响可行性和时延；不能假定“网格更细就更容易且更快”。这些数量来自合成观测，非实机全ULD性能。',
 '以相同原始实测射线分别测覆盖和对象数/场景字节/阶段耗时；先优化表示与查询，不能用max_objects裁掉真实障碍。',
 [(P+'placement_prior_volume.py','merge only vertical runs'),(P+'joint_operation_runtime_ports.py','def same_collision_map')],['release_resolution_02/summary.json'])
add('F26','P1','源码＋缺失身份证据','释放后观测采用全顶面栅格检查，既可能苛刻也不能证明同箱',
 'observed_target要求目标物理顶面内所有栅格中心occupied且高度误差<=一格；无中心返回未评估。它依据计划settled位置检查O，未做物理箱体唯一关联。',
 '腕部遮挡或邻箱同高会影响结论；未接观测的TOP occupancy不能代替实际箱位姿/身份。当前规则不授权替换O，不能声称误关联已解决。',
 '设计可观测性/视角闭环与条件唯一匹配；缺少来源保留O，先拿真实释放视角判断是否需要新模块及最低范围。',
 [(P+'joint_operation_runtime_ports.py','def observed_target'),(P+'joint_operation_runtime_ports.py','def adopt_observed_release')])
add('F27','P1','源码','旧路径reachable优先且浅处优先，与容量目标未做统一权衡',
 'ranked_feasible_candidates将reachable/marginal放前，并在这些band内浅处优先；packing preferred仍是深处优先。整链首位+最浅也没有完整装箱后效评估。',
 '有利于尽快找到一条路，但可能过早占据入口或丢失后续箱数；尚无容量损失或时延改善的成对证据。',
 '把深处容量价值与规划成本分开记录，用相同箱序列比较已落稳数量/体积和完整周期，允许浅处有界后备。',
 [(K+'placement_solver.py','def _motion_rank_key'),(P+'route_producer.py','EXECUTABLE-FIRST')])
add('F28','P1','源码','时间有效性存在60 s与600 s等独立合同',
 'attachment最大年龄60 s；issued plan默认300 s；operation最大预算600 s，原始观测不会因为计划存储而变新。',
 '慢速多段执行或人工等待可在中途触发合理的陈旧拒绝。预算长度不等于承诺计划在整个预算内有效；尚无实机时长分布。',
 '记录从观测到seal/各腿/release的年龄，确定刷新/重规划/保留载荷的状态机；不能直接延长TTL掩盖问题。',
 [('st09_contracts/attachment.py','ATTACHMENT_MAX_AGE_NS ='),(P+'joint_operation_runtime.py',"param('operation_max_budget_s',600.)")])
add('F29','P1','源码','几何搜索失败缺少统一的补观测/补候选原因反馈',
 '旧solve_placement有OBSERVATION_REQUIRED及目标区域；整链enumerate仅返回PlacementCandidate，丢弃拒绝分布和观测需求，最终可能只得到SEARCH_INCOMPLETE。',
 '难以决定下一步该换抓取点、换IK branch、补O视角还是确实没有位置，容易重复花预算。',
 '按阶段返回可定位的失败与未搜索集合，接入抓取前有界补视角；不把所有失败都当不可达。',
 [(K+'placement_solver.py','observation_requirement = _observation_requirement('),(P+'placement_prior_requests.py','return tuple(result)'),(P+'joint_operation_preparation.py',"False,'SEARCH_INCOMPLETE'")])
add('F30','P2','源码／未测量','结构化搜索默认关闭，成功轨迹缓存尚未接通',
 'placement结构化搜索默认False且整链绕过该入口；独立prior缓存主要是候选和模板，不是可直接复用的完整成功轨迹。',
 '用户之前的结构化/几何筛选优化并非不存在，但不能说已全面服务新整链；也没有可复用轨迹带来的效率证据。',
 '先统一入口和语义再比较结构化搜索；成功轨迹作为proposal按模型/G/场景/phase绑定，复用前仍验证当前O。',
 [(NODE,'"structural_search", False'),(P+'placement_prior_prefetch.py','class ')])
add('F31','P2','源码','旧motion atlas配置存在可能未消费的遗留字段',
 'motion_executor保存_atlas_provider、_atlas_fk_tol、_atlas_submit_mode，但所查当前模块只有赋值，没有实际读取；部署YAML还写joint_goal和0.25 m FK容差。新整链走自己的精确refinement。',
 '不能把配置文字当成实际行为，也不能仅见0.25 m就断言执行接受25 cm误差。此项是旧接口消费审计疑点，不是新整链安全缺陷结论。',
 '对实际执行入口做provider调用计数和精确FK witness，删除或迁移无消费者配置，明确两套atlas消费边界。',
 [(P+'motion_executor.py','self._atlas_provider = atlas_seed_provider'),('src/luggage_planning/config/atlas_runtime_deploy_update.yaml','fk_verify_tol_m:')])

# Save all resolvable excerpts before creating the report.
manifest['inputs']=list(inputs.values())
(OUT/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
(OUT/'findings.json').write_text(json.dumps(findings,indent=2,ensure_ascii=False)+'\n')

head='''# ComputePlacement与MoveIt整链设计审查及数据证据 -- 2026-11-12

本轮是只读设计审查：**发现的不只是5 cm过大，还包括候选入口不一致、方向漏枚举、O分辨率和分页容量、启动/安装未配套。**这些问题优先于盲目扩大atlas或增加规划超时。

用户已确认：最终允许相邻箱侧面贴靠，禁止穿透和靠挤压就位；本轮重审释放高度/方式；吸附后G固定；默认使用O；腕部RGB-D/LiDAR，抓前补视角。没有改任何生产参数、规划行为或轨迹，也没有操作实机/Gazebo/控制器。

## 审查方法与证据等级

- 基准计划：`fc42bb89`；源行为基线`bcebf44d`。共享工作区脏状态完整保留在manifest，初始138项，不能视为干净可部署版本。
- 抓取当前源码/配置/关键历史摘要的逐文件SHA256和快照；审查末尾重验，发生并行变化则单独标记，不能沿用稳定结论。
- 新运行3个有限离线行为对照，直接调用真实packing函数；全部符合所列机制。合成fixture只证明算法行为，不证明实机成功率。
- 重新核对34个冻结模型/场景/资源依赖，34/34仍匹配；重新核对10份录包metadata，10/10哈希匹配，并检查原MCAP文件大小/mtime。未重新哈希几十GB原始MCAP，也未重新解码整个录包；下面225/237等是已保留数据审计结果的复核引用。
- 检查`install`和`install-mo`共14个明确入口/配置；没有读取活跃ROS进程参数，不能把源码默认值当成实机effective值。
- 等级：**源码**为调用与配置事实；**行为**为本轮有限对照；**历史原生**为已保存真实O/MoveIt执行结果；**缺失**为尚无法评估。P0=真实整链/实机验证前须澄清或修复；P1=明显影响搜索质量、容量、反馈或时效；P2=优化/维护待验证。不是每个P1/P2都是已复现故障。

## 当前链路及已经正确保留的机制

旧主路径：实测detection → ComputePlacement(O同lease surface+volume、placed/hull等约束、atlas标注) → Select/route → 分段执行。新路径：实测detection+顶面domain+冻结观测+当前关节/O → 内部简化枚举 → 回推抓取候选 → 两端原生局部路径 → align/carry连接 → 定时与全链认证 → 绑定计划 → seal后实测G复核 → 当前O原始suffix复核 → release → 观测与空载退出。

两条路径尚未完成等价与生产接入。新整链保持了完整吸盘覆盖检查、固定G、不改原始timed knots/J6 winding、阶段事件屏障、原生最终检查、O当前性/重置/版本复核和故障保留载荷等边界。这些不是要删除的阻碍。标准化误差策略也不意味着擦unknown、跳碰撞或放宽穿透。

## 必须更正的先前表述

1. 新整链“继承并执行5 cm箱间距”不够准确：它调用底层枚举但没有ComputePlacement传入的placed约束回调；默认值存在不等于调用生效。F03/F14给出准确边界。
2. 10 cm是合成夹具，O源配置默认1 cm；但ComputePlacement另有2 cm解释参数(F04)，不能只报一个“生产默认”就认为全链一致。
3. attachment配置源码/安装字节不同，但解析后参数相同，不把注释变化报告为数值行为漂移。
4. 原生报告碰撞是检查拒绝，并非真实物理撞击；此前2 cm/零padding只证明退出起点剩unknown拒绝，不证明整494点轨迹可执行。

## 设计所需数据：已有、能说明什么、还缺什么

| 数据项 | 已有并核查 | 目前能支持 | 缺口及最小下一步 |
|---|---|---|---|
| 当前部署几何/模型 |34/34冻结依赖匹配；源scene在base约[0,-1.059,-0.86]m、yaw=-90° | 固定几何下离线重放、静态atlas身份 | 实机TF/安装偏差独立核对；不能用AGENTS旧1.5m值替换 |
| ULD高度合同 | container绝对floor=.53、ceiling=2.01，usable height=1.48m | compute floor-relative与O absolute的转换 | 每个边界保留frame/datum，不把2.01写入size |
| atlas |5600格、12098存储状态原生资格、33状态paired replay；空载 | seed与静态质量先验 | 载箱/偏心G/当前O、完整真实模型操作性能未资格 |
| 地图配置 |O源1 cm；compute另2 cm；全网格4,344,244格/67页 | 精确发现配置和读取容量冲突 | 活跃owner/消费者有效参数、digest、page完整性、覆盖和延迟 |
| 原始传感器录包 |10份metadata及文件stat一致；9/16有RGB-D、mask、TF、joints、真空事件、LiDAR | 历史可用流与失败/同步分析 | 没有当前ULD释放/多箱场景，不能迁移成部署精度 |
| 图像精确配对 |237 masks中225四源精确配对；12缺RGB、2缺depth/info，可重叠 | 录包内join覆盖 | 还需每帧TF和真实投影模型；计数不是几何准确率 |
| 检测输出 |224条：168 measure、56 hold；142 fresh full-height；82无有效height | 过滤可作为实测几何的记录 | 不能将held、缺height和视角变化混合估计误差 |
| mask与身份 |仅抽查2个mask均bbox填满；224 instance_id全空 | 确定这些样本不能证明物理轮廓/跨搬运身份 | 真实顶面/实例边界和多候选唯一性，需要保留原始观测 |
| 相机内参 |抽查3份有非零D，新partition要求D=0 | 明确接口不兼容类别 | 核对真实流是否rectified；验证畸变射线/TF，不能伪造D |
| 箱体/抓取误差 |10%尺寸、±5°、5mm只是当前policy | 构建几何包络测试 | 独立尺寸/位姿参考及当前手眼/系统误差未有，绝对误差not_evaluated |
| O/吸盘碰撞根因 |同39帧/494点native对照；2 cm/5 cm仍拒绝；零padding起点4组unknown | 定位夹具中占用膨胀与unknown贡献 | 当前真实tool全模型与实测射线；未发生真实物理碰撞 |
| 执行跟踪 |历史joint_states与轨迹事件存在 | 可做时间同步/跟踪取证的候选数据 | 尚未建立同控制器插值/限位/速度误差资格；不能只看action success |
| seal/release |历史抓取真空事件；整链合成事件边界 | 接口和固定G逻辑检查 | 实际G误差、泄漏/丢失、release落点/回弹/退出视角未评估 |
| 环境静止与异常 |用户条件性确认其他箱体静止；默认无扭矩/电流 | 条件化规划前提 | 碰撞/保护停/执行错误接口、触发后的停机与重建地图须核实 |
| 目标性能 |有局部耗时和历史单例 | 定位分段成本，不构成提升率 | 同箱序/同O/同模型的完整成功率、延迟、落稳数量/体积对照尚缺 |

原始重点包：`~/work/robotarm_bags/20260916_212503_floor_box_pick/bags/pick/`。已确认流包括`/luggage/preprocessed/camera/{color/image,depth/image,depth/camera_info}`、`/luggage/semantic/instance_mask`、`/tf`、`/tf_static`、`/joint_states`、`/vacuum/state`和`/vacuum/events_json`；服务CDR边界由各owner recorder单独保存，不能假定rosbag自动录到服务。

## 问题索引

| ID | 优先级 | 证据类型 | 问题 |
|---|---|---|---|
'''
text=head
for f in findings:text+=f"|{f['id']}|{f['priority']}|{f['evidence_class']}|{f['title']}|\n"
text+='\n## 逐项机制、影响与下一步\n'
for f in findings:
 text+=f"\n### {f['id']} · {f['priority']} · {f['title']}\n\n**证据：{f['evidence_class']}。**{f['mechanism']}\n\n**影响：**{f['impact']}\n\n**下一步：**{f['next_step']}\n\n"
 text+='来源：'+'；'.join(f"[{r['path'].split('/')[-1]}:{r['line']}]({ROOT/r['path']}:{r['line']})" for r in f['sources'])+'。\n'
 if f['artifacts']:text+='保留证据：'+', '.join('`'+e+'`' for e in f['artifacts'])+'。\n'
text+='''
## 5 cm规则及相近数值的归属清单

| 值/位置 | 实际意义 | 本轮处理 |
|---|---|---|
|collision_margin_m=.05|O XYZ每侧膨胀，准备/运行时/廉价排序使用|重审误差归属和数值，代码未改|
|SAME_LAYER_CLEARANCE=.05|placed回调中的同层物理footprint间距|用户允许贴靠，重设独立工艺gap；不能说整链未接回调也已执行|
|gap_min=.05|结构化候选相邻位置生成|与布局gap统一，不独立藏默认|
|inflate_m=.05|旧occupancy sweep/XY障碍表示|记录实际入口及重复覆盖再决定|
|carry_margin_m=.05|载箱跨越顶面净空，部分函数取max(carry,pad)|区别于终点间距和release，不把max误报成相加|
|support_tol=.05|支撑/同层识别高度阈值|重审实际支撑含义，不等于5 cm都真接触|
|PLACE_RELEASE_GAP与RELEASE_DROP=.05|新/旧释放位姿与支撑面的箱底间隙|用户明确同意重审，无新值|
|min_candidate_separation=.050|吸附候选间的多样性采样距离|搜索策略，不是箱间距；需看是否妨碍合法偏心点覆盖|
|PLACEMENT_COLUMN_M=.05、staging比较+.05|旧路径候选排序分组/模板触发距离|分类为搜索模板，不统一替换|
|max_support_tilt_ratio=.05|无量纲约2.9°坡度比|不是5 cm|
|runtime timer=.05|秒|不是5 cm|
|检测/placed尺寸一致性比较>.05|尺寸记录匹配容差|需独立审计允许的尺寸来源，不作为碰撞余量|

一格hull margin、occ tolerance，15mm吸盘边界余量、18cm CAD footprint，以及箱体10%/±5°/5mm包络都要进同一账本，但不能简单相加为“总误差”。每项需要对象、坐标轴、阶段、来源、相关性与有效性；未标定不能当零。完整吸盘包含规则维持，配置中95%质量项不替代新路径100%几何包含。

## 下一步取证与复核顺序

1. **先修可确定的软件接线与一致性。**优先F02–F08：四yaw、两入口同policy、O owner配置、完整分页、标准安装和plan-only启动。先用有限离线/无控制器排练验证，避免浪费实机时间。不能借修复顺便修改5 cm等产品政策。
2. **与你复核误差和间距合同。**工艺额外gap允许0；观测/箱体/G/机器人误差单独定义；原生真实模型对照F01/F14–F16。缺失误差先标unknown，用明确geometry-only实验定位算法，不为推进实机编造数值。
3. **准备最小真实观测试采。**使用静止单箱多帧/抓前多视角，冻结RGB、depth、K/D、mask、TF、joint stamp和顶面receipt；先确认rectification、投影和真实顶面边界。现有包可重复性分析，绝对误差需独立参考；若没有参考就明确不能给绝对误差门限。具体场景、数量和验收阈值在取证前另行冻结，不把本报告当硬件执行指令。
4. **单独讨论释放和观测能力。**覆盖空ULD、一已放箱、紧邻箱、不同允许yaw/G、释放前后、遮挡和无法唯一关联。记录支撑、落稳pose、实际真空状态、原始退出起点和碰撞对；未接触觉/力传感前不设计依赖它们的下降。只对运动扫过的必要区域补观测，不要求无限采完整世界。
5. **建立完整真实模型正向基线与负例。**输入固定，完整pick→seal→carry→release→观测→retreat均有证据；碰撞/unknown/超时/传感器失效分别归因。保持原反例。通过后再谈atlas指标调度、定向补采、结构化候选与成功轨迹proposal缓存。
6. **性能/容量独立评估。**相同箱序/模型/O/预算下比较整链成功率、各阶段耗时、unknown覆盖、候选漏失和最终落稳箱数/体积；不能用更少箱、更大间距或更容易目标换取快。当前尚无提升数值。

每个checkpoint给出：有效policy与输入哈希、被证实问题/剩余假设、修改前后有限对照、未覆盖情况和需用户决定的边界；讨论后才生成更高generation的可执行任务。当前ST-0 broad和ST-3/4 whole-chain没有因为本次审查而完成。

## 本轮交付与验证边界

证据目录：`docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-DESIGN-AUDIT/run_01/`。

- `manifest.json`：版本、完整dirty列表、输入哈希/快照、并行变更、采集范围。
- `findings.json`：每项问题的源码行号/SHA/保留片段及分类。
- `probes.py`、`probes.json`、`probe_summary.json`：3个有限实际函数行为对照，全部按预期复现。
- `data_recheck.json`：34依赖、10包metadata/stat、1cm网格/67页核算。
- `install_parity.json`：14个安装路径及源码/解析值对照。
- `collector.py`、`supplement.py`、`build_report.py`：可重放审查程序；重放应使用新目录，不能覆盖本次快照。

没有新实机采集、完整原生重跑、修复或性能试验。结论范围是当前保留源码和明确检查的安装前缀，不代表线上effective配置；校准、释放、物理身份和实机可靠性均保留not_evaluated。
'''
report=ROOT/'docs/reviews/PLACEMENT_PLANNER_DESIGN_AUDIT_20261112.md';report.parent.mkdir(parents=True,exist_ok=True);report.write_text(text)
print(json.dumps({'findings':len(findings),'P0':sum(f['priority']=='P0' for f in findings),'P1':sum(f['priority']=='P1' for f in findings),'P2':sum(f['priority']=='P2' for f in findings),'provenance_sources':len(inputs),'report_bytes':len(text.encode())},ensure_ascii=False))
