# HULL-GATES G3 派发补充规格

- parent: PLACEMENT-GRID-LAYOUT-20261023
- subtask: HULL-GATES
- generation: 3
- parent_plan_revision: 33ba6745868cb2a55be49a94be7c25cdc514039f
- product_base_revision: fd8f57a1a987ec0fcdae9c27a271306d7fda2636
- owner: eng/claude/glm-5.3
- dispatch_ready: yes
- authorization: 用户明确派发HULL-GATES；仅离线证据、按证据限定的修复、回归与交付，不授权仿真/运动/性能战役。

## 1. 权威范围与三条收紧

本文件补充33ba6745计划的HULL-GATES单元；冲突处以本文件为准，其他单元仍未派发。实现从本补充规格所在提交建立独立worktree，产品代码基座仍为fd8f57a1。

1. H3已确认的是solver使用through_aperture谓词，不是已证明完整生产链会返回非法终态。先构造最小输入通过真实候选枚举、硬门、排序/最终选择的完整选位链；若实际node另加validator，须离线调用真实适配/回调验证该门，而不是只调用一个谓词。能输出非法最终槽位才签当前输出缺陷。若其他门拦住，记录拦截位置与剩余风险，不声称已复现；无反例时先交付验证，不自行改业务行为。
2. H5先独立OBB投影区间对照，不能把交叉角点顺序本身等同缺陷。有反例须保存实际误判输入及两侧结果，再修共用几何内核；无反例则只加验证，不修改内核。反例应区分假合法与假拒绝、真实体积重叠与间距策略。
3. 完备性只指预算充足、fallback完整遍历后的离散集合覆盖；有限预算截断不得宣传完备。本单元不实现fallback。

## 2. 本次交付

- H1：descriptor/hash准入合同及行为测试。对于生产明确要求倒角的请求，缺失/错误descriptor必须具名拒绝，显式cuboid fixture保留合法入口；首先测现有路径，已满足则不重复改动。
- H2：现有高度区间/可行中心域几何核与独立半空间oracle对照，法向margin语义一致；有偏差留最小反例后修复，不复制另一套产品hull。
- H3：完整选位链反例或负复现报告；若证实缺陷，分离最终完整包含与路线过开口语义，修正最终放置入口，不破坏路线过境。
- H4：独立验证水平支撑与倒角包含，未知区域不补平；实际地板与候选包络支撑检查分别保留。
- H5：独立OBB/SAT验证与证据驱动修复；不继承“旧oracle相同即正确”的口径。
- 输出稳定、无ROS的硬门入口或现有helper入口清单，供后续STRUCTURAL-CORE使用。若没有证据要求重构，仅交付入口/语义映射及测试，不为了建立Class扩大改动。

允许修改luggage_description几何核、luggage_packing最终placement硬门与其最小节点准入适配、focused tests、相邻架构文档/规则。不改地图、Attachment参数、评分、候选生成策略、atlas、控制器、物理资产或共享install。不启动ROS进程栈；离线真实回调可用依赖替身，但几何/solver/选择逻辑不可替换为桩，报告列出所有替身。

## 3. 冻结测试和判据

创建 `src/luggage_packing/test/test_placement_hull_gates.py` 及必要description纯几何测试。

矩阵：部署真实descriptor与显式cuboid；resolution0.01/0.02/0.05；候选yaw0/90、独立几何对照yaw0/5/45/90；z底在floor、wall_z上下1mm及箱跨wall_z；有效法向margin边界-1mm/0/+1mm；多非零container-world平移/yaw；完整Attachment偏心/保守中心偏移；nominal合法包络非法、中心合法角非法、开口外突出、hull合法支撑不足、unknown支撑、错/缺descriptor/hash。

- 全矩阵合法性必须与独立半空间/OBB oracle一致；假合法和假拒绝均为0；数值误差<=1e-9m，零边界使用同一冻结EPS，不加通用碰撞容差。
- H3每个反例记录原始request、snapshot、ledger、descriptor、Attachment、原参数、未截断候选/最终响应和逐门原因；不得通过单独伪造候选绕过枚举来宣称完整链缺陷。可另加针对未来直接候选入口的predicate测试，但标签分开。
- SAT显式覆盖对称性、交换矩形顺序、平移/共旋转不变性、分离/相切/微重叠；独立oracle不调用被测角点/轴生成函数。
- 每个已证缺陷先在产品基座复现，再在修复commit上同输入通过。无反例时报告有限矩阵负复现，不证明所有输入无缺陷。
- 原solver/atlas/几何基线测试新增失败ID=0；若修复必然改变旧错误expected，单列反例、规范依据和改变列表，不能静默更新golden。所有未满足门记FAIL或not_evaluated，不以实现交付代替通过。
- 这是确定性矩阵AND门，要求全部通过，不统计或声称在线可靠性提升。性能门和真实消息/部署身份不在此单元，保持未评估。

精确执行入口（新文件须交付；现有几何test文件以git ls-files定向发现并记录实际命令）：

```bash
set -o pipefail
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/luggage_packing:src/luggage_description:src:. python3 -m pytest -q src/luggage_packing/test/test_placement_hull_gates.py
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/luggage_packing:src/luggage_description:src:. python3 -m pytest -q src/luggage_packing/test/test_placement_solver.py src/luggage_packing/test/test_placement_atlas_hook.py
```

保存全量stdout/stderr和退出码，尾部摘要不是完整证据。对适配回调的额外依赖明示环境，不使用不同checkout的隐式import；报告打印关键模块__file__。

## 4. 工作区与收口

建议实施worktree `/tmp/elfin-placement-hull-g3`，branch `agent/eng/placement-hull-g3`，从派发plan_revision创建。先在主workspace调用agent_start.sh领取。产品基座/父计划/补充规格commit均记录；不得在领取后静默换base。

证据主workspace路径 `docs/status/evidence/occ_owner_b1/placement_hull_gates_g3_20261024/`：REPORT.md、环境/版本、fixture矩阵、反例或负复现、真实链门结果、完整测试日志、修复前后diff。测试产品代码先提交并冻结，记录dirty状态；证据报告提交单列，不夹带其他agent文件。

完成后写完整runnable角色note并agent_complete；Result指向实际实现commit与各门判定。只有上述本单元门全部满足才能outcome=pass；未复现允许以已完成验证收口，但不得声称修复了不存在证据的缺陷。不自行cherry-pick到集成、不改G3/master指针、不重建共享install。

STRUCTURAL-CORE必须另行派发，并绑定本单元通过commit。用户此次授权不延伸到其余单元。
