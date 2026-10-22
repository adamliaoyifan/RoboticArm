# 历史成功与当前失败：scene_tf 和 staging 分支对比

## 结论

上次完整两箱通过是 2026-10-19 gate3_aprime_r5，7377959a，2/2。
当前 G4 live_02 为 6ebaac7c，1 attempted/0 packed。两者保存的 scene_tf_used.yaml
逐字节一致，SHA256 均为 fa2af81a47a0946581faf5bd69ed28f942baabff90ad0c87b191d6d457d1e960。
10-22 live_151711 同样一致。这排除配置文件变化，不证明每一时刻实际 TF 全部相等。

## 直接证据与源码机制

旧 r5 launch.log 已出现 carrying-staging 的 ANCHOR_PAYLOAD_COLLISION，关节分支
约 [3.724,-0.709,-2.427,4.710,1.576,-0.988]，与当前问题分支相近。
7377959a 在 carrying staging 全被剔除时，has_staging=false，继续使用默认
home→portal 入口；只在有合法 staging 时才插入 staging hop。旧箱0实际提交8段：
transit/traverse/insert/descend/retreat/retreat/retreat/return。没有额外 staging 段。
它到 portal 的首段203点，终点TCP按当前相同机械臂运动学重算约
[-0.905027,1.010088,0.947561]m，而非当前图中的 staging [-0.405002,0.870008,1.177541]m。

后续 a71df14a 新增 staging endpoint gate：有 staging_nodes 时，要求 carrying 与
unloaded 各至少一个节点检查通过，否则提前 EDGE_BUILD_FAILED / staging_endpoint /
no_valid_staging。即使 carrying 节点已被前置 gate 全剔除，只剩 unloaded 节点，
该检查仍会触发，失去旧版尝试直接 home→portal 的机会。
G4 live_02 有70行明确因 no_valid_staging 拒绝；另外46行 NO_FEASIBLE_ROUTE、6行
ATTACHMENT_EXPIRED，不能把所有拒绝都归到这一个门。

此外，旧规划使用感知名义盒，当前采用595.487×413.696×244.295mm保守几何；
staging 从单基准扩成最多6位置，新增 corridor 高度修正。相机 xacro 内参也有变化，
旧测量约509.886×333.994×217.504mm，当前514.327×332.526×217.541mm，未做消融归因。
机械臂 URDF joints/collision mesh 源码未变化；不宣称旧运行时完整 expanded URDF 哈希相等。

## 判断边界

能确认：不是保存的 scene_tf 配置变化；旧版已经会拒绝碰箱 staging，但保留直接
入 portal 的建路分支。当前更大几何与新增前置门改变了可尝试的搜索空间。
不能确认：恢复该分支即可通过当前保守包络；旧两箱路线满足当前不确定性边界；
全部历史成功/异常都由此解释。此前337点的 nominal0/physical15 碰撞对照来自
10-22诊断成功选择响应，不是这次 r5两箱轨迹，二者不可混用。
本轮未改产品代码、未启动仿真或新增闭环。

## 索引

结构化证据：successful_baseline_comparison.json。
成功判定书：../../gate3_aprime/VERDICT.md。
源码：route_producer.py 的 anchor 生成、staging endpoint gate、has_staging分支。
