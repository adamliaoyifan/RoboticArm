# OctoMap Claude 接手：S2 收口与 S3 待开放

- parent: OCC-QUERY-INDEX-20261025
- owner: eng / claude / glm-5.3
- base_revision: a70929be5f4b18c79564e1d84f7a8507e6484ea8
- original_plan: docs/plans/OCC_OCTOMAP_REPAIR_2CM_G1_20261031.md
- original_plan_revision: c02c1cfbd9ba16a1c4a6c6169695f1daf8eb6a76

用户要求改派Claude。具体路由沿用仓库登记的Claude Code agent=claude/model=glm-5.3；不是认定该会话已启动。模型若不同，领取前由reviews按实际路由调整未领取任务；不得冒用模型。继承原计划所有矩阵、门限、证据与预算。本补充只固化未完成项和接手程序，不降低验收要求。S2证据g6–g9不是正式任务代次；S2下一正式代次为6。

## 接手与边界

在主工作区领取；从上述基线创建独立工作树，不进入Cursor旧树写代码，协调只写主docs/agents。保留所有历史失败与O9。S2只允许离线、行为、非计分预演和O9资格；禁止正式O5–O8、Gazebo、规划运行、生产迁移、1cm新增测量、后端公平比较。单线程融合、全有效射线与概率模型保留。

查阅s2_g9 owner证据位于/tmp/elfin-occ-octomap-repair-2cm-g1/docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_g9，审查证据在主docs/status/evidence/.../s2_g9_review。审查结论docs/agents/reviews/2026-11-01_0800_octomap-s2-g9-review.md为接手事实，不将缺项当成已实现。

## S2 gen6 验收（全部为AND）

1. 修采样判定：逐帧检查终态及身份；所有延迟/年龄/RSS/资源字段拒绝bool、NaN、Inf、负值和缺失。保留健康对照；对三种Surface各注入NaN/Inf/负值/缺失及第二帧拒绝，必须使接线失败、退出非零。正式判官继续拒绝短样本，不能用缺组拒绝掩盖采集坏样本。
2. 每个服务独立记录request ID、开始/结束单调时钟及状态；查询总链时长另列，不能将0.717092s两条链总时长写成一次ROS往返。发布记录计划、实际调用起止、返回、callback进入、入队、完成、账本/回执发布与接收。云stamp独立字段，不冒充publish返回。记录真实同ID回执列表及计数，重复必须失败。时钟域无法直接相减时保留各域，不伪造跨域耗时。
3. 行为预演验证独立发送：客户端4执行线程，独立sender。2Hz两次发送，查询故意阻塞1s，第二次实际publish开始应在首个计划时刻后0.5s±0.1s且早于阻塞查询结束；保存边界和线程身份。离线虚拟时钟反例与真实非计分实验分开。正常短链每个已接纳ID闭合，WaitSnapshot真实租约，全页7425列/15页仅适用于该固定夹具、ROI4096、完整身份和release，缺页/重复列/错误身份/1格ROI均失败。所有health/RSS样本须有效，缺采样/重复时间/进程退出令接线失败。
4. 实现完整正式runner但不启动正式样本：O5四组、O6 96组、O7九窗及过载/恢复、O8三轮和hold，次数/输入/门限严格继承原计划。profile同进程累计帧，明确3预热+30计分、首次构建和冷启动流程；--stage只执行指定阶段。O5失败停止O6/O7。离线调度器用完整矩阵及虚拟时钟验证精确计数和预算；缺一组/一帧/一查询、单窗失败、全拒绝/unknown、重复终态、callback前缺口、恢复timeout/错快照、超门及缺字段均拒绝。每个case/window为AND，有限集合要求100%成功；不推断长期可靠率。
5. 失败证据在拆卸前落盘：清单按实际存在、身份可关联、hash及重放输入检查capture_complete/replay_possible，不能无条件true或只存截断异常。注入启动缺程序、发布后异常、查询失败、采集截断四类，逐项保留生成manifest/必要输入前缀、TF、边界与错误。已保存真实失败用新进程重跑产品并比对身份/状态；缺输入或丢事件包拒绝。遵守512MiB/例、2GiB/run、8GiB总量，不能为满足容量删重放必要数据。
6. 完成GetResourceUsage实际归属覆盖与峰值：共享块、目录map节点、cache、仅旧读者保留对象、在途响应、队列/账本，树及中间件归属范围明确。计容量、共享去重；多句柄/两版本/第9句柄/TTL/release/reset/外部读者/最后所有者释放均做行为验证，与独立分配/所有权清单核对。任何门相关未覆盖项保持NOT_EVALUATED，S2不得宣称资源资格完成；512MiB视图/缓存和2GiB进程RSS门仍按原计划评分，不能以部分字节代签。
7. 非计分预演≤2h（不含等锁）。当前x86 Release/Humble/OctoMap1.9.8/discretize=false、powersave、不绑核；持/tmp/elfin-occ-backend-perf.lock，不停他人进程。冻结产品、工具、fixture、input manifest全SHA及干净状态；生成无占位COMMANDS.sh。冻结后O9按原计划2cm chamfer/30k/4096ROI/首次Surface MISS，三操作各30对AB/BA，nearest-rank P95比≤1.10，全部ON trace有请求/观察身份、无丢事件，实际失败可重放。缺样/非有限值不通过，7537b033不能代签。
8. 交付每条pass/fail/not_evaluated与证据。原三个测试文件需全跑并记精确安装路径和二进制SHA；新增行为反例穿过实际adapter→artifact→judge。声明实际复现/离线注入/源码机制的边界。资源或O9未合格则按实际blocked收口，不伪称完整通过；S3仍需reviews基于完整证据明确开放。

## 执行程序

Claude领取后首先提交可执行COMMANDS.sh，固定本机路径：构建→行为/离线测试→预演→冻结→O9→正式runner离线证明→独立判官→失败重放。构建命令根据新工作树与独立build/install实填，不复用未知版本/tmp/s2-install。S2脚本默认禁止执行正式O5–O8；正式命令仅在未来S3授权入口中保留。脚本本身必须可执行，无TODO/占位参数。

取主工作区AGENTS/INDEX/README/FORMAT规定；agent_start领取、角色笔记、实现/测试/修复/提交、agent_complete实际关闭均由同一owner完成。产品修复后重跑受影响行为及冻结资格，不拼接候选成绩。

## S3 gen2（未开放）

owner同S2，依赖S2 gen6及reviews明确开放。当前dispatch_ready=no，不领取、不启动正式样本。S2完成后先把S3基线更新为已冻结完整产品/工具SHA、冻结manifest与精确命令；在尚未领取状态下由reviews落实开放。随后严格按原计划：正确性/捕获资格→O5→O6/O7→O8→独立复算/失败重放，正式预算8h不含等锁。O5失败停O6/O7，缺项not_evaluated；冻结期间改代码则结束候选并保留失败，新版本重跑受影响门。历史1cm失败、新恢复合同与旧立即查询、工程交付与产品验收分别报告。

## Placement 与自动合并

本轮交付接口接入设计说明，不授权实际规划接入：明确2D轴序/坐标/高度、observed与unknown/free/occupied、floor_prior仅地板存在、同租约全页、epoch/reset/observed_revision和规划同图身份；列出独立适配任务与行为oracle。自研双log-odds阈值与OctoMap概率阈值不可混称相同；后端比较另授权。保留现有无损prune，不把不同概率或unknown吞并成occupied；现有收益探测不等于新优化或资源认证。
