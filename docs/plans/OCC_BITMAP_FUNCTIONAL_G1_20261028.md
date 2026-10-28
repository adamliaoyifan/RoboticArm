# 帧内位图去重实现与初步功能验证

- base_revision: bef80d81b373bb0fe057fd6170006c68530fd9fe
- owner: eng/codex/gpt-6
- scope: 现有 /tmp/elfin-occ-octomap-perf-cursor-g1/src/uld_occ_observer

用户授权实现位图，不改OctoMap DDA、不加线程、不合并端点、不改变discretize默认或Surface。OccupancyGrid管理帧内分类状态，每次collect清空；失败不能apply，随后帧恢复。free由唯一列表表达，occupied保留KeySet，每帧命中优先且每格一次更新。整数AABB检查先于线性下标；包围盒外不索引，包围盒内缓存seen/active两位，保留诊断计数含义。

位图用64位word编码每格2位，使用checked size_t乘法；默认上限64MiB，通过OccupancyGrid构造参数可缩小/扩大；超限具名拒绝、不降分辨率、不丢点。该上限只约束位图，不代表列表和地图总RSS。每帧完整清空，形状改变重新分配；分类结果不跨帧缓存。完成标记只有collect成功后设置，apply拒绝未完成scan。

验证环境：Release、OctoMap1.9.8，无ROS运行/Gazebo/性能战役。构建命令 cmake -S <worktree>/src/uld_occ_observer -B /tmp/occ-bitmap-functional-build -DOBSERVER_WITH_ROS=OFF -DCMAKE_BUILD_TYPE=Release；cmake --build该目录 -j2；ctest --test-dir该目录 --output-on-failure。CLI pytest指定此构建二进制。全部测试必须通过，零集合/概率/快照差异，测试是确定性有限行为覆盖，不声明产品性能通过。

新增测试：独立旧set参考完整free/hit及诊断计数；1cm/5cm、不同origin/shape、负/上界域外key、倒角inactive重复、hit穿过free、端点连续准入；每格列表唯一；Off/Stages/Detailed；失败后apply拒绝、图不变、下一帧可重用；reset/restore后复用；位图容量拒绝及恢复。沿用多帧logodds、checkpoint、列及配置回归；增加对旧set更新的完整树序列化对照。

证据在主仓库 docs/status/evidence/occ_owner_b1/occ_bitmap_functional_20261028/run01，包括base/dirty/diff、构建及测试命令日志、失败输出和固定输入/seed、最终提交与报告。失败保存当前日志/源码patch和可重放固定case后才能修复重跑，日志不覆盖。预算本轮功能验证，不启动重负载；小测试失败不消耗正式O5窗口。提交只包含包内修改；记录主仓库角色笔记并完成任务生命周期。性能与正式产品验收保持未完成。
