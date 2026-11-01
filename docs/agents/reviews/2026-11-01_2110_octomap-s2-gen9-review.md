# 2026-11-01 -- S2 gen9：磁盘复算成立，资源所有权与区域格数尚未闭合

- role: reviews
- agent: codex
- model: gpt-6
- cli: codex
- status: done

## Summary

702ed36e工作树clean；FREEZE五个二进制/库SHA一致；O9独立重判30/30 trace完整。本轮未重测ROS/O9计时。固定s2g9安装独立test_repair_s0+test_s2*offline：123 passed、11 deselected（live/持锁实机排除）。磁盘内容复算、资源真实peak字段接入及坏行拒绝已有进展，但S3仍不开放。下一代只补本记录三条已复现路径，原阈值/范围不变。

## Direct product observations

resource_probe.cpp独立编译并链接冻结的/tmp/observer-s2g9-core/libuld_occ_observer_io.a和libuld_occ_observer_core.a；未修改产品，输出resource_probe.log：

- shared_cache same=1 before=72 during=144；两个Snapshot的surface_cache指针相同，但tracker对象计数翻倍；两个Snapshot释放后shared_cache_after_last_owner=72，没有归零。
- 一条真实core积分INTEGRATED后取得surface()返回的shared_ptr，release租约、reset store、放开Snapshot，仅保留返回列数组。external_surface_columns=1200、retained_buffer_bytes=48000仍可读，repo.usage().cache_bytes=0，tracker.live(surface_caches)=0。此是非ROS的小规模真实产品行为，不是伪造回执/资源行。

机制：Snapshot构造函数把surface_cache参数move进成员，再用if(!surface_cache)判断是否新分配，移动后的参数为空导致共享缓存对象重复add。SurfaceCache析构按value.capacity扣缓冲，但surface()返回的shared_ptr可独立延长vector寿命；最后cache所有者不等于最后buffer所有者。ownership walk也只经Snapshot/cache查找，漏掉外部buffer读者。这是旧“共享去重/外部读者/最后所有者”要求，不能视为范围外。

## Oracle observation

audit.py加载主工作区复制的gen9 stage_rehearsal/o8/runs.json，将artifact_dir明确定位到该复制目录；真实健康样本从盘复算PASS。

随后不改原证据，独立注入：保留固定完整region标签，before/after都改为cells=1，inline内容为1个unknown state及float32零值，重算各SHA和content_sha。judge_o8仍PASS。_verify_probe_content验证len(states)==自报cells、len(odds)==4*cells，却没验证cells==prod(end-begin)=483516。此为离线反例，不声称实机只返回1格。磁盘内容真实性检查有效，但区域完整性仍可被自报格数绕过。

## S2 gen10 bounded acceptance

沿c474c507、813f78b5、21d3145c及原计划全部验收，不新增矩阵或门限。owner eng/claude/glm-5.3，基线702ed36e。以下全部AND：

1. 缓存对象按真实创建/最后释放计一次。修复移动后判空的所有权判断。独立产品测试：同revision with_identity共享同指针，缓存对象字节不增长；反复新identity后最后对象释放，活值回基线，peak不因重复计数增长。复用本审查resource_probe第一段验证。
2. Surface返回buffer按其实际shared所有权独立计量。保持surface返回对象后销毁cache/Snapshot、release/reset，缓冲容量与其控制/对象归属仍计入活值及同刻组合峰；最终外部读者释放才扣。两版本buffer并存和仅保留buffer的瞬态峰必须进入资源接口/判官；共享同buffer不重复，释放后回基线。不得以所有者不在repo目录而排除。复用第二段应报告至少48000缓冲字节（另对象/控制块按实际口径），并验证最终归零。先小型core测试，不消耗长窗。
3. 判官从冻结region/shape计算应有格数，要求before/after cells、state字节数、odds字节数均精确等于该值；不得依赖自报cells。正式region为483516格；1格/少一格/多一格/错界/相同假标签+合法SHA全部FAIL；主复制目录中的完整健康内容仍PASS。测试必须经过实际内容装载与哈希校验，不只改顶层cells造成自相矛盾。保留此前NaN、身份、时长和资源反例回归。

先离线与core小行为修复，补最近测试/文档，再运行受影响非计分资格、冻结并重测O9。O8保存内容可先按新判官独立复算；产品改动后不得用旧版本结果代签受影响资源/行为。预演≤2h（不含等锁）、证据容量及正式评分禁令不变。若未完成如实NOT_EVALUATED/blocked，不以partial scope全通过。S3 dispatch_ready仍no。无需重复扩大到无关功能。

## Pointers

- docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_gen9_review/resource_probe.cpp
- docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_gen9_review/resource_probe.log
- docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_gen9_review/audit.py
- docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_gen9_review/RESULTS.json
- docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_gen9_review/pytest.log
- docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_gen9_review/o9_rejudge.json
- docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_gen9/FINAL_REPORT.md
