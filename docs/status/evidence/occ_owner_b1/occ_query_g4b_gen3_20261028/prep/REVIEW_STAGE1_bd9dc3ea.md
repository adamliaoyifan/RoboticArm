# Stage1 bd9dc3ea复核：原反例关闭，固定计划守卫待补

- revision: bd9dc3eac2643db310fe3484d6c3eab6f30e5352
- scope: 定向离线审查，不修改产品或启动评分
- result: 前轮三组具体反例关闭；固定负载合同仍有两项准备缺口

## 已关闭

独立pytest：27 passed in0.03s。diff相对0da4086b仅scripts核心和专用tests。二次start拒绝、非法重试reason/1ms/无身份ok/inf拦截，members共享误覆盖消除，input_hash双层校验有实现和正负对照。上述修复接受，不再要求重复这些测试/性能运行。

## P1：固定CURRENT日程没有被判官核对

对bounded_current_plan每个正确check_id及scheduled_s创建CurrentCheck，但全部start(0)、ok(.001,identity='A')，bounded_current_verdict返回20/20 complete=true。判官只核ID集合，不核记录日程与实际事件关系；既定t=3+6k计划实际上完全没有执行。

另独立探针start(3)、ok(2.996)返回passed：新增RETRY_TOLERANCE_S=5ms用于首拍，允许成功早于开始，不能把调度容差当时间倒退许可。

修复限同核心与测试：核对check_id对应冻结scheduled_s；实际start不得早于scheduled，事件不得早于start或倒退；原始scheduled/start/send/result/end分别保留，迟调度如实记账而不改scheduled。50ms等待和首次2s期限保持原合同，不新增5ms提前重试豁免。不要求生产调度零抖动，不新开性能测试。

## P2：C2计划在校验前被dict静默去重

planned_frames输入240条同frame_id='f'/hash='h'（t_s=0），机器实际只有一次完成，c2_closure仍complete=true、planned=1。构造器和判官先dict化，掩盖了固定240帧输入被破坏的事实。

需在建索引前校验原始计划ID唯一、字段合法、固定到达时刻序列；评分入口固定C2 240个ID和t=0..119.5（0.5s），不可从观测或去重后结果反推计划数量。核心可以继续支持短fixture，但必须显式传短计划，不能让非法重复计划变合法。实际记录须join计划hash及计划时刻，actual arrival/send延迟单列。已有正常240快/慢写测试保留。

## 直接复演（真实核心，不是镜像判官）

```python
from occ_g4b_capacity_core import *
checks = []
for p in bounded_current_plan():
    c = CurrentCheck(p['check_id'], p['scheduled_s'])
    c.start(0.)
    c.attempt(.001, 'ok', identity='A')
    checks.append(c)
print(bounded_current_verdict(checks))  # 当前complete=True，必须拒绝

planned = [dict(frame_id='f', input_hash='h', t_s=0.)] * 240
a = C2Admission(planned=planned)
a.on_arrival(0., planned[0])
a.on_send_result(.02, 'f', 'committed')
a.records['f']['propagation'] = 'exact_visible'
print(c2_closure(a, planned, True))  # 当前complete=True, planned=1，必须拒绝

c = CurrentCheck(0, 3.)
c.start(3.)
c.attempt(2.996, 'ok', identity='A')
print(c.finished)  # 当前passed，必须拒绝时间倒退
```

## 后续

Binding不撤销，Stage2真实进程接线继续准备；以上两项可同轮修复，不再单独等待策略批准。冻结脚本和输入manifest前必须钉住；评分/资源代表组仍受准备门约束。无需改产品、A门、资源阈值或增加任何测量批次。
