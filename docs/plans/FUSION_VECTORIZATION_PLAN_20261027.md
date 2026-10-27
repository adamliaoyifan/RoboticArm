# 融合射线投射向量化计划（方案 A → B，2026-10-27）

- parent: OCC-QUERY-INDEX-20261025
- owner: eng/claude/glm-5.3
- base: a17152e3
- 授权: 用户 2026-10-27（A+B 一起进行，A 先行；仅改射线投射模块）

## 1. 射线投射属于 cargo_volume_mapper.py（OCC 融合模块），不在预处理模块

## 2. 修改区域：integrate_points 内三个 kernel（详见完整版计划）
## 3. Oracle 测试先行；kernels 为纯函数接口便于 B 拓展
## 4. 预期：融合 739ms → ~65ms（A）→ ~5-15ms（B）
