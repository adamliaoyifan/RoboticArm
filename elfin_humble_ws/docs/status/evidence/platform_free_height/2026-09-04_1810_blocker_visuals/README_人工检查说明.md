# PF-R5 blocker 现场图片（人工检查用，2026-09-04 18:10）

用户已人工确认（2026-09-04）：

1. **有纹理的行李箱（mesh）全部可见**，素色箱场景不必关心；
   素色 suitcase 生成代码已删除，仿真一律使用 thirdparty 行李箱资产。
2. **box/ 目录里 YOLO 框住的右边缘竖条 = 机械臂 base link**——场景里
   没有可识别的行李时，YOLO-World 把 base link 误分类为 luggage
   （label 2，conf 0.24）。这佐证了素色箱不是有效检测目标。

---

以下为原始记录。

两个目录，每个 trial 子目录含三件套：
`rgb_raw.png`（segmenter 输入原图）、`rgb_annotated.png`（画了 YOLO 框+置信度+
cargo 点数+pca 结果）、`snapshot.json`（数值遥测）。

## box/（素色无纹理 primitive box）——已定性：配置错误，非 YOLO 缺陷

- `snap_1`..`snap_10`：10/10 全部 FAIL（DETECT_TOP_UNOBSERVABLE）。
- YOLO 框锁定的是图像**右边缘一条 ~29px 宽的竖条**（bbox 约 611-640 ×
  112-295，conf 0.24）＝机械臂 base link，cargo 点数按尺寸恒定
  （8694/8864/8689），即 mask 覆盖的是竖直结构的切片，不是箱子顶面。
- 结论（用户确认）：素色 box 不带纹理，YOLO-World 开放词汇检测无法
  将其稳定识别为行李；此类视觉已从生成代码中删除。

## mesh/（thirdparty 行李箱 STL：suitcase_loafbrr / suitcase_vintage）

- `snap_1`..`snap_6`：6/6 全部检测 OK，cargo 点数 56k-106k，
  pca ok，height_valid=True（FULL_3D）。
- 现存的唯一真实问题是**参考错配**（blocker 2）：相机观测的是行李箱
  STL 曲面（圆盖/收边），GT 是 catalog AABB 尺寸。实测（loafbrr_large，
  GT 0.80×0.50×0.32）：
  - est height 0.304 vs GT 0.32（−16 mm，盖面低于 AABB 顶）
  - est width 0.780 vs GT 0.80（−20 mm）
  - est depth 0.449 vs GT 0.50（−51 mm，收边/圆角）
- 与 30-trial run2 的系统偏差一致（top −12..16 mm、w −14..23、d −39..53）。
- 烘焙的 `sized_suitcases.json` measure_size（lid-band 足迹）也不匹配
  顶面拟合器的观测（est 介于 catalog 与 lid-band 之间）。

