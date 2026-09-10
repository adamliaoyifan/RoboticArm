# PF-R9 g2 D8 — D455 单元侧执行包（待网络可达后运行）

前置：单元主机 `192.168.11.70`（Jazzy，MTU 9000，librealsense 2.58.4），
D555 PoE `192.168.11.55`，`640x360@15`、`align_depth.enable=true`（HB-1
已验证）。单元已有本仓库 checkout（HB 用过 snapshot `40ab61c`）。

```bash
# 0) 单元上更新到 g2 提交并构建
cd <cell-workspace> && git fetch && git checkout 1d19528
source /opt/ros/jazzy/setup.bash
colcon build --packages-select luggage_perception --symlink-install
source install/setup.bash

# 1) 确认设备枚举（HB-1 命令）；D555 驱动节点按 HB 方式启动
#    （colour=/camera/d555/color/image_raw，
#     aligned=/camera/d555/aligned_depth_to_color/image_raw，
#     frame=d555_color_optical_frame）
# ！！本包不启动 point-cloud 发布/订阅（D8 要求 absent）

# 2) 预处理器：后端绑定走 D455 行（yaml 注释已写）：
ros2 run luggage_perception sensor_preprocessor_node.py --ros-args \
  -p input.color_image:=/camera/d555/color/image_raw \
  -p input.depth_image:=/camera/d555/aligned_depth_to_color/image_raw \
  -p output_cloud_frame:=d555_color_optical_frame

# 3) D3/D4 探针（>=120 s 计分 + 15 s 预热）：
python3 src/luggage_perception/test/pf_r9_g2_d34_probe.py \
  --out <evidence>/d34_d455_run1.json --duration 125 --warmup 15

# 4) 判定（D8 bars）：
#    emitted/unique colour >= 0.80（探针 d3_emission_over_rgb）
#    p50 <= 60 ms 且 p95 <= 150 ms（报告 max）
#    d4_paired_depth_over_emitted（第一方 flags）>= 0.95
#    filter join/depth >= 0.95、stale < 0.05（filter stats 订阅在探针内）
#    逐对 stamp/640x360/d555_color_optical_frame/colour-K 一致：
#      ros2 topic echo /luggage/preprocessed/status --once 里
#      buffers<=15；配对一致性由预处理器 acquisition_mismatches==0 保证
#    载荷物化=0（prep_counters.payload_materialisations）
#    点云 pub/sub 缺席：ros2 topic list | grep depth/points 为空
# 5) RSS 趋势：采样期间每 5s 记
#    ps -o pid,rss,comm -C sensor_preprocessor_node（判定无单调增长）
# 6) 设备枚举在采样前/中/后各确认一次；DDS 掉线=立即失败
```

D8 不测世界系几何（安装误差属 HE-2 范围，不阻塞 PF-R9 g2）。
