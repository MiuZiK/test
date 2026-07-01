# 轨道跟随算法 - 离线验证指南

## 📋 概述

`validate_rail_follower.py` 是一个**完整的离线验证工具链**，允许你在不启动实车的情况下：
- ✅ 验证轨道检测算法的可行性
- ✅ 评估不同参数配置的性能
- ✅ 分析失败模式和边界情况
- ✅ 生成可视化结果供人工审核
- ✅ 自动搜索最优参数

---

## 🔧 环境准备

### 1. 安装依赖

```bash
# ROS相关（必须）
sudo apt-get install python3-rosbag python3-rospy python3-sensor-msgs

# Python库（必须）
pip install numpy scipy matplotlib opencv-python tqdm pillow

# 可选（用于更高级的分析）
pip install seaborn plotly  # 更美观的图表
```

### 2. 准备Bag文件

确保你的bag文件包含激光雷达数据：

```bash
# 检查bag文件内容
rosbag info your_recording.bag

# 应该看到类似输出：
topics:      /scan              1234 msgs    : sensor_msgs/LaserScan
             /Odometry          1234 msgs    : nav_msgs/Odometry
             /sbus               100 msgs    : sbus_serial/Sbus
```

**关键要求：**
- 必须包含 `/scan` (或自定义) 话题的 `sensor_msgs/LaserScan` 消息
- 录制时小车应该在轨道附近（能看到轨道）
- 建议录制 **10-30秒** 的数据（100-300帧足够分析）

---

## 🚀 快速开始

### 场景1：基本验证（5分钟）

```bash
# 运行默认参数验证
python validate_rail_follower.py --bag /path/to/recording.bag
```

**预期输出：**
```
================================================================================
  VALIDATION ENGINE - Processing 247 frames
================================================================================
  Config:
    Angle range: [-75.0°, 75.0°]
    Range: [0.3m, 5.0m]
    RANSAC: 50 iters, thresh=0.08m
    Expected rail width: 0.6m ± 0.3m
================================================================================

Processing: 100%|██████████| 247/247 [00:05<00:00, 46.2frame/s]

================================================================================
  VALIDATION RESULTS SUMMARY
================================================================================
  Total frames:     247
  Successful:       223 (90.3%)
  Failed:           24 (9.7%)

  Confidence:
    Average:         0.847
    Range:           [0.512, 0.982]
    Std Dev:         0.098

  Performance:
    Avg processing:  4.23 ms/frame
    Max processing:  12.8 ms
    P99 latency:     9.1 ms

  Rail Geometry:
    Avg width:       0.634 m ± 0.089

  Failure Reasons:
    • Geometric validation failed: 18 frames (7.3%)
    • Insufficient points: 6 frames (2.4%)

================================================================================

================================================================================
  VALIDATION COMPLETE
================================================================================
  Input: recording.bag
  Frames processed: 247
  Success rate: 90.3%
  Verdict: ✅ EXCELLENT - Ready for deployment!
================================================================================
```

### 场景2：完整分析 + 可视化（10分钟）

```bash
python validate_rail_follower.py \
    --bag /path/to/recording.bag \
    --generate-gif \
    --generate-plots \
    --export-report \
    --gif-fps 5 \
    --output-dir ./validation_results
```

**生成的文件：**
```
validation_results/
├── detection_animation.gif        # 动画GIF（每帧检测结果）
├── stats_overview.png            # 统计总览图（4子图）
├── stats_failures.png            # 失败原因饼图
└── validation_report.json        # 详细JSON报告（每帧数据）
```

### 场景3：参数调优（15分钟）

```bash
# 自动搜索最佳参数组合
python validate_rail_follower.py \
    --bag /path/to/recording.bag \
    --optimize \
    --params ransac_threshold expected_rail_width range_max \
    --generate-plots
```

**输出示例：**
```
================================================================================
  PARAMETER OPTIMIZATION - Grid Search
================================================================================
  Parameters to optimize: ['ransac_threshold', 'expected_rail_width', 'range_max']
  Search space size: 125

Testing configs: 100%|██████████| 125/125 [01:23<00:00, 1.50config/s]

================================================================================
  TOP-5 PARAMETER COMBINATIONS
================================================================================
Rank  Success Rate   Avg Conf     Avg Time     Parameters
----- --------------- ---------- ---------- ----------------------------------
1     94.7%          0.891      3.8ms      ransac_threshold=0.10, expected_rail_width=0.72, range_max=6.0
2     93.5%          0.883      3.5ms      ransac_threshold=0.08, expected_rail_width=0.70, range_max=5.0
3     92.3%          0.876      3.2ms      ransac_threshold=0.12, expected_rail_width=0.68, range_max=5.0
4     91.1%          0.869      4.1ms      ransac_threshold=0.10, expected_rail_width=0.68, range_max=5.0
5     89.9%          0.862      3.9ms      ransac_threshold=0.08, expected_rail_width=0.72, range_max=4.0

================================================================================
```

---

## 📊 输出文件详解

### 1️⃣ **检测动画GIF** (`detection_animation.gif`)

![GIF预览示意](https://via.placeholder.com/400x300?text=Detection+Animation)

**内容：**
- 每帧显示裁剪后的激光点云（灰色）
- 左轨内点（红色）和右轨内点（蓝色）
- 拟合的左轨线（红）、右轨线（蓝）、中心线（绿）
- 小车位置（黄色三角形）
- PurePursuit前视点（黄色圆圈）
- 实时状态信息（成功/失败、置信度、控制指令）

**用途：**
- 直观观察算法行为
- 识别系统性失败模式
- 向非技术人员展示效果

---

### 2️⃣ **统计总览图** (`stats_overview.png`)

包含4个子图：

| 子图 | 内容 | 分析要点 |
|------|------|---------|
| **左上** | 成功率时间序列 | 是否有连续失败段？→ 可能是特定场景问题 |
| **右上** | 置信度时间序列 | 波动是否大？→ 参数可能不够鲁棒 |
| **左下** | 处理时间直方图 | P99是否<10ms？→ 实时性保证 |
| **右下** | 轨距分布 | 分布是否集中？→ 轨道几何一致性 |

---

### 3️⃣ **详细JSON报告** (`validation_report.json`)

```json
{
  "metadata": {
    "generated_at": "2026-05-06 14:30:00",
    "total_frames": 247,
    "config": { ... }
  },
  "statistics": {
    "success_rate": 0.903,
    "avg_confidence": 0.847,
    ...
  },
  "per_frame_results": [
    {
      "frame_idx": 0,
      "success": true,
      "confidence": 0.923,
      "rail_width": 0.651,
      "control_cmd": [0.33, 0.045],
      "processing_time_ms": 3.8,
      ...
    },
    ...
  ]
}
```

**用途：**
- 自动化测试集成（CI/CD）
- 长期性能回归追踪
- 与其他算法版本对比

---

## 🎯 结果解读指南

### 成功率阈值

| 成功率 | 含义 | 建议 |
|-------|------|------|
| **>95%** | 优秀 | 直接部署到实车 |
| **85-95%** | 良好 | 微调参数后部署 |
| **70-85%** | 一般 | 需要--optimize或调整算法 |
| **<70%** | 较差 | 检查数据质量或重新设计算法 |

### 常见失败模式及对策

#### ❌ **"Insufficient points after cropping"**

**原因：** 裁剪后剩余点太少  
**症状：** 连续多帧失败  
**对策：**
```bash
# 放宽裁剪参数
python validate_rail_follower.py --bag x.bag \
    --range-max 8.0 \          # 增加最远距离
    --lateral-max 3.5 \        # 增加横向范围
    --angle-min -90 \          # 扩大角度范围
    --angle-max 90
```

#### ❌ **"Geometric validation failed"**

**原因：** RANSAC拟合的直线不满足平行度/轨距约束  
**症状：** 失败帧分散，无规律  
**对策：**
```bash
# 放宽几何约束
python validate_rail_follower.py --bag x.bag \
    --width-tolerance 0.5 \     # 增加轨距容差
    --expected-width 0.8 \      # 调整预期轨距
    # 或优化参数
    --optimize --params expected_rail_width parallel_threshold_deg
```

#### ❌ **"Very few points"**

**原因：** 激光被严重遮挡或反射异常  
**症状：** 偶发单帧失败  
**对策：** 通常可忽略，算法有容错机制。如果频繁出现，检查：
- 激光雷达镜头是否脏污
- 是否有强反光物体（金属、玻璃）
- 天气条件（雨雾会衰减激光）

---

## 🔬 高级用法

### 1. 对比两个Bag文件

```bash
# 测试不同场景
echo "=== Scenario A: Straight track ==="
python validate_rail_follower.py --bag straight_track.bag -o results_A

echo -e "\n=== Scenario B: Curved track ==="
python validate_rail_follower.py --bag curved_track.bag -o results_B

# 手动对比结果
diff <(jq '.statistics.success_rate' results_A/validation_report.json) \
     <(jq '.statistics.success_rate' results_B/validation_report.json)
```

### 2. 回归测试（代码修改后）

```bash
#!/bin/bash
# regression_test.sh

BAGS=("test1.bag" "test2.bag" "test3.bag")
EXPECTED_RATES=(0.95 0.90 0.88)  # 基准成功率

for i in "${!BAGS[@]}"; do
    echo "Testing ${BAGS[$i]}..."
    RATE=$(python validate_rail_follower.py --bag ${BAGS[$i]} --export-report \
        | grep "Success rate:" | awk '{print $NF}' | tr -d '%')
    
    if (( $(echo "$RATE >= ${EXPECTED_RATES[$i]}" | bc -l) )); then
        echo "✅ PASS ($RATE%)"
    else
        echo "❌ FAIL ($RATE%, expected ≥${EXPECTED_RATES[$i]}%)"
        exit 1
    fi
done

echo -e "\n✅ All regression tests passed!"
```

### 3. 提取特定时间段分析

```bash
# 使用rosbag filter提取感兴趣的时间段
rosbag filter input.bag output.bag "t.secs > 30.0 and t.secs < 35.0"

# 只分析这段数据
python validate_rail_follower.py --bag output.bag
```

### 4. 性能基准测试

```bash
# 测试不同RANSAC迭代次数的性能影响
for iters in 20 50 80 100 150; do
    echo "Iterations: $iters"
    python validate_rail_follower.py --bag test.bag \
        --ransac-iters $iters \
        --export-report \
        -o results_iters_$iters
    
    # 提取关键指标
    jq '{iters: $iters, success: .statistics.success_rate, avg_time: .statistics.avg_processing_time_ms}' \
       results_iters_$iters/validation_report.json
done
```

---

## 🐛 故障排查

### 问题1：`ModuleNotFoundError: No module named 'rosbag'`

**解决方案：**
```bash
# Ubuntu/Debian
sudo apt-get install python3-rosbag

# 或使用conda（如果用Anaconda）
conda install -c conda-forge rosbag
```

### 问题2：`[ERROR] No laser scan data found in bag!`

**检查步骤：**
```bash
# 1. 确认话题名称
rosbag info your.bag | grep scan

# 2. 如果不是/scan，指定正确的话题
python validate_rail_follower.py --bag your.bag --topic /laser/scan

# 3. 确认消息类型是LaserScan
rosbag info your.bag | grep -A1 "sensor_msgs/LaserScan"
```

### 问题3：成功率很低(<50%)

**诊断流程：**
```bash
# 1. 先看可视化结果
python validate_rail_follower.py --bag your.bag --generate-gif

# 2. 打开GIF观察：
#    - 能否看到两条平行的轨道线？
#    - 点云分布是否符合预期？
#    - 失败帧有什么特殊之处？

# 3. 尝试放宽参数
python validate_rail_follower.py --bag your.bag \
    --range-max 10.0 \
    --ransac-threshold 0.15 \
    --width-tolerance 0.5

# 4. 如果仍然低，可能是数据质量问题
#    → 检查录制时小车位置是否在轨道上
#    → 检查激光雷达是否正常工作
```

### 问题4：生成的GIF太大/太长

**解决方案：**
```bash
# 降低帧率和数量
python validate_rail_follower.py --bag your.bag \
    --generate-gif \
    --gif-fps 2 \           # 降低FPS（默认5）
    --max-gif-frames 50    # 减少最大帧数（默认100）
```

---

## 📈 典型工作流

### 开发阶段

```
1. 录制bag文件（实车或仿真）
   ↓
2. 运行基本验证
   $ python validate_rail_follower.py --bag test.bag
   ↓
3. 查看成功率（目标>90%）
   ↓
4. 如果<90%，运行参数优化
   $ python validate_rail_follower.py --bag test.bag --optimize
   ↓
5. 使用最佳参数重新验证
   ↓
6. 生成可视化报告审核
   $ python validate_rail_follower.py --bag test.bag --generate-gif --generate-plots
   ↓
7. 人工确认效果OK
   ↓
8. 将参数移植到C++代码部署
```

### 生产阶段

```
每次代码修改后：
1. 运行回归测试套件
   $ bash regression_test.sh
2. 如果通过 → 合并代码
3. 如果失败 → 回归分析原因
```

---

## 🎓 算法原理参考

本工具实现的算法与C++版本完全一致：

1. **点云裁剪**: O(N) - 角度/距离/横向三重过滤
2. **RANSAC直线拟合**: O(K·N) - 随机采样+提前终止+PCA优化
3. **几何验证**: 平行度+轨距双重约束
4. **PurePursuit+Stanley控制**: O(1) - 混合路径跟踪

详见源码中的注释和[RAIL_FOLLOWER_PERFORMANCE_ANALYSIS.md](RAIL_FOLLOWER_PERFORMANCE_ANALYSIS.md)

---

**文档版本**: v1.0  
**最后更新**: 2026-05-06  
**适用工具**: validate_rail_follower.py
