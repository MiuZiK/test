# 规格：车底全景图拼接优化

## 1. 功能规格

### 1.1 核心功能

#### F1: 内容变化检测
- **输入**：连续两帧图像
- **输出**：
  - `change_score`: -1.0（无效帧）、0-100（有效）
  - `change_map`: 二值变化图
  - `interest_regions`: 变化密集区域列表
- **算法**：
  ```
  1. 转换为灰度
  2. 计算梯度幅值 (Sobel)
  3. 计算帧差绝对值
  4. 综合评分: change_score = α * mean(diff) + β * edge_density
  ```
- **边界条件**：
  - 尺寸不一致 → 自动裁剪到最小公共区域
  - 全黑帧(mean<5) → 返回 -1.0（无效）
  - 全白帧(mean>250) → 返回 -1.0（无效）
  - 低方差帧(std<2) → 返回 0.0（无变化）
  - 帧差>100 → 返回 100.0（截断）

#### F2: 自适应采样
- **输入**：视频流 + 当前采样位置
- **输出**：下一个采样帧索引
- **规则**：
  ```
  if change_score < 0:           # 无效帧
      next_interval = min_interval
  elif change_score > 60:         # 密集区域
      next_interval = base_interval * 0.5
  elif change_score > 20:        # 正常区域
      next_interval = base_interval
  else:                           # 稀疏区域
      next_interval = base_interval * 1.5
  ```
- **约束**：
  - `min_interval <= interval <= max_interval`
  - `interval <= remaining_frames`（防止越界）

#### F3: 自然接缝检测
- **输入**：两帧待融合区域
- **输出**：最优接缝线位置
- **算法**：
  ```
  1. 在重叠区计算梯度幅度图
  2. 使用动态规划寻找最小代价路径
  3. 路径作为自然接缝线
  ```
- **边界条件**：
  - 空图像 → 抛出 `ValueError`
  - 过小区域(h<3 or w<3) → 返回中心线
  - 全零能量图 → 返回中心线
  - 动态规划异常 → 返回中心线

#### F4: 接缝融合
- **输入**：frame1, frame2, seam_line, overlap_width
- **输出**：融合后图像
- **算法**：
  ```
  1. 逐行扫描
  2. seam_line[y] 左侧用 frame1
  3. seam_line[y] 右侧用 frame2
  4. 混合区使用 smoothstep 加权
  ```
- **边界条件**：
  - 高度不一致 → 裁剪到最小高度
  - seam_line 长度不一致 → 调整到 h
  - x_seam 越界 → 限制到 [0, w1)

#### F5: 多尺度锐化
- **输入**：融合后的全景图
- **输出**：锐化增强后的图像
- **参数**：
  ```
  - USM amount: 1.5
  - USM radius: 2.5px
  - 对比度增强: CLAHE clip=3.5
  ```

### 1.2 用户可配置参数

| 参数 | 默认值 | 范围 | 说明 |
|------|--------|------|------|
| `base_interval` | 60 | 1-500 | 基础采样帧间隔 |
| `high_threshold` | 60 | 0-100 | 加密采样阈值 |
| `low_threshold` | 20 | 0-100 | 减密采样阈值 |
| `min_interval` | 5 | 1-50 | 最小采样间隔 |
| `max_interval` | 200 | 100-1000 | 最大采样间隔 |
| `overlap_ratio` | 0.15 | 0.01-0.5 | 重叠比例 |
| `output_width` | 600 | 100-2000 | 输出图像宽度 |
| `seamless_blend` | true | bool | 是否启用自然接缝 |
| `fallback_to_center` | true | bool | 接缝失败时退到中心线 |
| `max_frames` | 500 | 10-2000 | 最大处理帧数 |
| `max_silent_frames` | 15 | 5-50 | 早停阈值 |
| `jpeg_quality` | 97 | 1-100 | JPEG输出质量 |

---

## 2. 性能规格

### 2.1 处理性能

| 指标 | 要求 | 降级触发条件 |
|------|------|-------------|
| 处理时间 | < 30秒 (200帧) | >60s禁用锐化 |
| 内存峰值 | < 2GB | >1.5GB减少output_width |
| 帧处理速率 | > 10 fps | <5fps启用缓存 |

### 2.2 输出质量

| 指标 | 要求 | 测量方法 |
|------|------|---------|
| 零件连续性 | > 80% | 人工评估 |
| 接缝可见度 | < 0.1 (0-1) | 边缘检测自动化 |
| 清晰度 | 边缘清晰 | 梯度幅度测量 |
| 色彩一致性 | ΔE < 10 | CIE Lab色差 |

---

## 3. 接口规格

### 3.1 命令行接口

```bash
python optimize_stitching.py \
  --input <video_path> \
  --output <output_dir> \
  --base_interval 60 \
  --high_threshold 60 \
  --low_threshold 20 \
  --overlap_ratio 0.15 \
  --output_width 600 \
  --seamless_blend \
  --max_frames 500 \
  --jpeg_quality 97
```

### 3.2 Python API

```python
from optimize_stitching import AdaptiveStitcher, StitcherConfig

config = StitcherConfig(
    base_interval=60,
    high_threshold=60.0,
    low_threshold=20.0,
    overlap_ratio=0.15,
    output_width=600,
    seamless_blend=True,
    fallback_to_center=True,
    max_frames=500,
    max_silent_frames=15,
    jpeg_quality=97
)

stitcher = AdaptiveStitcher(config)
result = stitcher.stitch('input_video.avi')
result.save('output.jpg', quality=97)
```

### 3.3 返回值

```python
@dataclass
class StitchResult:
    images: List[np.ndarray]       # 分段图像
    stats: ProcessingStats          # 处理统计
    config: StitcherConfig         # 使用配置

@dataclass
class ProcessingStats:
    total_frames_read: int         # 读取帧数
    frames_used: int              # 使用帧数
    frames_skipped: int           # 跳过帧数
    frames_invalid: int            # 无效帧数
    early_stopped: bool            # 是否早停
    total_time_s: float           # 总耗时
    avg_interval: float           # 平均间隔
```

---

## 4. 异常规格

### 4.1 异常层次

```
StitcherError (base)
├── VideoReadError
│   └── CorruptFrameError
├── InvalidFrameError
│   ├── BlackFrameError
│   └── WhiteFrameError
├── SeamDetectionError
├── BlendError
└── OutputWriteError
```

### 4.2 异常处理策略

| 异常类型 | 处理策略 | 是否终止 |
|---------|---------|---------|
| `VideoReadError` | 跳过帧，继续 | 否 |
| `InvalidFrameError` | 标记+跳过 | 否 |
| `SeamDetectionError` | 回退到中心线 | 否 |
| `BlendError` | 使用简单alpha融合 | 否 |
| `OutputWriteError` | 重试3次，写入临时目录 | 是 |

---

## 5. 数据规格

### 5.1 输入数据
- **格式**：AVI/MP4 视频
- **分辨率**：806×452 (已去畸变增强)
- **帧率**：30 fps
- **总帧数**：~12,000

### 5.2 输出数据
- **格式**：JPEG
- **分辨率**：宽度可配置 (默认600px), 高度按内容自适应
- **质量**：quality=97

### 5.3 中间数据
- **采样记录**：JSON格式
- **接缝线**：numpy .npy格式
- **能量图**：numpy .npy格式（调试用）

---

## 6. 验收标准

### 6.1 功能验收

- [ ] 自适应采样正常工作
- [ ] 自然接缝检测找到合理接缝点
- [ ] 多尺度锐化提升清晰度
- [ ] 输出尺寸符合配置
- [ ] 边界条件正确处理（全黑/全白/尺寸不一致）
- [ ] 异常情况正确fallback

### 6.2 质量验收

- [ ] 零件连续性 > 80%
- [ ] 无明显接缝线
- [ ] 放大200%后细节可辨
- [ ] 完整覆盖车底

### 6.3 性能验收

- [ ] 处理时间 < 30秒
- [ ] 内存占用 < 2GB
- [ ] 无内存泄漏

### 6.4 稳定性验收

- [ ] 空视频输入不崩溃
- [ ] 损坏帧输入不中断
- [ ] 无效配置参数不导致错误
- [ ] 磁盘满时优雅失败

---

## 7. 测试用例

### T1: 正常场景
```
输入: 两帧有中等差异的图像
条件: mean(gray1)=128, 30<diff<60
预期: 0 < change_score < 60
```

### T2: 相同帧
```
输入: 两帧完全相同的图像
预期: change_score = 0
```

### T3: 全黑帧
```
输入: frame1 = 全黑 (mean<5), frame2 = 正常
预期: change_score = -1.0
```

### T4: 全白帧
```
输入: frame1 = 全白 (mean>250), frame2 = 正常
预期: change_score = -1.0
```

### T5: 低方差帧
```
输入: 两帧都是纯色 (std<2)
预期: change_score = 0
```

### T6: 尺寸不一致
```
输入: frame1=100x100, frame2=80x120
预期: change_score 正常计算，不抛异常
```

### T7: 极端帧差
```
输入: frame1 = 全黑, frame2 = 全白
预期: change_score = 100 (截断)
```

### T8: 采样加密
```
输入: change_score = 70 (>60)
预期: interval = base_interval * 0.5
```

### T9: 采样减密
```
输入: change_score = 10 (<20)
预期: interval = base_interval * 1.5
```

### T10: 采样无效帧
```
输入: change_score = -1.0
预期: interval = min_interval
```

### T11: 间隔边界
```
输入: base_interval=5, min_interval=3, max_interval=10
输入: change_score 触发加密 *0.5 = 2.5
预期: interval = 3 (min_interval 下限)
```

### T12: 接缝-空图像
```
输入: overlap_region = None
预期: 抛出 ValueError
```

### T13: 接缝-过小区域
```
输入: overlap_region.shape = (2, 2, 3)
预期: 返回中心线 (seam_line[y] = 1)
```

### T14: 接缝-低纹理
```
输入: 全零图像
预期: fallback_to_center=True → 返回中心线
预期: fallback_to_center=False → 抛出 ValueError
```

### T15: 融合-高度不一致
```
输入: frame1.shape = (100, 200, 3), frame2.shape = (80, 200, 3)
预期: 正常融合，h = 80
```

### T16: 融合-seam_line越界
```
输入: x_seam = -1 或 x_seam = w1+10
预期: 限制到 [0, w1) 范围内
```

### T17: 早停-正常
```
输入: 连续 15 帧 change_score < 20，且已处理 > 50 帧
预期: early_stopped = True
```

### T18: 早停-不足帧数
```
输入: 连续 15 帧 change_score < 20，但已处理 < 10 帧
预期: early_stopped = False（防止过早终止）
```

### T19: 视频读取失败
```
输入: 损坏的视频文件
预期: 跳过损坏帧，继续处理
预期: 最终 frames_invalid > 0
```

### T20: 输出目录不存在
```
输入: output_dir = "/nonexistent/path/"
预期: 尝试创建目录，成功则继续
预期: 创建失败则抛出 OutputWriteError
```
