# 图像拼接算法

## 模块结构

```
optimize_stitching/
├── __init__.py              # 包导出
├── main.py                  # CLI入口
├── config.py                # 配置管理
├── exceptions.py            # 异常定义
├── logger.py                # 日志记录
├── degrade_manager.py       # 降级策略
├── content_detector.py      # 内容变化检测
├── adaptive_sampler.py      # 自适应采样
├── seam_detector.py         # 接缝检测
├── seam_blender.py          # 接缝融合
├── image_enhancer.py        # 图像增强
├── video_reader.py          # 视频读取
├── output_writer.py         # 分段输出
├── optical_flow.py          # 光流估计
├── stitcher.py              # 主流程集成
└── tests/                   # 单元测试 (166个用例)
```

## 核心算法

### 1. 光流估计

**文件**: `optical_flow.py`

多方法融合策略：
- ECC算法：精准对齐
- Farneback算法：稠密光流
- ORB特征匹配：特征点对齐
- 回退策略：当所有方法失败时使用简单平移

### 2. 自适应采样

**文件**: `adaptive_sampler.py`

基于内容变化动态调整采样间隔：
- 高变化区域：加密采样（间隔缩小50%）
- 低变化区域：减密采样（间隔增大50%）
- 早停机制：连续15帧无变化且已处理10帧以上时停止

### 3. 接缝检测

**文件**: `seam_detector.py`

动态规划寻找最小代价接缝线：
- 梯度代价函数
- 平滑约束
- 低纹理区域回退到中心线

### 4. 接缝融合

**文件**: `seam_blender.py`

smoothstep加权融合：
- 在接缝线两侧创建渐变过渡
- 消除拼接痕迹
- 备选：线性alpha融合

### 5. 图像增强

**文件**: `image_enhancer.py`

- USM锐化：提升细节清晰度
- CLAHE对比度优化：增强暗部细节

## 使用方法

### CLI方式

```bash
python -m optimize_stitching.main \
  --input video.avi \
  --output output/ \
  --base_interval 60 \
  --output_width 600 \
  --max_frames 500
```

### Python API

```python
from optimize_stitching import AdaptiveStitcher, StitcherConfig

config = StitcherConfig(
    base_interval=60,
    output_width=600,
    seamless_blend=True
)

stitcher = AdaptiveStitcher(config)
result = stitcher.stitch('input.avi')

for i, img in enumerate(result.images):
    cv2.imwrite(f'output_{i}.jpg', img)
```

## 配置参数

| 参数 | 默认值 | 范围 | 说明 |
|------|--------|------|------|
| base_interval | 60 | 1-500 | 基础采样帧间隔 |
| high_threshold | 60 | 0-100 | 加密采样阈值 |
| low_threshold | 20 | 0-100 | 减密采样阈值 |
| output_width | 600 | 100-2000 | 输出图像宽度 |
| overlap_ratio | 0.15 | 0.01-0.5 | 重叠比例 |
| seamless_blend | true | bool | 自然接缝融合 |
| max_frames | 500 | 10-2000 | 最大处理帧数 |
| jpeg_quality | 97 | 1-100 | JPEG输出质量 |

## 性能指标

| 指标 | 值 |
|------|-----|
| 处理时间 | < 30秒 (200帧) |
| 内存峰值 | < 2GB |
| 代码覆盖率 | 94% |
| 测试用例 | 166 |
