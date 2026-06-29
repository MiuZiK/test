# 车底全景图拼接优化模块

高性能车底全景图拼接系统，采用垂直长图拼接技术，模拟手机截长图效果。

## 功能特性

- **垂直长图拼接**: 光流对齐 + 加权融合，实现连续自然的拼接效果
- **多方法光流估计**: ECC/Farneback/ORB 多方法融合，提高鲁棒性
- **自适应采样**: 基于内容变化动态调整采样间隔，平衡速度与质量
- **自然接缝检测**: 动态规划寻找最小代价接缝线
- **图像增强**: USM锐化 + CLAHE对比度优化
- **降级策略**: 内存/性能/接缝失败三级降级保证可用性

## 项目结构

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
└── tests/                   # 测试套件 (166个用例)
```

## 快速开始

### 安装依赖

```bash
pip install opencv-python numpy pytest pytest-cov flake8
```

### 命令行使用

```bash
python -m optimize_stitching.main \
  --input video.avi \
  --output output/ \
  --base_interval 60 \
  --output_width 600
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
    cv2.imwrite(f'output_{i}.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 97])

print(f"处理帧数: {result.stats.frames_used}")
print(f"总耗时: {result.stats.total_time_s:.1f}s")
```

## 配置参数

| 参数 | 默认值 | 范围 | 说明 |
|------|--------|------|------|
| base_interval | 60 | 1-500 | 基础采样帧间隔 |
| high_threshold | 60 | 0-100 | 加密采样阈值 |
| low_threshold | 20 | 0-100 | 减密采样阈值 |
| output_width | 600 | 100-2000 | 输出图像宽度 |
| overlap_ratio | 0.15 | 0.01-0.5 | 重叠比例 |
| seamless_blend | true | bool | 是否启用自然接缝 |
| max_frames | 500 | 10-2000 | 最大处理帧数 |
| jpeg_quality | 97 | 1-100 | JPEG输出质量 |

## 运行测试

```bash
# 运行所有测试
python -m pytest optimize_stitching/tests/ -v

# 运行测试并生成覆盖率报告
python -m pytest optimize_stitching/tests/ --cov=optimize_stitching --cov-report=html

# 代码质量检查
flake8 optimize_stitching/
```

## 性能指标

| 指标 | 目标 | 实际 |
|------|------|------|
| 处理时间 | < 30秒 (200帧) | ✅ |
| 内存峰值 | < 2GB | ✅ |
| 代码覆盖率 | > 90% | ✅ 94% |
| flake8错误 | 0 | ✅ |
| 测试用例 | 20+ | ✅ 166 |

## 许可证

MIT License
