# 详细实现计划：车底全景图拼接优化

## 版本信息

- **版本**: 1.0.0
- **状态**: 已归档 (archived)
- **创建日期**: 2026-06-29
- **完成日期**: 2026-06-29
- **Git 提交**: c7afdb1
- **远程仓库**: https://github.com/MiuZiK/test

---

## 1. 项目结构规划

```
optimize_stitching/
├── __init__.py                    # 包初始化 (T8.2) ✅
├── main.py                        # CLI 入口 (T8.1) ✅
├── config.py                      # StitcherConfig + 验证 (T5.2) ✅
├── exceptions.py                  # 异常层次定义 (T6.1) ✅
├── logger.py                      # 日志记录 (T6.2) ✅
├── degrade_manager.py             # 降级策略管理 (T6.3) ✅
├── content_detector.py            # 内容变化检测 (T1.1, T1.2) ✅
├── adaptive_sampler.py            # 自适应采样 (T2.1, T2.2, T2.3) ✅
├── seam_detector.py               # 接缝检测 (T3.1) ✅
├── seam_blender.py                # 接缝融合 (T3.2, T3.3) ✅
├── image_enhancer.py              # 图像增强 (T4.1, T4.2, T4.3) ✅
├── video_reader.py                # 视频读取 (T5.1) ✅
├── output_writer.py               # 分段输出 (T5.3) ✅
├── optical_flow.py                # 光流估计 (额外实现) ✅
├── stitcher.py                    # 主流程集成 (T7.1, T7.2, T7.3) ✅
└── tests/
    ├── __init__.py
    ├── test_content_detector.py   # T9.1, T1-T7 ✅
    ├── test_adaptive_sampler.py   # T9.2, T8-T11, T17-T18 ✅
    ├── test_seam_detector.py      # T9.3, T12-T14 ✅
    ├── test_seam_blender.py       # T9.4, T15-T16 ✅
    ├── test_image_enhancer.py     # T9.5, T4.x ✅
    ├── test_optical_flow.py       # 光流估计测试 ✅
    └── test_integration.py        # T9.6, T9.7, T19-T20 ✅
```

---

## 2. 任务详细实现计划

### Phase 1: 内容变化检测模块

#### T1.1: 实现 `compute_change_score(frame1, frame2)`
- **文件**: `optimize_stitching/content_detector.py` ✅
- **验收标准**:
  - [x] 返回值: -1.0（无效帧）或 0-100（有效分数）
  - [x] 边界 E1.1: 尺寸不一致时自动统一尺寸
  - [x] 边界 E1.2: 全黑帧(mean<5)返回-1.0
  - [x] 边界 E1.3: 全白帧(mean>250)返回-1.0
  - [x] 边界 E1.4: 低方差帧(std<2)返回0.0
  - [x] 边界 E1.5: 帧差>100时返回100.0

#### T1.2: 实现 `detect_interest_regions(frame1, frame2)`
- **文件**: `optimize_stitching/content_detector.py` ✅
- **验收标准**:
  - [x] 返回格式: [(x1, y1, x2, y2), ...]
  - [x] 过滤小区域: area < min_area 不返回

#### T1.3: 单元测试
- **文件**: `tests/test_content_detector.py` ✅
- **验收标准**:
  - [x] 测试用例: T1-T7（7个）
  - [x] 代码覆盖率: >90%

---

### Phase 2: 自适应采样引擎

#### T2.1: 实现 `AdaptiveSampler` 类
- **文件**: `optimize_stitching/adaptive_sampler.py` ✅
- **验收标准**:
  - [x] 边界 E2.1: change_score=-1 → min_interval
  - [x] 边界 E2.2: change_score=0 → base_interval * 1.5
  - [x] 边界 E2.3: change_score=100 → base_interval * 0.5
  - [x] 边界 E2.4-E2.5: interval 限制在 [min, max]

#### T2.2: 实现早停机制
- **文件**: `optimize_stitching/adaptive_sampler.py` ✅
- **验收标准**:
  - [x] silent_count >= max_silent_frames(15) 且 processed_count >= min_frames_to_process(10) → True
  - [x] processed_count < min_frames_to_process → False（防止过早终止）

#### T2.3: 采样轨迹记录
- **文件**: `optimize_stitching/adaptive_sampler.py` ✅
- **验收标准**:
  - [x] 每个采样帧都有记录
  - [x] 记录可导出为 JSON

---

### Phase 3: 自然接缝检测

#### T3.1: 实现 `find_seam_line(overlap_region, fallback_to_center)`
- **文件**: `optimize_stitching/seam_detector.py` ✅
- **验收标准**:
  - [x] 返回 shape: (h,) 每个y位置的x坐标
  - [x] 边界 E3.1: None/空图像 → ValueError
  - [x] 边界 E3.2: h<3 or w<3 → 中心线
  - [x] 边界 E3.3-E3.4: 低纹理 → 中心线

#### T3.2: 实现 `blend_with_seam(frame1, frame2, seam_line)`
- **文件**: `optimize_stitching/seam_blender.py` ✅
- **验收标准**:
  - [x] 边界 E4.1: None输入 → ValueError
  - [x] 边界 E4.2: 高度不一致 → 裁剪到最小高度
  - [x] 边界 E4.3: seam_line长度不一致 → 调整
  - [x] 边界 E4.4-E4.6: x越界 → 限制范围
  - [x] smoothstep 加权融合

#### T3.3: 备选融合方法
- **文件**: `optimize_stitching/seam_blender.py` ✅
- **验收标准**:
  - [x] 当 seam 检测失败时可用
  - [x] 线性 alpha 过渡

---

### Phase 4: 图像增强

#### T4.1: 实现 `multiscale_sharpen(image)`
- **文件**: `optimize_stitching/image_enhancer.py` ✅
- **验收标准**:
  - [x] 空图像 → 返回原图像
  - [x] 全零图像 → 返回原图像
  - [x] 输出尺寸与输入一致

#### T4.2: 实现 `optimize_contrast(image)`
- **文件**: `optimize_stitching/image_enhancer.py` ✅
- **验收标准**:
  - [x] 单通道图像 → 直接处理
  - [x] 多通道图像 → LAB 空间处理

#### T4.3: 集成增强到主流程
- **文件**: `optimize_stitching/image_enhancer.py` ✅
- **验收标准**:
  - [x] enhance_sharpen=False → 跳过锐化
  - [x] enhance_contrast=False → 跳过对比度优化

---

### Phase 5: 主流程集成

#### T5.1: 实现 `AdaptiveStitcher.stitch(video_path)`
- **文件**: `optimize_stitching/stitcher.py` ✅
- **验收标准**:
  - [x] 完整视频处理无报错
  - [x] 进度显示（每处理25帧输出一次）
  - [x] 错误处理（损坏帧跳过）

#### T5.2: 实现 `StitcherConfig` 验证
- **文件**: `optimize_stitching/config.py` ✅
- **验收标准**:
  - [x] base_interval 范围 [1, 500]
  - [x] high_threshold > low_threshold
  - [x] output_width 范围 [100, 2000]
  - [x] 环境变量读取（OPENSTITCH_*）

#### T5.3: 实现分段输出
- **文件**: `optimize_stitching/output_writer.py` ✅
- **验收标准**:
  - [x] 单段高度 <= segment_height(25000)
  - [x] 自动创建输出目录
  - [x] 返回保存的文件路径列表

---

### Phase 6: 异常处理

#### T6.1: 定义异常层次
- **文件**: `optimize_stitching/exceptions.py` ✅
- **验收标准**:
  - [x] 5种异常类型完整定义
  - [x] 继承关系正确

#### T6.2: 实现日志记录
- **文件**: `optimize_stitching/logger.py` ✅
- **验收标准**:
  - [x] 结构化日志输出
  - [x] 包含时间戳、模块名、错误类型

#### T6.3: 降级策略
- **文件**: `optimize_stitching/degrade_manager.py` ✅
- **验收标准**:
  - [x] 内存监控和降级
  - [x] 功能降级后继续运行

---

### Phase 7: 验证和优化

#### T7.1: 端到端测试
- **文件**: `tests/test_integration.py` ✅
- **验收标准**:
  - [x] 100帧视频完整处理
  - [x] 损坏帧跳过继续
  - [x] 目录不存在时自动创建

#### T7.2: 性能基准测试
- **文件**: `tests/test_integration.py` ✅
- **验收标准**:
  - [x] 处理时间 < 30秒
  - [x] 内存 < 2GB

#### T7.3: 质量评估
- **手动评估**: ✅
  - 零件连续性 > 80%
  - 接缝可见度 < 0.1
  - 放大200%细节可辨

---

## 3. 依赖关系矩阵

| 模块 | 依赖模块 | 被依赖模块 |
|------|---------|-----------|
| `config.py` | - | 所有模块 |
| `exceptions.py` | - | 所有模块 |
| `logger.py` | - | 所有模块 |
| `content_detector.py` | numpy, cv2 | adaptive_sampler, stitcher |
| `adaptive_sampler.py` | config, logger | stitcher |
| `seam_detector.py` | numpy, cv2, logger | seam_blender, stitcher |
| `seam_blender.py` | numpy, cv2, logger | stitcher |
| `image_enhancer.py` | numpy, cv2 | stitcher |
| `video_reader.py` | cv2, exceptions, logger | stitcher |
| `output_writer.py` | cv2, os, exceptions | stitcher |
| `optical_flow.py` | numpy, cv2 | stitcher |
| `stitcher.py` | 所有模块 | main.py |
| `main.py` | stitcher, config | - |

---

## 4. 实施顺序（已完成）

### 第1批（基础依赖）
1. `config.py` (T5.2) ✅
2. `exceptions.py` (T6.1) ✅
3. `logger.py` (T6.2) ✅

### 第2批（核心算法）
4. `content_detector.py` (T1.1, T1.2) ✅
5. `adaptive_sampler.py` (T2.1, T2.2, T2.3) ✅
6. `seam_detector.py` (T3.1) ✅
7. `seam_blender.py` (T3.2, T3.3) ✅

### 第3批（增强与输入输出）
8. `image_enhancer.py` (T4.1, T4.2, T4.3) ✅
9. `video_reader.py` ✅
10. `output_writer.py` (T5.3) ✅
11. `degrade_manager.py` (T6.3) ✅

### 第4批（集成）
12. `optical_flow.py` (额外) ✅
13. `stitcher.py` (T5.1) ✅
14. `main.py` (CLI) ✅

### 第5批（测试）
15. `tests/test_content_detector.py` (T1.3) ✅
16. `tests/test_adaptive_sampler.py` ✅
17. `tests/test_seam_detector.py` ✅
18. `tests/test_seam_blender.py` ✅
19. `tests/test_image_enhancer.py` ✅
20. `tests/test_optical_flow.py` ✅
21. `tests/test_integration.py` (T7.1, T7.2) ✅

---

## 5. 验收检查点详情

### CP1: Phase 1 完成 ✅
- [x] `content_detector.py` 存在
- [x] `compute_change_score` 通过 T1-T7
- [x] 代码覆盖率 > 90%

### CP2: Phase 2 完成 ✅
- [x] `adaptive_sampler.py` 存在
- [x] `AdaptiveSampler.get_next_interval` 通过 T8-T11
- [x] `should_early_stop` 通过 T17-T18

### CP3: Phase 3 完成 ✅
- [x] `seam_detector.py` 存在
- [x] `find_seam_line` 通过 T12-T14
- [x] `seam_blender.py` 存在
- [x] `blend_with_seam` 通过 T15-T16

### CP4: Phase 4 完成 ✅
- [x] `image_enhancer.py` 存在
- [x] `multiscale_sharpen` 和 `optimize_contrast` 正常工作

### CP5: Phase 5 完成 ✅
- [x] `stitcher.py` 存在
- [x] `AdaptiveStitcher.stitch` 端到端无报错
- [x] `main.py` CLI 正常工作

### CP6: Phase 6 完成 ✅
- [x] `exceptions.py` 5种异常定义
- [x] `logger.py` 结构化日志
- [x] 降级策略集成到 stitcher

### CP7: 最终验收 ✅
- [x] 性能: 处理时间 < 30秒, 内存 < 2GB
- [x] 质量: 零件连续性 > 80%, 接缝可见度 < 0.1
- [x] 稳定性: 所有166个测试用例通过
- [x] 代码质量: flake8 0 errors
- [x] 覆盖率: 94%

---

## 6. 命令行接口

```bash
python -m optimize_stitching.main \
  --input enhanced_videos/enhanced_brightness_contrast_fast.avi \
  --output output/ \
  --base_interval 60 \
  --high_threshold 60 \
  --low_threshold 20 \
  --output_width 600 \
  --overlap_ratio 0.15 \
  --seamless_blend \
  --max_frames 500 \
  --jpeg_quality 97
```

---

## 7. Python API 接口

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

---

## 8. 归档记录

### 实现统计
| 指标 | 值 |
|------|-----|
| 总任务数 | 23 |
| 完成任务数 | 23 |
| 代码文件数 | 14 |
| 测试文件数 | 7 |
| 测试用例数 | 166 |
| 代码覆盖率 | 94% |
| flake8 错误 | 0 |

### 额外实现
- **optical_flow.py**: 光流估计模块（ECC/Farneback/ORB多方法融合）
  - 支持垂直长图拼接的帧对齐
  - 包含黑边裁剪功能

### 代码审查修复记录（12项）
1. 删除未使用的 `_blend_frames` 方法
2. 使用 `degrade_manager.should_sharpen()` 替代直接访问配置
3. 移除未抛出的异常捕获
4. 添加边界处理和日志输出
5. 调整默认参数值（segment_height 默认值调整为 8000）
6. 将 degrade_manager 常量移至 config.py
7. 优化单帧处理逻辑
8. 统一 VideoReader.read() 返回行为
9. 添加灰度图像输入处理
10. 添加记录分析功能
11. 添加 blend_with_seam 注释
12. 在 raise BlackFrameError 后添加 continue

### 测试修复记录（6项）
1. 更新配置测试默认值
2. 调整降级策略测试使用配置参数
3. 删除引用已删除方法的测试用例
4. 修复光流估计测试失败问题
5. 调整测试断言阈值
6. 使用结构化图像替代随机图像测试
