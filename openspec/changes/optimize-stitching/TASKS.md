# 任务：车底全景图拼接优化

## 任务清单

### Phase 1: 内容变化检测模块（可独立开发）

- [ ] **T1.1**: 实现 `compute_change_score(frame1, frame2)` 函数
  - **文件**: `content_detector.py`
  - **独立验收**: 返回值范围正确，边界条件 E1.1-E1.5 处理正确
  - **测试**: T1-T7（独立运行）

- [ ] **T1.2**: 实现 `detect_interest_regions(frame1, frame2)` 函数
  - **文件**: `content_detector.py`
  - **独立验收**: 返回格式正确，小区域过滤生效
  - **依赖**: T1.1（同一文件内函数调用）

- [ ] **T1.3**: 单元测试 T1.1 和 T1.2
  - **文件**: `tests/test_content_detector.py`
  - **独立验收**: 7个测试用例通过，覆盖率>90%

---

### Phase 2: 自适应采样引擎（可独立开发）

- [ ] **T2.1**: 实现 `AdaptiveSampler` 类（核心逻辑）
  - **文件**: `adaptive_sampler.py`
  - **独立验收**: `get_next_interval` 通过 T8-T11，边界 E2.1-E2.5 正确
  - **依赖**: `StitcherConfig`（从 config.py 导入）

- [ ] **T2.2**: 实现早停机制
  - **文件**: `adaptive_sampler.py`
  - **独立验收**: `should_early_stop` 通过 T17-T18
  - **依赖**: T2.1（同一类的方法）

- [ ] **T2.3**: 实现 `SampleRecord` 数据结构和轨迹记录
  - **文件**: `adaptive_sampler.py`
  - **独立验收**: 记录可导出为 JSON
  - **依赖**: T2.1（同一类中使用）

---

### Phase 3: 自然接缝检测（可独立开发）

- [ ] **T3.1**: 实现 `find_seam_line(overlap_region, fallback_to_center)` 函数
  - **文件**: `seam_detector.py`
  - **独立验收**: 返回 shape 正确，边界 E3.1-E3.4 处理正确
  - **测试**: T12-T14（独立运行）

- [ ] **T3.2**: 实现 `blend_with_seam(frame1, frame2, seam_line)` 函数
  - **文件**: `seam_blender.py`
  - **独立验收**: 边界 E4.1-E4.6 处理正确，smoothstep 加权生效
  - **测试**: T15-T16（独立运行）

- [ ] **T3.3**: 实现 `simple_alpha_blend` 备选融合方法
  - **文件**: `seam_blender.py`
  - **独立验收**: 线性 alpha 过渡正常，可单独调用
  - **依赖**: T3.2（同一文件内函数）

---

### Phase 4: 图像增强（可独立开发）

- [ ] **T4.1**: 实现 `multiscale_sharpen(image)` 函数
  - **文件**: `image_enhancer.py`
  - **独立验收**: 空图像/全零图像不崩溃，输出尺寸与输入一致

- [ ] **T4.2**: 实现 `optimize_contrast(image)` 函数
  - **文件**: `image_enhancer.py`
  - **独立验收**: 单通道/多通道都能正确处理

- [ ] **T4.3**: 实现 `enhance(image, config)` 集成函数
  - **文件**: `image_enhancer.py`
  - **独立验收**: 根据 config 正确启用/禁用增强
  - **依赖**: T4.1, T4.2（同一文件内函数调用）

---

### Phase 5: 输入输出模块（可独立开发）

- [ ] **T5.1**: 实现 `VideoReader` 类
  - **文件**: `video_reader.py`
  - **独立验收**: 能读取视频帧，损坏帧返回 None，支持随机访问
  - **依赖**: `exceptions.py`, `logger.py`

- [ ] **T5.2**: 实现 `StitcherConfig` 数据类和验证
  - **文件**: `config.py`
  - **独立验收**: 参数范围验证正确，环境变量读取正常

- [ ] **T5.3**: 实现 `OutputWriter` 分段保存功能
  - **文件**: `output_writer.py`
  - **独立验收**: 单段高度 <= 25000，目录不存在时自动创建

---

### Phase 6: 异常处理与日志（可独立开发）

- [ ] **T6.1**: 定义异常层次结构
  - **文件**: `exceptions.py`
  - **独立验收**: 5种异常类型完整定义，继承关系正确

- [ ] **T6.2**: 实现结构化日志记录
  - **文件**: `logger.py`
  - **独立验收**: 输出包含时间戳、模块名、错误类型

- [ ] **T6.3**: 实现降级策略管理器
  - **文件**: `degrade_manager.py`
  - **独立验收**: 内存监控、接缝失败计数、性能监控正确工作
  - **依赖**: `logger.py`, `StitcherConfig`

---

### Phase 7: 主流程集成（依赖所有模块）

- [ ] **T7.1**: 实现 `AdaptiveStitcher.__init__` 和组件初始化
  - **文件**: `stitcher.py`
  - **独立验收**: 所有组件正确初始化，配置正确传递

- [ ] **T7.2**: 实现 `AdaptiveStitcher.stitch` 主循环
  - **文件**: `stitcher.py`
  - **独立验收**: 采样→检测→融合→增强流程正确执行
  - **依赖**: T5.1, T2.1, T1.1, T3.1, T3.2, T4.3, T6.3

- [ ] **T7.3**: 实现 `AdaptiveStitcher` 进度显示和统计收集
  - **文件**: `stitcher.py`
  - **独立验收**: 每25帧输出进度，最终统计正确

---

### Phase 8: CLI 和 API（依赖 stitcher）

- [ ] **T8.1**: 实现 CLI 入口 `main.py`
  - **文件**: `main.py`
  - **独立验收**: argparse 参数解析正确，调用 stitcher 正常

- [ ] **T8.2**: 实现包 `__init__.py` 导出
  - **文件**: `__init__.py`
  - **独立验收**: `from optimize_stitching import AdaptiveStitcher, StitcherConfig` 正常

---

### Phase 9: 测试（依赖对应模块）

- [ ] **T9.1**: 内容检测单元测试
  - **文件**: `tests/test_content_detector.py`
  - **依赖**: T1.1, T1.2

- [ ] **T9.2**: 自适应采样单元测试
  - **文件**: `tests/test_adaptive_sampler.py`
  - **依赖**: T2.1, T2.2

- [ ] **T9.3**: 接缝检测单元测试
  - **文件**: `tests/test_seam_detector.py`
  - **依赖**: T3.1

- [ ] **T9.4**: 接缝融合单元测试
  - **文件**: `tests/test_seam_blender.py`
  - **依赖**: T3.2, T3.3

- [ ] **T9.5**: 图像增强单元测试
  - **文件**: `tests/test_image_enhancer.py`
  - **依赖**: T4.1, T4.2

- [ ] **T9.6**: 端到端集成测试
  - **文件**: `tests/test_integration.py`
  - **依赖**: 所有模块
  - **验收**: T19（损坏帧恢复）、T20（目录不存在）通过

- [ ] **T9.7**: 性能基准测试
  - **文件**: `tests/test_integration.py`
  - **验收**: 处理时间<30s，内存<2GB

---

## 任务依赖关系图

```
独立模块（无相互依赖）:
├── config.py (T5.2)
├── exceptions.py (T6.1)
├── logger.py (T6.2)

核心算法模块（依赖独立模块）:
├── content_detector.py (T1.1→T1.2→T1.3)
│   └── 依赖: numpy, cv2
├── adaptive_sampler.py (T2.1→T2.2→T2.3)
│   └── 依赖: config.py, logger.py
├── seam_detector.py (T3.1)
│   └── 依赖: numpy, cv2, logger.py
├── seam_blender.py (T3.2→T3.3)
│   └── 依赖: numpy, cv2, logger.py
├── image_enhancer.py (T4.1→T4.2→T4.3)
│   └── 依赖: numpy, cv2, config.py

输入输出模块（依赖独立模块）:
├── video_reader.py (T5.1)
│   └── 依赖: cv2, exceptions.py, logger.py
├── output_writer.py (T5.3)
│   └── 依赖: cv2, os, exceptions.py

辅助模块（依赖独立模块）:
└── degrade_manager.py (T6.3)
    └── 依赖: logger.py, config.py

集成模块（依赖所有模块）:
├── stitcher.py (T7.1→T7.2→T7.3)
│   └── 依赖: 所有模块

最终接口（依赖集成模块）:
├── main.py (T8.1)
│   └── 依赖: stitcher.py, config.py
└── __init__.py (T8.2)

测试模块（依赖对应模块）:
├── tests/test_content_detector.py (T9.1)
├── tests/test_adaptive_sampler.py (T9.2)
├── tests/test_seam_detector.py (T9.3)
├── tests/test_seam_blender.py (T9.4)
├── tests/test_image_enhancer.py (T9.5)
└── tests/test_integration.py (T9.6, T9.7)
```

---

## 独立开发验证矩阵

| 任务 | 独立开发 | 独立验证 | 依赖模块 | 验证方式 |
|------|:-------:|:-------:|---------|---------|
| T1.1 | ✅ | ✅ | numpy, cv2 | pytest 直接运行 |
| T1.2 | ✅ | ✅ | T1.1 | pytest 直接运行 |
| T1.3 | ✅ | ✅ | T1.1, T1.2 | pytest 直接运行 |
| T2.1 | ✅ | ✅ | config.py | pytest 直接运行 |
| T2.2 | ✅ | ✅ | T2.1 | pytest 直接运行 |
| T2.3 | ✅ | ✅ | T2.1 | 导出 JSON 验证 |
| T3.1 | ✅ | ✅ | numpy, cv2 | pytest 直接运行 |
| T3.2 | ✅ | ✅ | numpy, cv2 | pytest 直接运行 |
| T3.3 | ✅ | ✅ | T3.2 | pytest 直接运行 |
| T4.1 | ✅ | ✅ | numpy, cv2 | pytest 直接运行 |
| T4.2 | ✅ | ✅ | numpy, cv2 | pytest 直接运行 |
| T4.3 | ✅ | ✅ | T4.1, T4.2 | pytest 直接运行 |
| T5.1 | ✅ | ✅ | exceptions, logger | 读取测试视频 |
| T5.2 | ✅ | ✅ | dataclasses, os | pytest 直接运行 |
| T5.3 | ✅ | ✅ | cv2, os, exceptions | 写入测试目录 |
| T6.1 | ✅ | ✅ | - | pytest 直接运行 |
| T6.2 | ✅ | ✅ | logging | 检查日志输出 |
| T6.3 | ✅ | ✅ | logger, config | pytest 模拟场景 |
| T7.1 | ✅ | ✅ | 所有独立模块 | 检查组件初始化 |
| T7.2 | ❌ | ❌ | 所有模块 | 端到端测试 |
| T7.3 | ✅ | ✅ | T7.1, T7.2 | 检查统计输出 |
| T8.1 | ✅ | ✅ | stitcher, config | CLI 参数测试 |
| T8.2 | ✅ | ✅ | - | import 测试 |
| T9.1-T9.5 | ✅ | ✅ | 对应模块 | pytest |
| T9.6 | ❌ | ❌ | 所有模块 | pytest 端到端 |
| T9.7 | ❌ | ❌ | 所有模块 | pytest 性能测试 |

---

## 实施批次

### Batch 1: 独立基础设施（0依赖，可并行开发）
- T5.2 (config.py)
- T6.1 (exceptions.py)
- T6.2 (logger.py)

### Batch 2: 核心算法（仅依赖 Batch 1）
- T1.1 → T1.2 → T1.3 (content_detector.py)
- T2.1 → T2.2 → T2.3 (adaptive_sampler.py)
- T3.1 (seam_detector.py)
- T3.2 → T3.3 (seam_blender.py)
- T4.1 → T4.2 → T4.3 (image_enhancer.py)

### Batch 3: 输入输出（仅依赖 Batch 1）
- T5.1 (video_reader.py)
- T5.3 (output_writer.py)
- T6.3 (degrade_manager.py)

### Batch 4: 集成（依赖 Batch 1-3）
- T7.1 → T7.2 → T7.3 (stitcher.py)

### Batch 5: 接口（依赖 Batch 4）
- T8.1 (main.py)
- T8.2 (__init__.py)

### Batch 6: 测试（依赖对应模块，与开发并行）
- T9.1-T9.5 (单元测试)
- T9.6-T9.7 (集成测试)

---

## 验收检查点

| 检查点 | 批次 | 完成条件 |
|--------|------|---------|
| CP1 | Batch 1 | config/exceptions/logger 三个文件存在，可导入 |
| CP2 | Batch 2 | 5个核心算法模块存在，各自单元测试通过 |
| CP3 | Batch 3 | video_reader/output_writer/degrade_manager 存在，可独立验证 |
| CP4 | Batch 4 | stitcher.py 存在，端到端流程无报错 |
| CP5 | Batch 5 | CLI 和 API 正常工作 |
| CP6 | Batch 6 | 所有20个测试用例通过 |
| CP7 | 最终 | 性能<30s，质量验收通过 |

---

## 任务粒度评估

| 粒度标准 | 当前状态 | 评价 |
|---------|---------|------|
| 单一职责 | 每个任务只做一件事 | ✅ 合格 |
| 独立验证 | 大多数任务可单独测试 | ✅ 合格 |
| 时间估算 | 每个任务 30-60 分钟 | ✅ 合格 |
| 依赖清晰 | 依赖关系明确标注 | ✅ 合格 |
| 验收标准 | 每个任务有明确验收条件 | ✅ 合格 |

**唯一非独立任务**: T7.2（主循环）和 T9.6-T9.7（集成测试），这是合理的，因为它们需要所有模块就绪后才能验证。
