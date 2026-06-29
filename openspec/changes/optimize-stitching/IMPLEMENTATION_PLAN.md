# 详细实现计划：车底全景图拼接优化

## 1. 项目结构规划

```
optimize_stitching/
├── __init__.py                    # 包初始化 (T8.2)
├── main.py                        # CLI 入口 (T8.1)
├── config.py                      # StitcherConfig + 验证 (T5.2)
├── exceptions.py                  # 异常层次定义 (T6.1)
├── logger.py                      # 日志记录 (T6.2)
├── degrade_manager.py             # 降级策略管理 (T6.3)
├── content_detector.py            # 内容变化检测 (T1.1, T1.2)
├── adaptive_sampler.py            # 自适应采样 (T2.1, T2.2, T2.3)
├── seam_detector.py               # 接缝检测 (T3.1)
├── seam_blender.py                # 接缝融合 (T3.2, T3.3)
├── image_enhancer.py              # 图像增强 (T4.1, T4.2, T4.3)
├── video_reader.py                # 视频读取 (T5.1)
├── output_writer.py               # 分段输出 (T5.3)
├── stitcher.py                    # 主流程集成 (T7.1, T7.2, T7.3)
└── tests/
    ├── __init__.py
    ├── test_content_detector.py   # T9.1, T1-T7
    ├── test_adaptive_sampler.py   # T9.2, T8-T11, T17-T18
    ├── test_seam_detector.py      # T9.3, T12-T14
    ├── test_seam_blender.py       # T9.4, T15-T16
    ├── test_image_enhancer.py     # T9.5, T4.x
    └── test_integration.py        # T9.6, T9.7, T19-T20
```

---

## 2. 任务详细实现计划

### Phase 1: 内容变化检测模块

#### T1.1: 实现 `compute_change_score(frame1, frame2)`
- **文件**: `optimize_stitching/content_detector.py`
- **函数签名**:
  ```python
  def compute_change_score(frame1: np.ndarray, frame2: np.ndarray) -> float:
      """计算两帧内容变化分数"""
  ```
- **验收标准**:
  - [x] 返回值: -1.0（无效帧）或 0-100（有效分数）
  - [x] 边界 E1.1: 尺寸不一致时自动统一尺寸
  - [x] 边界 E1.2: 全黑帧(mean<5)返回-1.0
  - [x] 边界 E1.3: 全白帧(mean>250)返回-1.0
  - [x] 边界 E1.4: 低方差帧(std<2)返回0.0
  - [x] 边界 E1.5: 帧差>100时返回100.0
- **依赖**: `numpy`, `cv2`
- **测试**: `tests/test_content_detector.py` (T1-T7)

#### T1.2: 实现 `detect_interest_regions(frame1, frame2)`
- **文件**: `optimize_stitching/content_detector.py`
- **函数签名**:
  ```python
  def detect_interest_regions(frame1: np.ndarray, frame2: np.ndarray, 
                              min_area: int = 500) -> List[Tuple[int, int, int, int]]:
      """检测变化密集区域"""
  ```
- **验收标准**:
  - [x] 返回格式: [(x1, y1, x2, y2), ...]
  - [x] 过滤小区域: area < min_area 不返回
  - [x] 边界处理: 同 compute_change_score
- **依赖**: `compute_change_score`, `cv2.findContours`
- **测试**: `tests/test_content_detector.py`

#### T1.3: 单元测试
- **文件**: `tests/test_content_detector.py`
- **验收标准**:
  - [x] 测试用例: T1-T7（7个）
  - [x] 代码覆盖率: >90%
- **依赖**: `pytest`, `content_detector.py`

---

### Phase 2: 自适应采样引擎

#### T2.1: 实现 `AdaptiveSampler` 类
- **文件**: `optimize_stitching/adaptive_sampler.py`
- **类结构**:
  ```python
  class AdaptiveSampler:
      def __init__(self, config: StitcherConfig): ...
      def get_next_interval(self, change_score: float) -> int: ...
  ```
- **验收标准**:
  - [x] 边界 E2.1: change_score=-1 → min_interval
  - [x] 边界 E2.2: change_score=0 → base_interval * 1.5
  - [x] 边界 E2.3: change_score=100 → base_interval * 0.5
  - [x] 边界 E2.4-E2.5: interval 限制在 [min, max]
- **依赖**: `StitcherConfig`
- **测试**: `tests/test_adaptive_sampler.py` (T8-T11)

#### T2.2: 实现早停机制
- **文件**: `optimize_stitching/adaptive_sampler.py`
- **方法**:
  ```python
  def should_early_stop(self, silent_count: int, processed_count: int) -> bool: ...
  ```
- **验收标准**:
  - [x] silent_count >= max_silent_frames(15) 且 processed_count >= min_frames_to_process(10) → True
  - [x] processed_count < min_frames_to_process → False（防止过早终止）
- **测试**: `tests/test_adaptive_sampler.py` (T17-T18)

#### T2.3: 采样轨迹记录
- **文件**: `optimize_stitching/adaptive_sampler.py`
- **数据结构**:
  ```python
  @dataclass
  class SampleRecord:
      frame_index: int
      interval: int
      change_score: float
      decision: str
      timestamp: float
  ```
- **验收标准**:
  - [x] 每个采样帧都有记录
  - [x] 记录可导出为 JSON
- **依赖**: `dataclasses`, `json`

---

### Phase 3: 自然接缝检测

#### T3.1: 实现 `find_seam_line(overlap_region, fallback_to_center)`
- **文件**: `optimize_stitching/seam_detector.py`
- **函数签名**:
  ```python
  def find_seam_line(overlap_region: np.ndarray, 
                     fallback_to_center: bool = True) -> np.ndarray:
      """寻找最小代价接缝线"""
  ```
- **验收标准**:
  - [x] 返回 shape: (h,) 每个y位置的x坐标
  - [x] 边界 E3.1: None/空图像 → ValueError
  - [x] 边界 E3.2: h<3 or w<3 → 中心线
  - [x] 边界 E3.3-E3.4: 低纹理 → 中心线
- **依赖**: `numpy`, `cv2.Sobel`
- **测试**: `tests/test_seam_detector.py` (T12-T14)

#### T3.2: 实现 `blend_with_seam(frame1, frame2, seam_line)`
- **文件**: `optimize_stitching/seam_blender.py`
- **函数签名**:
  ```python
  def blend_with_seam(frame1: np.ndarray, frame2: np.ndarray,
                      seam_line: np.ndarray, overlap_width: int = None) -> np.ndarray:
      """沿接缝线融合两帧"""
  ```
- **验收标准**:
  - [x] 边界 E4.1: None输入 → ValueError
  - [x] 边界 E4.2: 高度不一致 → 裁剪到最小高度
  - [x] 边界 E4.3: seam_line长度不一致 → 调整
  - [x] 边界 E4.4-E4.6: x越界 → 限制范围
  - [x] smoothstep 加权融合
- **依赖**: `numpy`, `cv2.addWeighted`
- **测试**: `tests/test_seam_blender.py` (T15-T16)

#### T3.3: 备选融合方法
- **文件**: `optimize_stitching/seam_blender.py`
- **函数**:
  ```python
  def simple_alpha_blend(frame1: np.ndarray, frame2: np.ndarray, 
                         overlap_width: int) -> np.ndarray:
      """简单 alpha 融合（fallback）"""
  ```
- **验收标准**:
  - [x] 当 seam 检测失败时可用
  - [x] 线性 alpha 过渡
- **依赖**: `cv2.addWeighted`

---

### Phase 4: 图像增强

#### T4.1: 实现 `multiscale_sharpen(image)`
- **文件**: `optimize_stitching/image_enhancer.py`
- **函数签名**:
  ```python
  def multiscale_sharpen(image: np.ndarray, amount: float = 1.5, 
                         radius: float = 2.5) -> np.ndarray:
      """USM 多尺度锐化"""
  ```
- **验收标准**:
  - [x] 空图像 → 返回原图像
  - [x] 全零图像 → 返回原图像
  - [x] 输出尺寸与输入一致
- **依赖**: `cv2.GaussianBlur`, `cv2.addWeighted`
- **测试**: `tests/test_image_enhancer.py`

#### T4.2: 实现 `optimize_contrast(image)`
- **文件**: `optimize_stitching/image_enhancer.py`
- **函数签名**:
  ```python
  def optimize_contrast(image: np.ndarray, clip_limit: float = 3.5,
                        tile_grid_size: Tuple[int, int] = (8, 8)) -> np.ndarray:
      """CLAHE 对比度增强"""
  ```
- **验收标准**:
  - [x] 单通道图像 → 直接处理
  - [x] 多通道图像 → LAB 空间处理
- **依赖**: `cv2.createCLAHE`
- **测试**: `tests/test_image_enhancer.py`

#### T4.3: 集成增强到主流程
- **文件**: `optimize_stitching/image_enhancer.py`
- **函数**:
  ```python
  def enhance(image: np.ndarray, config: StitcherConfig) -> np.ndarray:
      """根据配置执行增强"""
  ```
- **验收标准**:
  - [x] enhance_sharpen=False → 跳过锐化
  - [x] enhance_contrast=False → 跳过对比度优化

---

### Phase 5: 主流程集成

#### T5.1: 实现 `AdaptiveStitcher.stitch(video_path)`
- **文件**: `optimize_stitching/stitcher.py`
- **类结构**:
  ```python
  class AdaptiveStitcher:
      def __init__(self, config: StitcherConfig): ...
      def stitch(self, video_path: str) -> StitchResult: ...
  ```
- **流程**:
  ```
  VideoReader → AdaptiveSampler → ContentDetector → 
  SeamDetector → SeamBlender → ImageEnhancer → OutputWriter
  ```
- **验收标准**:
  - [x] 完整视频处理无报错
  - [x] 进度显示（每处理25帧输出一次）
  - [x] 错误处理（损坏帧跳过）
- **依赖**: 所有模块

#### T5.2: 实现 `StitcherConfig` 验证
- **文件**: `optimize_stitching/config.py`
- **类**:
  ```python
  @dataclass
  class StitcherConfig:
      base_interval: int = 60
      # ... 其他参数
      
      def validate(self) -> None:
          """参数范围验证"""
  ```
- **验收标准**:
  - [x] base_interval 范围 [1, 500]
  - [x] high_threshold > low_threshold
  - [x] output_width 范围 [100, 2000]
  - [x] 环境变量读取（OPENSTITCH_*）
- **依赖**: `dataclasses`, `os`

#### T5.3: 实现分段输出
- **文件**: `optimize_stitching/output_writer.py`
- **函数**:
  ```python
  def write_segments(images: List[np.ndarray], output_dir: str,
                     prefix: str, quality: int = 97) -> List[str]:
      """分段保存图像"""
  ```
- **验收标准**:
  - [x] 单段高度 <= segment_height(25000)
  - [x] 自动创建输出目录
  - [x] 返回保存的文件路径列表
- **依赖**: `cv2.imwrite`, `os.makedirs`

---

### Phase 6: 异常处理

#### T6.1: 定义异常层次
- **文件**: `optimize_stitching/exceptions.py`
- **异常结构**:
  ```python
  class StitcherError(Exception): pass
  class VideoReadError(StitcherError): pass
  class InvalidFrameError(StitcherError): pass
  class SeamDetectionError(StitcherError): pass
  class BlendError(StitcherError): pass
  class OutputWriteError(StitcherError): pass
  ```
- **验收标准**:
  - [x] 5种异常类型完整定义
  - [x] 继承关系正确

#### T6.2: 实现日志记录
- **文件**: `optimize_stitching/logger.py`
- **函数**:
  ```python
  def get_logger(name: str = __name__) -> logging.Logger: ...
  def log_error(context: str, error: Exception, recoverable: bool = True): ...
  def log_warning(context: str, message: str): ...
  ```
- **验收标准**:
  - [x] 结构化日志输出
  - [x] 包含时间戳、模块名、错误类型

#### T6.3: 降级策略
- **文件**: `optimize_stitching/stitcher.py` (集成)
- **策略**:
  - 内存 > 1.5GB → 减少 output_width 20%
  - 连续3次接缝失败 → 禁用自然接缝
  - 每帧处理 > 2s → 跳过锐化
- **验收标准**:
  - [x] 内存监控和降级
  - [x] 功能降级后继续运行

---

### Phase 7: 验证和优化

#### T7.1: 端到端测试
- **文件**: `tests/test_integration.py`
- **测试**:
  ```python
  def test_small_video(): ...
  def test_corrupt_frame_recovery(): ...  # T19
  def test_output_dir_not_exists(): ...   # T20
  ```
- **验收标准**:
  - [x] 100帧视频完整处理
  - [x] 损坏帧跳过继续
  - [x] 目录不存在时自动创建

#### T7.2: 性能基准测试
- **文件**: `tests/test_integration.py`
- **测试**:
  ```python
  def test_performance(): ...
  ```
- **验收标准**:
  - [x] 处理时间 < 30秒
  - [x] 内存 < 2GB

#### T7.3: 质量评估
- **手动评估**:
  - 零件连续性 > 80%
  - 接缝可见度 < 0.1
  - 放大200%细节可辨
- **自动化评估**:
  ```python
  def assess_seam_visibility(image: np.ndarray) -> float: ...
  def assess_continuity(image1: np.ndarray, image2: np.ndarray) -> float: ...
  ```

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
| `stitcher.py` | 所有模块 | main.py |
| `main.py` | stitcher, config | - |

---

## 4. 实施顺序

### 第1批（基础依赖）
1. `config.py` (T5.2)
2. `exceptions.py` (T6.1)
3. `logger.py` (T6.2)

### 第2批（核心算法）
4. `content_detector.py` (T1.1, T1.2)
5. `adaptive_sampler.py` (T2.1, T2.2, T2.3)
6. `seam_detector.py` (T3.1)
7. `seam_blender.py` (T3.2, T3.3)

### 第3批（增强与输入输出）
8. `image_enhancer.py` (T4.1, T4.2, T4.3)
9. `video_reader.py`
10. `output_writer.py` (T5.3)

### 第4批（集成）
11. `stitcher.py` (T5.1)
12. `main.py` (CLI)

### 第5批（测试）
13. `tests/test_content_detector.py` (T1.3)
14. `tests/test_adaptive_sampler.py`
15. `tests/test_seam_detector.py`
16. `tests/test_seam_blender.py`
17. `tests/test_image_enhancer.py`
18. `tests/test_integration.py` (T7.1, T7.2)

---

## 5. 验收检查点详情

### CP1: Phase 1 完成
- [ ] `content_detector.py` 存在
- [ ] `compute_change_score` 通过 T1-T7
- [ ] 代码覆盖率 > 90%

### CP2: Phase 2 完成
- [ ] `adaptive_sampler.py` 存在
- [ ] `AdaptiveSampler.get_next_interval` 通过 T8-T11
- [ ] `should_early_stop` 通过 T17-T18

### CP3: Phase 3 完成
- [ ] `seam_detector.py` 存在
- [ ] `find_seam_line` 通过 T12-T14
- [ ] `seam_blender.py` 存在
- [ ] `blend_with_seam` 通过 T15-T16

### CP4: Phase 4 完成
- [ ] `image_enhancer.py` 存在
- [ ] `multiscale_sharpen` 和 `optimize_contrast` 正常工作

### CP5: Phase 5 完成
- [ ] `stitcher.py` 存在
- [ ] `AdaptiveStitcher.stitch` 端到端无报错
- [ ] `main.py` CLI 正常工作

### CP6: Phase 6 完成
- [ ] `exceptions.py` 5种异常定义
- [ ] `logger.py` 结构化日志
- [ ] 降级策略集成到 stitcher

### CP7: 最终验收
- [ ] 性能: 处理时间 < 30秒, 内存 < 2GB
- [ ] 质量: 零件连续性 > 80%, 接缝可见度 < 0.1
- [ ] 稳定性: 所有20个测试用例通过

---

## 6. 命令行接口

```bash
# 完整命令
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

# 简化命令（使用默认参数）
python -m optimize_stitching.main \
  --input enhanced_videos/enhanced_brightness_contrast_fast.avi
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

# 保存结果
for i, img in enumerate(result.images):
    cv2.imwrite(f'output_{i}.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 97])

# 查看统计
print(f"处理帧数: {result.stats.frames_used}")
print(f"总耗时: {result.stats.total_time_s:.1f}s")
```
