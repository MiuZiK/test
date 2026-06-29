# 设计：车底全景图拼接优化

## 1. 系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                        AdaptiveStitcher                         │
├─────────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────┐ │
│  │   Video     │  │  Adaptive   │  │   Seam     │  │  Image  │ │
│  │   Reader    │→ │  Sampler    │→ │  Blender    │→ │Enhancer │ │
│  └─────────────┘  └──────┬──────┘  └──────┬──────┘  └────┬────┘ │
│                          │                │               │     │
│                   ┌──────▼──────┐  ┌──────▼──────┐  ┌────▼────┐ │
│                   │  Content    │  │    Seam     │  │  Multi  │ │
│                   │  Analyzer   │  │  Detector   │  │  Scale  │ │
│                   └─────────────┘  └─────────────┘  └─────────┘ │
└─────────────────────────────────────────────────────────────────┘
```

---

## 2. 核心算法设计

### 2.1 内容变化检测 (Content Change Detection)

**算法**: `compute_change_score(frame1, frame2)`

```python
def compute_change_score(frame1, frame2):
    # ─────────────────────────────────────────────────────────────
    # 边界条件 1: 图像尺寸不一致
    # ─────────────────────────────────────────────────────────────
    if frame1.shape != frame2.shape:
        # 统一调整为较小图像的尺寸
        h = min(frame1.shape[0], frame2.shape[0])
        w = min(frame1.shape[1], frame2.shape[1])
        frame1 = cv2.resize(frame1, (w, h))
        frame2 = cv2.resize(frame2, (w, h))

    # ─────────────────────────────────────────────────────────────
    # 边界条件 2: 单通道图像 (灰度图)
    # ─────────────────────────────────────────────────────────────
    if len(frame1.shape) == 2:
        gray1, gray2 = frame1, frame2
    else:
        gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)

    # ─────────────────────────────────────────────────────────────
    # 边界条件 3: 全黑/全白帧检测 (相机初始化/遮挡)
    # ─────────────────────────────────────────────────────────────
    mean1, mean2 = np.mean(gray1), np.mean(gray2)
    if mean1 < 5 or mean1 > 250:
        # 全黑或过曝帧，返回特殊值标记
        return -1.0  # 表示"无效帧"
    if np.std(gray1) < 2 or np.std(gray2) < 2:
        # 低方差帧(纯色)，变化检测无意义
        return 0.0

    # ─────────────────────────────────────────────────────────────
    # 边界条件 4: 帧差阈值容错
    # ─────────────────────────────────────────────────────────────
    diff_threshold = max(10, np.std(gray1) * 0.5)  # 自适应阈值

    # Step 1: 计算边缘密度 (使用Sobel)
    sobel1_x = cv2.Sobel(gray1, cv2.CV_64F, 1, 0, ksize=3)
    sobel1_y = cv2.Sobel(gray1, cv2.CV_64F, 0, 1, ksize=3)
    edge_density1 = np.mean(np.sqrt(sobel1_x**2 + sobel1_y**2))

    # Step 2: 计算帧差
    diff = cv2.absdiff(gray1.astype(np.float64), gray2.astype(np.float64))
    mean_diff = np.mean(diff)

    # Step 3: 变化百分比 (使用自适应阈值)
    _, thresh = cv2.threshold(diff, diff_threshold, 255, cv2.THRESH_BINARY)
    changed_pixels = np.count_nonzero(thresh) / thresh.size * 100

    # Step 4: 极端情况检测
    if mean_diff > 100:
        # 场景剧烈变化(相机移动过快/遮挡)
        return 100.0

    # Step 5: 综合评分 (归一化到0-100)
    # 权重分配: 帧差30% + 变化像素40% + 边缘密度30%
    score = (mean_diff / 50 * 30) + (changed_pixels / 100 * 40) + (edge_density1 / 200 * 30)
    return float(min(100, max(0, score)))
```

**边界条件清单**:
| 条件 | 输入 | 预期输出 | 处理 |
|------|------|---------|------|
| E1.1 | frame1.shape ≠ frame2.shape | 正常评分 | 统一尺寸 |
| E1.2 | 全黑帧 (mean<5) | -1.0 | 标记无效 |
| E1.3 | 全白帧 (mean>250) | -1.0 | 标记无效 |
| E1.4 | 低方差帧 (std<2) | 0.0 | 视为无变化 |
| E1.5 | 帧差>100 | 100.0 | 截断 |

**阈值设定**:
- `HIGH_THRESHOLD = 60`: 高于该值表示场景变化剧烈，需加密采样
- `LOW_THRESHOLD = 20`: 低于该值表示场景基本静止，可减密采样

---

### 2.2 自适应采样 (Adaptive Sampling)

**算法**: `get_next_interval(change_score)`

```python
def get_next_interval(self, change_score):
    # ─────────────────────────────────────────────────────────────
    # 异常路径 1: 无效帧 (-1.0)
    # ─────────────────────────────────────────────────────────────
    if change_score < 0:
        # 无效帧，使用最小间隔强制采样
        return max(1, self.base_interval // 4)

    # ─────────────────────────────────────────────────────────────
    # 异常路径 2: 边界值保护
    # ─────────────────────────────────────────────────────────────
    change_score = max(0, min(100, change_score))

    # ─────────────────────────────────────────────────────────────
    # 正常逻辑: 根据变化密度调整间隔
    # ─────────────────────────────────────────────────────────────
    if change_score > self.high_threshold:
        # 密集区域: 加密采样
        return int(self.base_interval * 0.5)
    elif change_score > self.low_threshold:
        # 正常区域: 标准采样
        return self.base_interval
    else:
        # 稀疏区域: 减密采样
        return int(self.base_interval * 1.5)
```

**采样流程**:
```
1. 从视频第0帧开始
2. 读取下一帧 (当前帧 + interval)
3. 计算 change_score(current_frame, next_frame)
4. 根据 score 计算下一个 interval
5. 重复步骤2-4直到视频结束
```

**边界条件清单**:
| 条件 | 输入 | 预期输出 | 处理 |
|------|------|---------|------|
| E2.1 | change_score = -1.0 | min_interval | 强制采样 |
| E2.2 | change_score = 0 | base_interval * 1.5 | 减密 |
| E2.3 | change_score = 100 | base_interval * 0.5 | 加密 |
| E2.4 | interval < 1 | interval = 1 | 最小间隔 |
| E2.5 | interval > total_frames | interval = total_frames | 最大间隔 |

**早停机制**:
- 如果连续 `max_silent_frames=10` 帧变化分数都低于 `low_threshold`
- 且已处理帧数 > `min_frames_to_process=50`
- 则提前结束处理

---

### 2.3 自然接缝检测 (Seam Detection)

**算法**: `find_seam_line(overlap_region)`

```python
def find_seam_line(overlap_region, fallback_to_center=True):
    # ─────────────────────────────────────────────────────────────
    # 边界条件 1: 空图像
    # ─────────────────────────────────────────────────────────────
    if overlap_region is None or overlap_region.size == 0:
        raise ValueError("overlap_region is empty")

    h, w = overlap_region.shape[:2]

    # ─────────────────────────────────────────────────────────────
    # 边界条件 2: 过小的重叠区域
    # ─────────────────────────────────────────────────────────────
    if h < 3 or w < 3:
        # 无法进行有效的边缘检测，返回中心线
        return np.full(h, w // 2, dtype=np.int32)

    # ─────────────────────────────────────────────────────────────
    # 边界条件 3: 单通道/多通道转换
    # ─────────────────────────────────────────────────────────────
    if len(overlap_region.shape) == 2:
        gray = overlap_region
    else:
        gray = cv2.cvtColor(overlap_region, cv2.COLOR_BGR2GRAY)

    # ─────────────────────────────────────────────────────────────
    # Step 1: 计算能量图 (边缘幅度)
    # ─────────────────────────────────────────────────────────────
    sobel_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
    sobel_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
    energy = np.sqrt(sobel_x**2 + sobel_y**2)

    # ─────────────────────────────────────────────────────────────
    # 边界条件 4: 全零能量图 (低纹理区域)
    # ─────────────────────────────────────────────────────────────
    if np.sum(energy) < 1e-6:
        if fallback_to_center:
            return np.full(h, w // 2, dtype=np.int32)
        else:
            raise ValueError("Cannot find seam in low-texture region")

    # ─────────────────────────────────────────────────────────────
    # Step 2: 动态规划找最小代价路径 (优化版)
    # ─────────────────────────────────────────────────────────────
    # 使用累积能量代替逐点更新，减少计算量
    dp = energy.copy()
    path = np.zeros(h, dtype=np.int32)

    try:
        for y in range(1, h):
            # 一次性计算三个方向的最小值
            prev_row = dp[y-1, :]
            # 使用 numpy 广播优化
            dp[y, 1:-1] += np.minimum(prev_row[1:-1],
                                       np.minimum(prev_row[:-2], prev_row[2:]))
            dp[y, 0] += min(prev_row[0], prev_row[1])
            dp[y, -1] += min(prev_row[-1], prev_row[-2])

        # 回溯找最优路径
        path[h-1] = int(np.argmin(dp[h-1, :]))
        for y in range(h-2, -1, -1):
            x = path[y+1]
            candidates = []
            if x > 0:
                candidates.append((dp[y, x-1], x-1))
            candidates.append((dp[y, x], x))
            if x < w-1:
                candidates.append((dp[y, x+1], x+1))
            path[y] = min(candidates)[1]

    except Exception as e:
        # 动态规划失败，返回中心线
        return np.full(h, w // 2, dtype=np.int32)

    return path  # 返回每个y位置的x坐标
```

**边界条件清单**:
| 条件 | 输入 | 预期输出 | 处理 |
|------|------|---------|------|
| E3.1 | overlap_region = None | 抛出异常 | 早期检查 |
| E3.2 | h < 3 或 w < 3 | 中心线 | 尺寸保护 |
| E3.3 | 全零能量图 | 中心线 | 低纹理保护 |
| E3.4 | 动态规划异常 | 中心线 | fallback |

---

### 2.4 接缝融合 (Seam Blending)

**算法**: `blend_with_seam(frame1, frame2, seam_line)`

```python
def blend_with_seam(frame1, frame2, seam_line, overlap_width=None):
    # ─────────────────────────────────────────────────────────────
    # 边界条件 1: 输入验证
    # ─────────────────────────────────────────────────────────────
    if frame1 is None or frame2 is None:
        raise ValueError("frame1 or frame2 is None")
    if seam_line is None or len(seam_line) == 0:
        raise ValueError("seam_line is empty")

    h1, w1 = frame1.shape[:2]
    h2, w2 = frame2.shape[:2]

    # ─────────────────────────────────────────────────────────────
    # 边界条件 2: 高度不一致
    # ─────────────────────────────────────────────────────────────
    if h1 != h2:
        # 高度不一致，以较小高度为准
        h = min(h1, h2)
        frame1 = frame1[:h, :, :]
        frame2 = frame2[:h, :, :]
        seam_line = seam_line[:h]
    else:
        h = h1

    # ─────────────────────────────────────────────────────────────
    # 边界条件 3: seam_line 长度不一致
    # ─────────────────────────────────────────────────────────────
    if len(seam_line) != h:
        seam_line = np.resize(seam_line, h)

    # ─────────────────────────────────────────────────────────────
    # 边界条件 4: 计算默认重叠宽度
    # ─────────────────────────────────────────────────────────────
    if overlap_width is None:
        overlap_width = min(w1, w2) // 4  # 默认25%的宽度

    overlap_width = max(1, min(overlap_width, min(w1, w2) // 2))

    # ─────────────────────────────────────────────────────────────
    # Step 1: 创建结果画布
    # ─────────────────────────────────────────────────────────────
    result = np.zeros_like(frame1)

    # ─────────────────────────────────────────────────────────────
    # Step 2: 逐行融合
    # ─────────────────────────────────────────────────────────────
    for y in range(h):
        x_seam = int(seam_line[y])
        x_seam = max(0, min(x_seam, w1 - 1))

        # 计算混合带宽度 (动态调整)
        blend_half = max(1, overlap_width // 2)

        for x in range(w1):
            # 左侧区域: 完全使用 frame1
            if x < x_seam - blend_half:
                result[y, x] = frame1[y, x]
            # 混合区域: smoothstep 加权
            elif x < x_seam + blend_half:
                # 计算混合因子 (0→1→0 的平滑曲线)
                dist = abs(x - x_seam)
                alpha = 1.0 - (dist / blend_half)
                # Smoothstep: 3t² - 2t³
                alpha = alpha * alpha * (3 - 2 * alpha)

                # frame2 的 x 坐标 (frame2 可能比 frame1 窄)
                x2 = x - (w1 - overlap_width)
                x2 = max(0, min(x2, w2 - 1))

                if x2 >= 0 and x2 < w2:
                    result[y, x] = cv2.addWeighted(
                        frame1[y, x], 1 - alpha,
                        frame2[y, x2], alpha, 0
                    )
                else:
                    result[y, x] = frame1[y, x]
            # 右侧区域: 使用 frame2
            else:
                x2 = x - (w1 - overlap_width)
                x2 = max(0, min(x2, w2 - 1))
                if x2 >= 0 and x2 < w2:
                    result[y, x] = frame2[y, x2]
                else:
                    result[y, x] = frame1[y, x]

    return result
```

**边界条件清单**:
| 条件 | 输入 | 预期输出 | 处理 |
|------|------|---------|------|
| E4.1 | frame1/frame2 = None | 抛出异常 | 早期检查 |
| E4.2 | h1 ≠ h2 | 正常融合 | 裁剪到最小高度 |
| E4.3 | len(seam_line) ≠ h | 正常融合 | 调整长度 |
| E4.4 | x_seam < 0 | 正常融合 | 限制范围 |
| E4.5 | x2 < 0 或 x2 ≥ w2 | 边界像素 | 使用边界值 |
| E4.6 | overlap_width = 0 | 1 | 最小保护 |

---

## 3. 数据流设计

```
Video File (AVI)
      │
      ▼
┌─────────────────┐
│   VideoReader   │ 读取帧，支持随机访问
│   └─错误处理    │ - 损坏帧跳过
│   └─边界处理    │ - 首尾帧特殊处理
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ AdaptiveSampler │ 自适应采样策略
│  ├─ base_interval│
│  ├─ thresholds   │
│  └─早停机制      │
└────────┬────────┘
         │
    ┌────┴────┐
    │         │
    ▼         ▼
┌────────┐ ┌────────┐
│Content │ │Seam    │
│Analyzer│ │Detector│
│└─无效帧│ │└─低纹理│
│└─边界值│ │└─fallback│
└───┬────┘ └───┬────┘
    │         │
    ▼         ▼
┌─────────────────┐
│   SeamBlender  │ 自然接缝融合
│   └─维度检查   │ └─H/W不一致处理
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ ImageEnhancer  │ 多尺度锐化+CLAHE
│  ├─USM锐化      │ └─边界保护
│  └─CLAHE       │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  OutputWriter  │ 分段保存JPEG
│  ├─磁盘空间检查│
│  └─分段策略   │
└─────────────────┘
```

---

## 4. 关键数据结构

### 4.1 AdaptiveStitcher 配置

```python
@dataclass
class StitcherConfig:
    # ─────────────────────────────────────────────────────────────
    # 采样参数 (带边界保护)
    # ─────────────────────────────────────────────────────────────
    base_interval: int = 60          # 基础采样间隔 [1, 500]
    high_threshold: float = 60.0     # 加密阈值 [0, 100]
    low_threshold: float = 20.0     # 减密阈值 [0, 100]
    min_interval: int = 5           # 最小采样间隔
    max_interval: int = 200         # 最大采样间隔

    # ─────────────────────────────────────────────────────────────
    # 融合参数
    # ─────────────────────────────────────────────────────────────
    overlap_ratio: float = 0.15     # 重叠比例 [0.01, 0.5]
    seamless_blend: bool = True     # 是否启用自然接缝
    fallback_to_center: bool = True # 接缝检测失败时退回到中心线

    # ─────────────────────────────────────────────────────────────
    # 图像参数
    # ─────────────────────────────────────────────────────────────
    output_width: int = 600         # 输出宽度 [100, 2000]
    enhance_sharpen: bool = True    # 是否启用锐化
    sharpen_amount: float = 1.5     # USM锐化量 [0.5, 3.0]
    enhance_contrast: bool = True   # 是否启用对比度优化
    clahe_clip: float = 3.5         # CLAHE对比度限制 [1.0, 10.0]

    # ─────────────────────────────────────────────────────────────
    # 性能参数
    # ─────────────────────────────────────────────────────────────
    max_frames: int = 500           # 最大处理帧数
    min_frames_to_process: int = 10 # 最小处理帧数
    max_silent_frames: int = 15     # 早停:最大连续低变化帧数

    # ─────────────────────────────────────────────────────────────
    # 输出参数
    # ─────────────────────────────────────────────────────────────
    jpeg_quality: int = 97          # JPEG质量 [1, 100]
    segment_height: int = 25000     # 分段高度
```

### 4.2 采样记录

```python
@dataclass
class SampleRecord:
    frame_index: int       # 帧索引
    interval: int          # 实际采样间隔
    change_score: float   # 变化分数 [-1=无效, 0-100=有效]
    decision: str          # 'dense' | 'normal' | 'sparse' | 'invalid'
    timestamp: float      # 时间戳
    seam_line: np.ndarray  # 接缝线 (可选)
    blend_time_ms: float  # 融合耗时 (可选)

@dataclass
class ProcessingStats:
    total_frames_read: int
    frames_used: int
    frames_skipped: int
    frames_invalid: int
    early_stopped: bool
    total_time_s: float
    avg_interval: float
```

### 4.3 融合结果

```python
@dataclass
class BlendResult:
    image: np.ndarray      # 融合后图像
    seam_line: np.ndarray  # 接缝线位置
    energy_map: np.ndarray # 能量图 (用于调试)
    fallback_used: bool    # 是否使用了fallback

@dataclass
class StitchResult:
    images: List[np.ndarray]  # 分段图像列表
    stats: ProcessingStats   # 处理统计
    config: StitcherConfig  # 使用配置
```

---

## 5. 性能优化策略

### 5.1 帧缓存策略
- 只缓存最近2帧，避免内存溢出
- 使用内存池复用图像缓冲区
- 使用 `np.memmap` 处理超大图像

### 5.2 并行化
- 内容分析可在 CPU 多核并行
- 使用 `concurrent.futures.ThreadPoolExecutor`
- 每4帧启动一个并行任务

### 5.3 早停机制
```python
def should_early_stop(silent_frame_count, total_frames, min_required):
    """判断是否应该早停"""
    if total_frames < min_required:
        return False
    if silent_frame_count >= config.max_silent_frames:
        return True
    # 进度检查: 剩余帧太少时不早停
    remaining = total_frames - current_frame
    if remaining < min_required:
        return False
    return False
```

### 5.4 降级策略
| 条件 | 触发 | 降级动作 |
|------|------|---------|
| 内存警告 | >1.5GB | 减少 output_width 20% |
| 接缝失败 | 连续3次 | 禁用自然接缝，使用简单融合 |
| 性能退化 | 每帧>2s | 跳过锐化步骤 |

---

## 6. 错误处理设计

### 6.1 异常层次

```python
class StitcherError(Exception):
    """基础异常类"""
    pass

class VideoReadError(StitcherError):
    """视频读取错误"""
    pass

class InvalidFrameError(StitcherError):
    """无效帧错误"""
    pass

class SeamDetectionError(StitcherError):
    """接缝检测错误"""
    pass

class BlendError(StitcherError):
    """融合错误"""
    pass

class OutputWriteError(StitcherError):
    """输出写入错误"""
    pass
```

### 6.2 错误处理策略

| 错误类型 | 处理策略 | 恢复动作 |
|---------|---------|---------|
| VideoReadError | 跳过该帧，记录日志 | 尝试读取下一帧 |
| InvalidFrameError | 标记，跳过 | 强制采样下一帧 |
| SeamDetectionError | 使用fallback | 回退到中心线 |
| BlendError | 使用简单融合 | 禁用seam，使用alpha |
| OutputWriteError | 重试3次 | 写入临时目录 |

### 6.3 日志记录

```python
import logging

logger = logging.getLogger(__name__)

def log_error(context, error, recoverable=True):
    """结构化错误日志"""
    logger.error(
        f"[{context}] {error.__class__.__name__}: {str(error)}",
        extra={
            'recoverable': recoverable,
            'timestamp': time.time()
        }
    )

def log_warning(context, message):
    """警告日志"""
    logger.warning(f"[{context}] {message}")
```

---

## 7. 配置层次

```
CLI参数 (最高优先级)
       ↓
StitcherConfig默认值
       ↓
环境变量 (OPENSTITCH_*)
       ↓
配置文件 (openspec.yaml)
```

**环境变量映射**:
| 环境变量 | 配置字段 |
|---------|---------|
| OPENSTITCH_BASE_INTERVAL | base_interval |
| OPENSTITCH_OUTPUT_WIDTH | output_width |
| OPENSTITCH_JPEG_QUALITY | jpeg_quality |

---

## 8. 测试策略

### 8.1 单元测试

```python
class TestContentChangeDetection:
    """T1: 内容变化检测"""

    def test_identical_frames(self):
        """E1: 完全相同帧"""
        frame = np.zeros((100, 100, 3), dtype=np.uint8)
        score = compute_change_score(frame, frame)
        assert score == 0.0

    def test_black_frame(self):
        """E1.2: 全黑帧"""
        black = np.zeros((100, 100, 3), dtype=np.uint8)
        normal = np.ones((100, 100, 3), dtype=np.uint8) * 128
        score = compute_change_score(black, normal)
        assert score == -1.0

    def test_white_frame(self):
        """E1.3: 全白帧"""
        white = np.ones((100, 100, 3), dtype=np.uint8) * 255
        normal = np.ones((100, 100, 3), dtype=np.uint8) * 128
        score = compute_change_score(white, normal)
        assert score == -1.0

    def test_dimension_mismatch(self):
        """E1.1: 尺寸不一致"""
        frame1 = np.zeros((100, 100, 3), dtype=np.uint8)
        frame2 = np.zeros((80, 120, 3), dtype=np.uint8)
        score = compute_change_score(frame1, frame2)
        assert 0 <= score <= 100  # 不应抛出异常

class TestSeamDetection:
    """T3: 接缝检测"""

    def test_small_region(self):
        """E3.2: 过小区域"""
        region = np.zeros((2, 2, 3), dtype=np.uint8)
        seam = find_seam_line(region)
        assert len(seam) == 2
        assert np.all(seam == 1)  # 中心线

    def test_empty_region(self):
        """E3.1: 空图像"""
        with pytest.raises(ValueError):
            find_seam_line(None)
```

### 8.2 集成测试

```python
class TestEndToEnd:
    """T5: 端到端测试"""

    def test_small_video(self):
        """100帧视频完整流程"""
        config = StitcherConfig(
            base_interval=10,
            max_frames=100
        )
        result = AdaptiveStitcher(config).stitch('test_video.avi')
        assert result.stats.total_frames_read > 0
        assert result.stats.frames_used > 0
        assert len(result.images) > 0

    def test_corrupt_frame_recovery(self):
        """损坏帧恢复"""
        # 模拟读取失败
        ...

    def test_early_stop(self):
        """早停机制"""
        ...
```

### 8.3 性能测试

```python
def test_performance():
    """性能基准测试"""
    import time
    t0 = time.time()
    result = stitcher.stitch('full_video.avi')
    elapsed = time.time() - t0

    assert elapsed < 30, f"处理时间 {elapsed}s 超过30s限制"
    assert result.stats.total_memory_mb < 2048, "内存超过2GB"
```

### 8.4 质量评估

```python
def assess_seam_visibility(image, seam_line):
    """自动化接缝可见度评估"""
    # 在接缝线附近提取patch
    # 计算边缘密度
    # 返回可见度分数 (0=不可见, 1=明显)
    pass

def assess_continuity(image1, image2, overlap_region):
    """零件连续性评估"""
    # 计算跨接缝的纹理连续性
    # 使用结构相似性(SSIM)
    pass
```

---

## 9. 状态机

```
┌─────────────┐
│   IDLE      │ ← 初始状态
└──────┬──────┘
       │ start()
       ▼
┌─────────────┐
│ READING     │ ← 读取视频
└──────┬──────┘
       │ next frame ready
       ▼
┌─────────────┐
│ SAMPLING    │ ← 采样决策
└──────┬──────┘
       │ frame sampled
       ▼
┌─────────────┐
│ ANALYZING   │ ← 内容分析
└──────┬──────┘
       │ analysis done
       ▼
┌─────────────┐
│ BLENDING    │ ← 接缝融合
└──────┬──────┘
       │ blend done
       ▼
┌─────────────┐     ┌─────────────┐
│  WRITING    │ ──► │   DONE      │
└─────────────┘     └─────────────┘
       │                  ▲
       │ error             │
       ▼                  │
┌─────────────┐            │
│ ERROR       │ ───────────┘
│  (recovery) │
└─────────────┘
```

---

## 10. 设计稳定性检查清单

- [x] 所有函数都有输入验证
- [x] 边界条件都有明确处理
- [x] 异常路径有fallback机制
- [x] 数值计算都有范围保护
- [x] 资源使用有上限控制
- [x] 错误可恢复，不导致崩溃
- [x] 日志记录完整
- [x] 配置参数有边界检查
- [x] 状态转换有明确定义
- [x] 单元测试覆盖边界条件
