# API 接口参考

## optimize_stitching 模块

### 1. AdaptiveStitcher 类

**导入**:
```python
from optimize_stitching import AdaptiveStitcher
```

**方法**:

#### `__init__(self, config: StitcherConfig)`

初始化拼接器。

**参数**:
- `config`: StitcherConfig 配置对象

#### `stitch(self, video_path: str) -> StitchResult`

执行图像拼接。

**参数**:
- `video_path`: 视频文件路径

**返回**:
- `StitchResult` 对象，包含：
  - `images`: 拼接后的图像列表
  - `stats`: 统计信息

---

### 2. StitcherConfig 类

**导入**:
```python
from optimize_stitching import StitcherConfig
```

**构造函数**:
```python
StitcherConfig(
    base_interval=60,
    high_threshold=60.0,
    low_threshold=20.0,
    output_width=600,
    overlap_ratio=0.15,
    seamless_blend=True,
    max_frames=500,
    jpeg_quality=97,
    enhance_sharpen=True,
    enhance_contrast=True
)
```

**参数说明**:

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| base_interval | int | 60 | 基础采样帧间隔 |
| high_threshold | float | 60.0 | 加密采样阈值 |
| low_threshold | float | 20.0 | 减密采样阈值 |
| output_width | int | 600 | 输出图像宽度 |
| overlap_ratio | float | 0.15 | 重叠比例 |
| seamless_blend | bool | True | 是否启用自然接缝融合 |
| max_frames | int | 500 | 最大处理帧数 |
| jpeg_quality | int | 97 | JPEG输出质量 |
| enhance_sharpen | bool | True | 是否启用锐化增强 |
| enhance_contrast | bool | True | 是否启用对比度增强 |

---

### 3. 异常类型

**导入**:
```python
from optimize_stitching import (
    StitcherError,
    BlackFrameError,
    WhiteFrameError,
    BlendError,
    VideoError
)
```

**异常层次**:
```
StitcherError (基类)
├── BlackFrameError   # 黑帧错误
├── WhiteFrameError   # 白帧错误
├── BlendError        # 融合错误
└── VideoError        # 视频读取错误
```

---

### 4. CLI 接口

```bash
python -m optimize_stitching.main --help
```

**命令行参数**:

| 参数 | 说明 | 默认值 |
|------|------|--------|
| --input | 输入视频文件路径 | (必填) |
| --output | 输出目录路径 | output/ |
| --base_interval | 基础采样帧间隔 | 60 |
| --high_threshold | 加密采样阈值 | 60.0 |
| --low_threshold | 减密采样阈值 | 20.0 |
| --output_width | 输出图像宽度 | 600 |
| --overlap_ratio | 重叠比例 | 0.15 |
| --max_frames | 最大处理帧数 | 500 |
| --jpeg_quality | JPEG输出质量 | 97 |
| --no_seam | 禁用自然接缝融合 | False |
| --no_sharpen | 禁用锐化增强 | False |
| --no_contrast | 禁用对比度增强 | False |

---

## ROS 节点接口

### SBUS 消息格式

**消息类型**: `sbus_serial/Sbus`

**字段**:
```
Header header
int16[16] channels
bool failsafe
bool lost_frame
```

### 服务接口

#### sbus_cmd_vel

将SBUS信号转换为Twist消息。

**输入**: SBUS消息

**输出**: geometry_msgs/Twist

---

## 错误处理

### 图像拼接错误码

| 错误码 | 错误类型 | 说明 |
|--------|---------|------|
| 1001 | BlackFrameError | 检测到黑帧，跳过 |
| 1002 | WhiteFrameError | 检测到白帧，跳过 |
| 1003 | BlendError | 融合失败，使用备选方案 |
| 2001 | VideoError | 视频读取失败 |

### 降级策略

当检测到资源限制时，系统会自动降级：

1. **内存超限**: 降低图像分辨率
2. **性能超限**: 减少处理帧数
3. **接缝失败**: 切换到简单融合模式
