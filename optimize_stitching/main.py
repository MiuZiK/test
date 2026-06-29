import argparse
import os
import sys
from .config import StitcherConfig
from .stitcher import AdaptiveStitcher
from .output_writer import write_segments
from .logger import log_info, log_error


def main():
    parser = argparse.ArgumentParser(description='车底全景图拼接优化')

    parser.add_argument('--input', required=True, help='输入视频路径')
    parser.add_argument('--output', default='output', help='输出目录')
    parser.add_argument('--base_interval', type=int, default=60, help='基础采样帧间隔')
    parser.add_argument('--high_threshold', type=float, default=60.0, help='加密采样阈值')
    parser.add_argument('--low_threshold', type=float, default=20.0, help='减密采样阈值')
    parser.add_argument('--output_width', type=int, default=600, help='输出图像宽度')
    parser.add_argument('--overlap_ratio', type=float, default=0.15, help='重叠比例')
    parser.add_argument('--max_frames', type=int, default=500, help='最大处理帧数')
    parser.add_argument('--jpeg_quality', type=int, default=97, help='JPEG质量')
    parser.add_argument('--no_seam', action='store_true', help='禁用自然接缝')
    parser.add_argument('--no_sharpen', action='store_true', help='禁用锐化')
    parser.add_argument('--no_contrast', action='store_true', help='禁用对比度增强')

    args = parser.parse_args()

    if not os.path.exists(args.input):
        log_error("main", f"输入文件不存在: {args.input}")
        sys.exit(1)

    config = StitcherConfig(
        base_interval=args.base_interval,
        high_threshold=args.high_threshold,
        low_threshold=args.low_threshold,
        output_width=args.output_width,
        overlap_ratio=args.overlap_ratio,
        max_frames=args.max_frames,
        jpeg_quality=args.jpeg_quality,
        seamless_blend=not args.no_seam,
        enhance_sharpen=not args.no_sharpen,
        enhance_contrast=not args.no_contrast
    )

    log_info("main", f"配置: base_interval={config.base_interval}, output_width={config.output_width}")

    stitcher = AdaptiveStitcher(config)
    result = stitcher.stitch(args.input)

    if result.images:
        os.makedirs(args.output, exist_ok=True)

        files = write_segments(result.images, args.output, prefix='stitch', quality=config.jpeg_quality)

        log_info("main", f"输出 {len(files)} 个分段到 {args.output}")
        for f in files:
            print(f"  {f}")

        log_info("main", f"统计: 使用 {result.stats.frames_used} 帧, 耗时 {result.stats.total_time_s:.1f}s")
    else:
        log_error("main", "未生成任何图像")
        sys.exit(1)


if __name__ == '__main__':
    main()
