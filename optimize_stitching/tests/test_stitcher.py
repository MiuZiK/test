import pytest
import numpy as np
from unittest.mock import Mock, patch
from optimize_stitching.stitcher import AdaptiveStitcher, StitchResult, ProcessingStats
from optimize_stitching.config import StitcherConfig


class TestAdaptiveStitcher:
    def test_initialization_default_config(self):
        stitcher = AdaptiveStitcher()
        assert isinstance(stitcher.config, StitcherConfig)
        assert stitcher.video_reader is None
        assert stitcher.sampler is not None
        assert stitcher.degrade_manager is not None
        assert isinstance(stitcher.stats, ProcessingStats)

    def test_initialization_custom_config(self):
        config = StitcherConfig(base_interval=30, output_width=800)
        stitcher = AdaptiveStitcher(config)
        assert stitcher.config.base_interval == 30
        assert stitcher.config.output_width == 800

    def test_stitch_with_nonexistent_video(self):
        stitcher = AdaptiveStitcher()
        result = stitcher.stitch("nonexistent_video.mp4")
        assert isinstance(result, StitchResult)
        assert len(result.images) == 0
        assert result.stats.total_frames_read == 0

    def test_stitch_with_valid_video(self):
        mock_frame1 = np.ones((100, 200, 3), dtype=np.uint8) * 100
        mock_frame2 = np.ones((100, 200, 3), dtype=np.uint8) * 150
        mock_frame3 = np.ones((100, 200, 3), dtype=np.uint8) * 200

        mock_reader = Mock()
        mock_reader.open.return_value = True
        mock_reader.total_frames = 100
        mock_reader.frame_width = 200
        mock_reader.frame_height = 100
        mock_reader.fps = 30.0
        mock_reader.read_frame.side_effect = [mock_frame1, mock_frame2, mock_frame3, None, None]
        mock_reader.close.return_value = None

        with patch('optimize_stitching.stitcher.VideoReader', return_value=mock_reader):
            config = StitcherConfig(base_interval=1, max_frames=50)
            stitcher = AdaptiveStitcher(config)
            result = stitcher.stitch("test.mp4")

            assert isinstance(result, StitchResult)
            assert result.stats.frames_used >= 1
            assert result.stats.total_frames_read >= 1

    def test_stitch_empty_video(self):
        mock_reader = Mock()
        mock_reader.open.return_value = True
        mock_reader.total_frames = 0
        mock_reader.frame_width = 200
        mock_reader.frame_height = 100
        mock_reader.fps = 30.0
        mock_reader.read_frame.return_value = None
        mock_reader.close.return_value = None

        with patch('optimize_stitching.stitcher.VideoReader', return_value=mock_reader):
            stitcher = AdaptiveStitcher()
            result = stitcher.stitch("test.mp4")

            assert len(result.images) == 0
            assert result.stats.frames_used == 0

    def test_stitch_with_all_invalid_frames(self):
        black_frame = np.zeros((100, 200, 3), dtype=np.uint8)
        black_frame[:, :50] = 100

        mock_reader = Mock()
        mock_reader.open.return_value = True
        mock_reader.total_frames = 5
        mock_reader.frame_width = 200
        mock_reader.frame_height = 100
        mock_reader.fps = 30.0
        mock_reader.read_frame.return_value = black_frame
        mock_reader.close.return_value = None

        with patch('optimize_stitching.stitcher.VideoReader', return_value=mock_reader):
            stitcher = AdaptiveStitcher()
            result = stitcher.stitch("test.mp4")

            assert result.stats.frames_used >= 1

    def test_resize_frame(self):
        config = StitcherConfig(output_width=100)
        stitcher = AdaptiveStitcher(config)
        frame = np.zeros((200, 200, 3), dtype=np.uint8)

        resized = stitcher._resize_frame(frame)

        assert resized.shape[1] == 100
        assert resized.shape[0] == 100

    def test_resize_frame_no_change(self):
        config = StitcherConfig(output_width=200)
        stitcher = AdaptiveStitcher(config)
        frame = np.zeros((100, 200, 3), dtype=np.uint8)

        resized = stitcher._resize_frame(frame)

        assert resized.shape == (100, 200, 3)

    def test_split_into_segments_small_image(self):
        stitcher = AdaptiveStitcher()
        stitcher.config.segment_height = 1000
        image = np.zeros((500, 1000, 3), dtype=np.uint8)

        segments = stitcher._split_into_segments(image)

        assert len(segments) == 1
        assert segments[0].shape == (500, 1000, 3)

    def test_split_into_segments_large_image(self):
        stitcher = AdaptiveStitcher()
        stitcher.config.segment_height = 100
        image = np.zeros((350, 200, 3), dtype=np.uint8)

        segments = stitcher._split_into_segments(image)

        assert len(segments) == 4
        assert segments[0].shape == (100, 200, 3)
        assert segments[1].shape == (100, 200, 3)
        assert segments[2].shape == (100, 200, 3)
        assert segments[3].shape == (50, 200, 3)

    def test_stats_initial_values(self):
        stats = ProcessingStats()
        assert stats.total_frames_read == 0
        assert stats.frames_used == 0
        assert stats.frames_skipped == 0
        assert stats.frames_invalid == 0
        assert stats.early_stopped is False
        assert stats.total_time_s == 0.0
        assert stats.avg_interval == 0.0

    def test_stitch_result_default(self):
        result = StitchResult()
        assert result.images == []
        assert isinstance(result.stats, ProcessingStats)
        assert result.config is None

    def test_stitch_result_with_data(self):
        images = [np.zeros((100, 100, 3), dtype=np.uint8)]
        stats = ProcessingStats(frames_used=5)
        config = StitcherConfig()

        result = StitchResult(images=images, stats=stats, config=config)

        assert result.images == images
        assert result.stats.frames_used == 5
        assert result.config == config

    def test_stitch_with_early_stop(self):
        mock_frame = np.ones((100, 200, 3), dtype=np.uint8) * 128
        mock_frame[:, :50] = 120

        mock_reader = Mock()
        mock_reader.open.return_value = True
        mock_reader.total_frames = 100
        mock_reader.frame_width = 200
        mock_reader.frame_height = 100
        mock_reader.fps = 30.0
        mock_reader.read_frame.return_value = mock_frame
        mock_reader.close.return_value = None

        with patch('optimize_stitching.stitcher.VideoReader', return_value=mock_reader):
            config = StitcherConfig(max_silent_frames=5, min_frames_to_process=5, base_interval=1)
            stitcher = AdaptiveStitcher(config)
            result = stitcher.stitch("test.mp4")

            assert result.stats.frames_used >= 5

    def test_stitch_memory_check(self):
        mock_frame = np.ones((100, 200, 3), dtype=np.uint8) * 128

        mock_reader = Mock()
        mock_reader.open.return_value = True
        mock_reader.total_frames = 10
        mock_reader.frame_width = 200
        mock_reader.frame_height = 100
        mock_reader.fps = 30.0
        mock_reader.read_frame.return_value = mock_frame
        mock_reader.close.return_value = None

        with patch('optimize_stitching.stitcher.VideoReader', return_value=mock_reader):
            stitcher = AdaptiveStitcher()
            result = stitcher.stitch("test.mp4")

            assert result is not None

    def test_split_into_segments_empty_image(self):
        stitcher = AdaptiveStitcher()
        result = stitcher._split_into_segments(np.array([]))
        assert result == []
