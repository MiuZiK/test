import pytest
from optimize_stitching.config import StitcherConfig


class TestStitcherConfig:
    def test_default_values(self):
        config = StitcherConfig()
        assert config.base_interval == 60
        assert config.high_threshold == 60.0
        assert config.low_threshold == 20.0
        assert config.min_interval == 5
        assert config.max_interval == 200
        assert config.overlap_ratio == 0.15
        assert config.seamless_blend is True
        assert config.fallback_to_center is True
        assert config.output_width == 600
        assert config.enhance_sharpen is True
        assert config.sharpen_amount == 1.5
        assert config.enhance_contrast is True
        assert config.clahe_clip == 3.5
        assert config.max_frames == 500
        assert config.min_frames_to_process == 10
        assert config.max_silent_frames == 15
        assert config.jpeg_quality == 97
        assert config.segment_height == 8000
        assert config.max_seam_failures == 3
        assert config.max_process_time_s == 2.0
        assert config.max_memory_mb == 1536
        assert config.memory_check_interval_s == 5.0

    def test_custom_values(self):
        config = StitcherConfig(
            base_interval=30,
            high_threshold=70.0,
            low_threshold=30.0,
            min_interval=10,
            max_interval=150,
            overlap_ratio=0.2,
            seamless_blend=False,
            output_width=800,
            enhance_sharpen=False,
            sharpen_amount=2.0,
            enhance_contrast=False,
            clahe_clip=5.0,
            max_frames=300,
            min_frames_to_process=5,
            max_silent_frames=20,
            jpeg_quality=90,
            segment_height=15000
        )
        assert config.base_interval == 30
        assert config.high_threshold == 70.0
        assert config.low_threshold == 30.0
        assert config.min_interval == 10
        assert config.max_interval == 150
        assert config.overlap_ratio == 0.2
        assert config.seamless_blend is False
        assert config.output_width == 800
        assert config.enhance_sharpen is False
        assert config.sharpen_amount == 2.0
        assert config.enhance_contrast is False
        assert config.clahe_clip == 5.0
        assert config.max_frames == 300
        assert config.min_frames_to_process == 5
        assert config.max_silent_frames == 20
        assert config.jpeg_quality == 90
        assert config.segment_height == 15000

    def test_validate_valid_config(self):
        config = StitcherConfig()
        assert config._validated is True

    def test_invalid_base_interval(self):
        with pytest.raises(ValueError):
            StitcherConfig(base_interval=0)
        with pytest.raises(ValueError):
            StitcherConfig(base_interval=501)

    def test_invalid_thresholds(self):
        with pytest.raises(ValueError):
            StitcherConfig(high_threshold=-1.0)
        with pytest.raises(ValueError):
            StitcherConfig(high_threshold=101.0)
        with pytest.raises(ValueError):
            StitcherConfig(low_threshold=-1.0)
        with pytest.raises(ValueError):
            StitcherConfig(low_threshold=101.0)
        with pytest.raises(ValueError):
            StitcherConfig(high_threshold=20.0, low_threshold=40.0)

    def test_invalid_intervals(self):
        with pytest.raises(ValueError):
            StitcherConfig(min_interval=0)
        with pytest.raises(ValueError):
            StitcherConfig(min_interval=101)
        with pytest.raises(ValueError):
            StitcherConfig(max_interval=99)
        with pytest.raises(ValueError):
            StitcherConfig(max_interval=1001)
        with pytest.raises(ValueError):
            StitcherConfig(min_interval=50, max_interval=30)

    def test_invalid_overlap_ratio(self):
        with pytest.raises(ValueError):
            StitcherConfig(overlap_ratio=0.001)
        with pytest.raises(ValueError):
            StitcherConfig(overlap_ratio=0.6)

    def test_invalid_output_width(self):
        with pytest.raises(ValueError):
            StitcherConfig(output_width=99)
        with pytest.raises(ValueError):
            StitcherConfig(output_width=2001)

    def test_invalid_sharpen_amount(self):
        with pytest.raises(ValueError):
            StitcherConfig(sharpen_amount=0.4)
        with pytest.raises(ValueError):
            StitcherConfig(sharpen_amount=3.1)

    def test_invalid_clahe_clip(self):
        with pytest.raises(ValueError):
            StitcherConfig(clahe_clip=0.5)
        with pytest.raises(ValueError):
            StitcherConfig(clahe_clip=10.1)

    def test_invalid_max_frames(self):
        with pytest.raises(ValueError):
            StitcherConfig(max_frames=9)
        with pytest.raises(ValueError):
            StitcherConfig(max_frames=2001)

    def test_invalid_min_frames_to_process(self):
        with pytest.raises(ValueError):
            StitcherConfig(min_frames_to_process=0)
        with pytest.raises(ValueError):
            StitcherConfig(min_frames_to_process=101)

    def test_invalid_max_silent_frames(self):
        with pytest.raises(ValueError):
            StitcherConfig(max_silent_frames=4)
        with pytest.raises(ValueError):
            StitcherConfig(max_silent_frames=51)

    def test_invalid_jpeg_quality(self):
        with pytest.raises(ValueError):
            StitcherConfig(jpeg_quality=0)
        with pytest.raises(ValueError):
            StitcherConfig(jpeg_quality=101)

    def test_invalid_segment_height(self):
        with pytest.raises(ValueError):
            StitcherConfig(segment_height=999)
        with pytest.raises(ValueError):
            StitcherConfig(segment_height=50001)

    def test_copy(self):
        config = StitcherConfig(base_interval=30, output_width=800)
        copied = config.copy()
        assert copied.base_interval == 30
        assert copied.output_width == 800
        assert copied is not config
        assert copied._validated is True
