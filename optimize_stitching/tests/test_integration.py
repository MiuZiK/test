import numpy as np
import os
import pytest
from optimize_stitching.stitcher import AdaptiveStitcher, StitchResult
from optimize_stitching.config import StitcherConfig


class TestIntegration:
    def test_stitcher_initialization(self):
        config = StitcherConfig()
        stitcher = AdaptiveStitcher(config)
        assert stitcher.config is not None

    def test_stitcher_with_config(self):
        config = StitcherConfig(base_interval=30, output_width=400)
        stitcher = AdaptiveStitcher(config)
        assert stitcher.config.base_interval == 30
        assert stitcher.config.output_width == 400

    def test_config_validation(self):
        config = StitcherConfig()
        assert config._validated is True

    def test_config_invalid_base_interval(self):
        with pytest.raises(ValueError):
            StitcherConfig(base_interval=0)

    def test_config_invalid_thresholds(self):
        with pytest.raises(ValueError):
            StitcherConfig(high_threshold=10, low_threshold=20)

    def test_empty_result(self):
        result = StitchResult()
        assert len(result.images) == 0

    def test_result_with_images(self):
        images = [np.zeros((100, 100, 3), dtype=np.uint8)]
        stats = StitchResult(images=images)
        assert len(stats.images) == 1
