import pytest
from optimize_stitching.config import StitcherConfig
from optimize_stitching.adaptive_sampler import AdaptiveSampler


class TestAdaptiveSampler:
    def test_dense_sampling(self):
        config = StitcherConfig(base_interval=60)
        sampler = AdaptiveSampler(config)
        interval = sampler.get_next_interval(70)
        assert interval == 30

    def test_normal_sampling(self):
        config = StitcherConfig(base_interval=60)
        sampler = AdaptiveSampler(config)
        interval = sampler.get_next_interval(40)
        assert interval == 60

    def test_sparse_sampling(self):
        config = StitcherConfig(base_interval=60)
        sampler = AdaptiveSampler(config)
        interval = sampler.get_next_interval(10)
        assert interval == 90

    def test_invalid_frame(self):
        config = StitcherConfig(base_interval=60, min_interval=5)
        sampler = AdaptiveSampler(config)
        interval = sampler.get_next_interval(-1)
        assert interval == 5

    def test_interval_bounds(self):
        config = StitcherConfig(base_interval=5, min_interval=3, max_interval=100)
        sampler = AdaptiveSampler(config)
        interval = sampler.get_next_interval(0)
        assert interval == 7

    def test_early_stop_true(self):
        config = StitcherConfig(max_silent_frames=5, min_frames_to_process=10)
        sampler = AdaptiveSampler(config)
        result = sampler.should_early_stop(5, 15)
        assert result is True

    def test_early_stop_false_not_enough_frames(self):
        config = StitcherConfig(max_silent_frames=5, min_frames_to_process=10)
        sampler = AdaptiveSampler(config)
        result = sampler.should_early_stop(5, 5)
        assert result is False

    def test_early_stop_false_not_enough_silent(self):
        config = StitcherConfig(max_silent_frames=5, min_frames_to_process=10)
        sampler = AdaptiveSampler(config)
        result = sampler.should_early_stop(3, 15)
        assert result is False
