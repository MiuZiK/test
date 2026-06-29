import pytest
from unittest.mock import patch, Mock
from optimize_stitching.degrade_manager import DegradeManager
from optimize_stitching.config import StitcherConfig


class TestDegradeManager:
    def test_initial_state(self):
        config = StitcherConfig(seamless_blend=True, enhance_sharpen=True)
        dm = DegradeManager(config)
        assert dm.seam_fail_count == 0
        assert dm.disable_seam is False
        assert dm.disable_sharpen is False
        assert dm.memory_warned is False

    def test_record_seam_failure_not_triggered(self):
        config = StitcherConfig(seamless_blend=True)
        dm = DegradeManager(config)
        dm.max_seam_failures = 3

        assert dm.record_seam_failure() is False
        assert dm.seam_fail_count == 1
        assert dm.disable_seam is False

        assert dm.record_seam_failure() is False
        assert dm.seam_fail_count == 2
        assert dm.disable_seam is False

    def test_record_seam_failure_triggered(self):
        config = StitcherConfig(seamless_blend=True)
        dm = DegradeManager(config)
        dm.max_seam_failures = 3

        dm.record_seam_failure()
        dm.record_seam_failure()
        result = dm.record_seam_failure()

        assert result is True
        assert dm.seam_fail_count == 3
        assert dm.disable_seam is True

    def test_record_seam_failure_already_disabled(self):
        config = StitcherConfig(seamless_blend=True)
        dm = DegradeManager(config)
        dm.disable_seam = True

        result = dm.record_seam_failure()
        assert result is True

    def test_should_use_seam_enabled(self):
        config = StitcherConfig(seamless_blend=True)
        dm = DegradeManager(config)
        assert dm.should_use_seam() is True

    def test_should_use_seam_disabled_by_config(self):
        config = StitcherConfig(seamless_blend=False)
        dm = DegradeManager(config)
        assert dm.should_use_seam() is False

    def test_should_use_seam_disabled_by_degrade(self):
        config = StitcherConfig(seamless_blend=True)
        dm = DegradeManager(config)
        dm.disable_seam = True
        assert dm.should_use_seam() is False

    def test_record_frame_time_not_triggered(self):
        config = StitcherConfig(enhance_sharpen=True)
        dm = DegradeManager(config)
        dm.max_process_time_s = 2.0

        for _ in range(5):
            result = dm.record_frame_time(0.5)
            assert result is False

        assert dm.disable_sharpen is False

    def test_record_frame_time_triggered(self):
        config = StitcherConfig(enhance_sharpen=True, max_process_time_s=0.5)
        dm = DegradeManager(config)

        for _ in range(10):
            dm.record_frame_time(1.0)

        assert dm.disable_sharpen is True

    def test_record_frame_time_already_disabled(self):
        config = StitcherConfig(enhance_sharpen=True)
        dm = DegradeManager(config)
        dm.disable_sharpen = True

        result = dm.record_frame_time(10.0)
        assert result is True

    def test_should_sharpen_enabled(self):
        config = StitcherConfig(enhance_sharpen=True)
        dm = DegradeManager(config)
        assert dm.should_sharpen() is True

    def test_should_sharpen_disabled_by_config(self):
        config = StitcherConfig(enhance_sharpen=False)
        dm = DegradeManager(config)
        assert dm.should_sharpen() is False

    def test_should_sharpen_disabled_by_degrade(self):
        config = StitcherConfig(enhance_sharpen=True)
        dm = DegradeManager(config)
        dm.disable_sharpen = True
        assert dm.should_sharpen() is False

    def test_check_memory_within_limit(self):
        config = StitcherConfig()
        dm = DegradeManager(config)
        dm.max_memory_mb = 10000
        result = dm.check_memory()
        assert result is False
        assert dm.memory_warned is False

    def test_check_memory_interval(self):
        config = StitcherConfig()
        dm = DegradeManager(config)
        dm.memory_check_interval_s = 60.0
        dm.check_memory()
        result = dm.check_memory()
        assert result is False

    def test_suggest_output_width_reduction_within_limit(self):
        config = StitcherConfig()
        dm = DegradeManager(config)
        dm.max_memory_mb = 10000
        reduction = dm.suggest_output_width_reduction()
        assert reduction == 0.0

    def test_suggest_output_width_reduction_exceeded(self):
        config = StitcherConfig(max_memory_mb=512)
        dm = DegradeManager(config)

        mock_process = Mock()
        mock_process.memory_info.return_value = Mock(rss=1024 * 1024 * 1024)

        with patch('optimize_stitching.degrade_manager.psutil.Process', return_value=mock_process):
            reduction = dm.suggest_output_width_reduction()
            assert 0 < reduction <= 0.2

    def test_reset(self):
        config = StitcherConfig()
        dm = DegradeManager(config)
        dm.seam_fail_count = 5
        dm.disable_seam = True
        dm.frame_process_times = [1.0, 2.0, 3.0]
        dm.disable_sharpen = True
        dm.memory_warned = True

        dm.reset()

        assert dm.seam_fail_count == 0
        assert dm.disable_seam is False
        assert dm.frame_process_times == []
        assert dm.disable_sharpen is False
        assert dm.memory_warned is False

    def test_frame_process_times_window(self):
        config = StitcherConfig()
        dm = DegradeManager(config)

        for i in range(15):
            dm.record_frame_time(float(i))

        assert len(dm.frame_process_times) == 10
        assert dm.frame_process_times[0] == 5.0
        assert dm.frame_process_times[-1] == 14.0
