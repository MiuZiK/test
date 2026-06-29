import numpy as np
import pytest
from optimize_stitching.seam_detector import find_seam_line


class TestSeamDetector:
    def test_empty_region(self):
        with pytest.raises(ValueError):
            find_seam_line(None)

    def test_small_region(self):
        region = np.zeros((2, 2, 3), dtype=np.uint8)
        seam = find_seam_line(region)
        assert len(seam) == 2
        assert np.all(seam == 1)

    def test_low_texture(self):
        region = np.ones((50, 50, 3), dtype=np.uint8) * 128
        seam = find_seam_line(region)
        assert len(seam) == 50
        assert np.all(seam == 25)

    def test_low_texture_no_fallback(self):
        region = np.ones((50, 50, 3), dtype=np.uint8) * 128
        with pytest.raises(ValueError):
            find_seam_line(region, fallback_to_center=False)

    def test_gradient_region(self):
        region = np.zeros((50, 50, 3), dtype=np.uint8)
        for x in range(50):
            region[:, x] = x * 5
        seam = find_seam_line(region)
        assert len(seam) == 50
        assert np.all(seam >= 0)
        assert np.all(seam < 50)

    def test_gray_region(self):
        region = np.zeros((50, 50), dtype=np.uint8)
        region[20:30, :] = 255
        seam = find_seam_line(region)
        assert len(seam) == 50
