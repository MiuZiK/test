import numpy as np
import pytest
from optimize_stitching.seam_blender import blend_with_seam, simple_alpha_blend


class TestSeamBlender:
    def test_none_input(self):
        frame1 = np.zeros((50, 50, 3), dtype=np.uint8)
        with pytest.raises(ValueError):
            blend_with_seam(None, frame1, np.array([25]))

    def test_empty_seam(self):
        frame1 = np.zeros((50, 50, 3), dtype=np.uint8)
        frame2 = np.ones((50, 50, 3), dtype=np.uint8) * 255
        with pytest.raises(ValueError):
            blend_with_seam(frame1, frame2, None)

    def test_height_mismatch(self):
        frame1 = np.zeros((50, 50, 3), dtype=np.uint8)
        frame2 = np.ones((30, 50, 3), dtype=np.uint8) * 255
        seam = np.full(30, 25, dtype=np.int32)
        result = blend_with_seam(frame1, frame2, seam)
        assert result.shape[0] == 30

    def test_seam_line_length_mismatch(self):
        frame1 = np.zeros((50, 50, 3), dtype=np.uint8)
        frame2 = np.ones((50, 50, 3), dtype=np.uint8) * 255
        seam = np.full(30, 25, dtype=np.int32)
        result = blend_with_seam(frame1, frame2, seam)
        assert result.shape[0] == 50

    def test_simple_alpha_blend(self):
        frame1 = np.zeros((50, 50), dtype=np.uint8)
        frame2 = np.ones((50, 50), dtype=np.uint8) * 255
        result = simple_alpha_blend(frame1, frame2, 10)
        assert result.shape == (50, 50)

    def test_simple_alpha_blend_color(self):
        frame1 = np.zeros((50, 50, 3), dtype=np.uint8)
        frame2 = np.ones((50, 50, 3), dtype=np.uint8) * 255
        result = simple_alpha_blend(frame1, frame2, 10)
        assert result.shape == (50, 50, 3)
