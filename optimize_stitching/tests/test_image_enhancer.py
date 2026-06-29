import numpy as np
import pytest
from optimize_stitching.image_enhancer import multiscale_sharpen, optimize_contrast, enhance
from optimize_stitching.config import StitcherConfig


class TestImageEnhancer:
    def test_sharpen_empty(self):
        result = multiscale_sharpen(None)
        assert result is None

    def test_sharpen_zero(self):
        image = np.zeros((50, 50, 3), dtype=np.uint8)
        result = multiscale_sharpen(image)
        assert result.shape == image.shape

    def test_sharpen_gray(self):
        image = np.zeros((50, 50), dtype=np.uint8)
        result = multiscale_sharpen(image)
        assert result.shape == image.shape

    def test_contrast_empty(self):
        result = optimize_contrast(None)
        assert result is None

    def test_contrast_gray(self):
        image = np.random.randint(0, 255, (50, 50), dtype=np.uint8)
        result = optimize_contrast(image)
        assert result.shape == image.shape

    def test_contrast_color(self):
        image = np.random.randint(0, 255, (50, 50, 3), dtype=np.uint8)
        result = optimize_contrast(image)
        assert result.shape == image.shape

    def test_enhance_disabled(self):
        image = np.zeros((50, 50, 3), dtype=np.uint8)
        config = StitcherConfig(enhance_sharpen=False, enhance_contrast=False)
        result = enhance(image, config)
        assert result.shape == image.shape

    def test_enhance_enabled(self):
        image = np.random.randint(0, 255, (50, 50, 3), dtype=np.uint8)
        config = StitcherConfig(enhance_sharpen=True, enhance_contrast=True)
        result = enhance(image, config)
        assert result.shape == image.shape
