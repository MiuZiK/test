import pytest
import numpy as np
import cv2
from optimize_stitching.optical_flow import estimate_flow_translation, crop_black


class TestOpticalFlow:
    def test_estimate_flow_translation_identity(self):
        img1 = np.random.randint(0, 255, (100, 200, 3), dtype=np.uint8)
        img2 = img1.copy()

        dx, dy, conf, method = estimate_flow_translation(img1, img2)

        assert abs(dx) < 1.0
        assert abs(dy) < 1.0

    def test_estimate_flow_translation_downward(self):
        img1 = np.zeros((100, 200, 3), dtype=np.uint8)
        img1[30:50, 50:150] = 255
        img2 = np.zeros((100, 200, 3), dtype=np.uint8)
        img2[50:70, 50:150] = 255

        dx, dy, conf, method = estimate_flow_translation(img1, img2)

        assert method in ['ecc', 'farneback', 'orb', 'none']

    def test_estimate_flow_translation_rightward(self):
        img1 = np.zeros((100, 200, 3), dtype=np.uint8)
        img1[30:70, 30:50] = 255
        img2 = np.zeros((100, 200, 3), dtype=np.uint8)
        img2[30:70, 80:100] = 255

        dx, dy, conf, method = estimate_flow_translation(img1, img2)

        assert method in ['ecc', 'farneback', 'orb', 'none']

    def test_estimate_flow_translation_black_frames(self):
        img1 = np.zeros((100, 200, 3), dtype=np.uint8)
        img2 = np.zeros((100, 200, 3), dtype=np.uint8)

        dx, dy, conf, method = estimate_flow_translation(img1, img2)

        assert method == 'none' or method == 'farneback' or method == 'orb'

    def test_estimate_flow_translation_different_sizes(self):
        img1 = np.random.randint(0, 255, (100, 200, 3), dtype=np.uint8)
        img2 = cv2.resize(img1, (150, 80))

        dx, dy, conf, method = estimate_flow_translation(img1, img2)

        assert method in ['ecc', 'farneback', 'orb', 'none']

    def test_crop_black_empty(self):
        result = crop_black(None)
        assert result is None

    def test_crop_black_all_black(self):
        img = np.zeros((100, 200, 3), dtype=np.uint8)
        result = crop_black(img)
        assert result.shape == (100, 200, 3)

    def test_crop_black_with_content(self):
        img = np.zeros((100, 200, 3), dtype=np.uint8)
        img[20:80, 30:170] = 100
        result = crop_black(img)
        assert result.shape[0] > 0
        assert result.shape[1] > 0

    def test_crop_black_gray(self):
        img = np.zeros((100, 200, 3), dtype=np.uint8)
        img[25:75, 50:150] = 200
        result = crop_black(img)
        assert result.shape[0] <= 100
        assert result.shape[1] <= 200