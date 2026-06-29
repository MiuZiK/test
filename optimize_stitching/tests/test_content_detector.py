import numpy as np
import pytest
from optimize_stitching.content_detector import compute_change_score, detect_interest_regions


class TestComputeChangeScore:
    def test_identical_frames(self):
        frame = np.ones((100, 100, 3), dtype=np.uint8) * 128
        frame[:, :50] = 100
        score = compute_change_score(frame, frame)
        assert abs(score) < 1.0

    def test_black_frame(self):
        black = np.zeros((100, 100, 3), dtype=np.uint8)
        normal = np.ones((100, 100, 3), dtype=np.uint8) * 128
        score = compute_change_score(black, normal)
        assert score == -1.0

    def test_white_frame(self):
        white = np.ones((100, 100, 3), dtype=np.uint8) * 255
        normal = np.ones((100, 100, 3), dtype=np.uint8) * 128
        score = compute_change_score(white, normal)
        assert score == -1.0

    def test_low_variance(self):
        frame1 = np.ones((100, 100, 3), dtype=np.uint8) * 128
        frame2 = np.ones((100, 100, 3), dtype=np.uint8) * 129
        score = compute_change_score(frame1, frame2)
        assert abs(score) < 1.0

    def test_dimension_mismatch(self):
        frame1 = np.ones((100, 100, 3), dtype=np.uint8) * 128
        frame1[:, :50] = 100
        frame2 = np.ones((80, 120, 3), dtype=np.uint8) * 128
        frame2[:, :50] = 150
        score = compute_change_score(frame1, frame2)
        assert 0 <= score <= 100

    def test_extreme_diff(self):
        frame1 = np.ones((100, 100, 3), dtype=np.uint8) * 10
        frame1[:, :50] = 0
        frame2 = np.ones((100, 100, 3), dtype=np.uint8) * 245
        frame2[:, :50] = 255
        score = compute_change_score(frame1, frame2)
        assert score >= 90.0

    def test_gray_frames(self):
        frame1 = np.ones((100, 100), dtype=np.uint8) * 50
        frame1[:, :50] = 100
        frame2 = np.ones((100, 100), dtype=np.uint8) * 150
        frame2[:, :50] = 200
        score = compute_change_score(frame1, frame2)
        assert 0 < score <= 100


class TestDetectInterestRegions:
    def test_no_regions(self):
        frame1 = np.zeros((100, 100, 3), dtype=np.uint8)
        frame2 = np.zeros((100, 100, 3), dtype=np.uint8)
        regions = detect_interest_regions(frame1, frame2)
        assert len(regions) == 0

    def test_with_regions(self):
        frame1 = np.zeros((100, 100, 3), dtype=np.uint8)
        frame2 = np.zeros((100, 100, 3), dtype=np.uint8)
        frame2[20:80, 20:80] = 255
        regions = detect_interest_regions(frame1, frame2, min_area=100)
        assert len(regions) >= 1

    def test_dimension_mismatch(self):
        frame1 = np.zeros((100, 100, 3), dtype=np.uint8)
        frame2 = np.zeros((80, 120, 3), dtype=np.uint8)
        frame2[10:70, 10:110] = 255
        regions = detect_interest_regions(frame1, frame2)
        assert len(regions) >= 0
