import pytest
import numpy as np
import os
import tempfile
from unittest.mock import patch, Mock
from optimize_stitching.output_writer import write_segments, save_full_image
from optimize_stitching.exceptions import OutputWriteError


class TestOutputWriter:
    def test_write_segments_empty_list(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            result = write_segments([], tmpdir)
            assert result == []

    def test_write_segments_with_images(self):
        images = [
            np.zeros((100, 100, 3), dtype=np.uint8),
            np.ones((100, 100, 3), dtype=np.uint8) * 255,
        ]

        with tempfile.TemporaryDirectory() as tmpdir:
            result = write_segments(images, tmpdir)

            assert len(result) == 2
            assert os.path.exists(result[0])
            assert os.path.exists(result[1])
            assert "output_001.jpg" in result[0]
            assert "output_002.jpg" in result[1]

    def test_write_segments_with_prefix(self):
        images = [np.zeros((100, 100, 3), dtype=np.uint8)]

        with tempfile.TemporaryDirectory() as tmpdir:
            result = write_segments(images, tmpdir, prefix="test_prefix")

            assert len(result) == 1
            assert "test_prefix_001.jpg" in result[0]

    def test_write_segments_with_quality(self):
        images = [np.zeros((100, 100, 3), dtype=np.uint8)]

        with tempfile.TemporaryDirectory() as tmpdir:
            result = write_segments(images, tmpdir, quality=80)

            assert len(result) == 1
            assert os.path.exists(result[0])

    def test_write_segments_skips_empty_images(self):
        images = [
            np.zeros((100, 100, 3), dtype=np.uint8),
            None,
            np.array([]),
            np.ones((100, 100, 3), dtype=np.uint8),
        ]

        with tempfile.TemporaryDirectory() as tmpdir:
            result = write_segments(images, tmpdir)

            assert len(result) == 2

    def test_write_segments_failure(self):
        images = [np.zeros((100, 100, 3), dtype=np.uint8)]

        with tempfile.TemporaryDirectory() as tmpdir:
            with patch('cv2.imwrite', return_value=False):
                result = write_segments(images, tmpdir)
                assert result == []

    def test_write_segments_exception(self):
        images = [np.zeros((100, 100, 3), dtype=np.uint8)]

        with tempfile.TemporaryDirectory() as tmpdir:
            with patch('cv2.imwrite', side_effect=Exception("test error")):
                result = write_segments(images, tmpdir)
                assert result == []

    def test_save_full_image_success(self):
        image = np.zeros((100, 100, 3), dtype=np.uint8)

        with tempfile.TemporaryDirectory() as tmpdir:
            filepath = os.path.join(tmpdir, "output.jpg")
            result = save_full_image(image, filepath)

            assert result is True
            assert os.path.exists(filepath)

    def test_save_full_image_with_subdirectory(self):
        image = np.zeros((100, 100, 3), dtype=np.uint8)

        with tempfile.TemporaryDirectory() as tmpdir:
            filepath = os.path.join(tmpdir, "subdir", "output.jpg")
            result = save_full_image(image, filepath)

            assert result is True
            assert os.path.exists(filepath)

    def test_save_full_image_with_quality(self):
        image = np.zeros((100, 100, 3), dtype=np.uint8)

        with tempfile.TemporaryDirectory() as tmpdir:
            filepath = os.path.join(tmpdir, "output.jpg")
            result = save_full_image(image, filepath, quality=80)

            assert result is True
            assert os.path.exists(filepath)

    def test_save_full_image_failure(self):
        image = np.zeros((100, 100, 3), dtype=np.uint8)

        with tempfile.TemporaryDirectory() as tmpdir:
            filepath = os.path.join(tmpdir, "output.jpg")
            with patch('cv2.imwrite', return_value=False):
                result = save_full_image(image, filepath)
                assert result is False

    def test_save_full_image_exception(self):
        image = np.zeros((100, 100, 3), dtype=np.uint8)

        with tempfile.TemporaryDirectory() as tmpdir:
            filepath = os.path.join(tmpdir, "output.jpg")
            with patch('cv2.imwrite', side_effect=Exception("test error")):
                result = save_full_image(image, filepath)
                assert result is False

    def test_write_segments_creates_directory(self):
        images = [np.zeros((100, 100, 3), dtype=np.uint8)]

        with tempfile.TemporaryDirectory() as tmpdir:
            nested_dir = os.path.join(tmpdir, "level1", "level2")
            result = write_segments(images, nested_dir)

            assert len(result) == 1
            assert os.path.exists(nested_dir)

    def test_save_full_image_no_directory(self):
        image = np.zeros((100, 100, 3), dtype=np.uint8)

        with tempfile.TemporaryDirectory() as tmpdir:
            filepath = os.path.join(tmpdir, "output.jpg")
            result = save_full_image(image, filepath)
            assert result is True

    def test_write_segments_multiple_calls(self):
        images1 = [np.zeros((100, 100, 3), dtype=np.uint8)]
        images2 = [np.ones((100, 100, 3), dtype=np.uint8) * 128]

        with tempfile.TemporaryDirectory() as tmpdir:
            result1 = write_segments(images1, tmpdir)
            result2 = write_segments(images2, tmpdir, prefix="second")

            assert len(result1) == 1
            assert len(result2) == 1
            assert "output_001.jpg" in result1[0]
            assert "second_001.jpg" in result2[0]
