import pytest
import cv2
import numpy as np
from unittest.mock import Mock, patch
from optimize_stitching.video_reader import VideoReader
from optimize_stitching.exceptions import VideoReadError, CorruptFrameError


class TestVideoReader:
    def test_initial_state(self):
        reader = VideoReader("test.mp4")
        assert reader.video_path == "test.mp4"
        assert reader.cap is None
        assert reader.total_frames == 0
        assert reader.frame_width == 0
        assert reader.frame_height == 0
        assert reader.fps == 0

    def test_open_success(self):
        mock_cap = Mock()
        mock_cap.isOpened.return_value = True
        mock_cap.get.side_effect = [100, 800, 600, 30.0]

        with patch('cv2.VideoCapture', return_value=mock_cap):
            reader = VideoReader("test.mp4")
            result = reader.open()

            assert result is True
            assert reader.total_frames == 100
            assert reader.frame_width == 800
            assert reader.frame_height == 600
            assert reader.fps == 30.0
            assert reader.cap is mock_cap

    def test_open_failure(self):
        mock_cap = Mock()
        mock_cap.isOpened.return_value = False

        with patch('cv2.VideoCapture', return_value=mock_cap):
            reader = VideoReader("test.mp4")
            result = reader.open()

            assert result is False

    def test_read_success(self):
        mock_cap = Mock()
        mock_cap.isOpened.return_value = True
        mock_frame = np.zeros((100, 100, 3), dtype=np.uint8)
        mock_cap.read.return_value = (True, mock_frame)

        with patch('cv2.VideoCapture', return_value=mock_cap):
            reader = VideoReader("test.mp4")
            reader.open()
            frame = reader.read()

            assert frame is not None
            assert frame.shape == (100, 100, 3)
            np.testing.assert_array_equal(frame, mock_frame)

    def test_read_failure(self):
        mock_cap = Mock()
        mock_cap.isOpened.return_value = True
        mock_cap.read.return_value = (False, None)

        with patch('cv2.VideoCapture', return_value=mock_cap):
            reader = VideoReader("test.mp4")
            reader.open()
            frame = reader.read()

            assert frame is None

    def test_read_not_opened(self):
        reader = VideoReader("test.mp4")
        with pytest.raises(VideoReadError):
            reader.read()

    def test_read_frame_success(self):
        mock_cap = Mock()
        mock_cap.isOpened.return_value = True
        mock_cap.get.side_effect = [100, 800, 600, 30.0]
        mock_frame = np.zeros((100, 100, 3), dtype=np.uint8)
        mock_cap.read.return_value = (True, mock_frame)

        with patch('cv2.VideoCapture', return_value=mock_cap):
            reader = VideoReader("test.mp4")
            reader.open()
            frame = reader.read_frame(10)

            assert frame is not None
            assert frame.shape == (100, 100, 3)
            mock_cap.set.assert_called_with(cv2.CAP_PROP_POS_FRAMES, 10)

    def test_read_frame_out_of_range(self):
        mock_cap = Mock()
        mock_cap.isOpened.return_value = True
        mock_cap.get.side_effect = [100, 800, 600, 30.0]

        with patch('cv2.VideoCapture', return_value=mock_cap):
            reader = VideoReader("test.mp4")
            reader.open()

            frame = reader.read_frame(-1)
            assert frame is None

            frame = reader.read_frame(100)
            assert frame is None

    def test_read_frame_not_opened(self):
        reader = VideoReader("test.mp4")
        with pytest.raises(VideoReadError):
            reader.read_frame(0)

    def test_read_frame_failure(self):
        mock_cap = Mock()
        mock_cap.isOpened.return_value = True
        mock_cap.get.side_effect = [100, 800, 600, 30.0]
        mock_cap.read.return_value = (False, None)

        with patch('cv2.VideoCapture', return_value=mock_cap):
            reader = VideoReader("test.mp4")
            reader.open()
            frame = reader.read_frame(10)

            assert frame is None

    def test_close(self):
        mock_cap = Mock()
        mock_cap.isOpened.return_value = True

        with patch('cv2.VideoCapture', return_value=mock_cap):
            reader = VideoReader("test.mp4")
            reader.open()
            reader.close()

            mock_cap.release.assert_called_once()
            assert reader.cap is None

    def test_close_when_not_open(self):
        reader = VideoReader("test.mp4")
        reader.close()
        assert reader.cap is None

    def test_context_manager(self):
        mock_cap = Mock()
        mock_cap.isOpened.return_value = True
        mock_cap.get.side_effect = [100, 800, 600, 30.0]

        with patch('cv2.VideoCapture', return_value=mock_cap):
            with VideoReader("test.mp4") as reader:
                assert reader.cap is not None
                assert reader.total_frames == 100

            mock_cap.release.assert_called_once()

    def test_video_properties(self):
        mock_cap = Mock()
        mock_cap.isOpened.return_value = True
        mock_cap.get.side_effect = [500, 1920, 1080, 29.97]

        with patch('cv2.VideoCapture', return_value=mock_cap):
            reader = VideoReader("test.mp4")
            reader.open()

            assert reader.total_frames == 500
            assert reader.frame_width == 1920
            assert reader.frame_height == 1080
            assert abs(reader.fps - 29.97) < 0.01
