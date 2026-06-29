import cv2
import numpy as np
from .exceptions import VideoReadError, CorruptFrameError
from .logger import log_error, log_warning


class VideoReader:
    def __init__(self, video_path: str):
        self.video_path = video_path
        self.cap = None
        self.total_frames = 0
        self.frame_width = 0
        self.frame_height = 0
        self.fps = 0

    def open(self) -> bool:
        try:
            self.cap = cv2.VideoCapture(self.video_path)
            if not self.cap.isOpened():
                raise VideoReadError(f"Cannot open video: {self.video_path}")

            self.total_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
            self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            self.fps = self.cap.get(cv2.CAP_PROP_FPS)

            return True
        except Exception as e:
            log_error("VideoReader.open", e)
            return False

    def read(self) -> np.ndarray:
        if self.cap is None or not self.cap.isOpened():
            raise VideoReadError("Video not opened")

        try:
            ret, frame = self.cap.read()
            if not ret:
                log_warning("VideoReader.read", "End of video stream")
                return None
            return frame
        except Exception as e:
            log_error("VideoReader.read", e)
            return None

    def read_frame(self, frame_index: int) -> np.ndarray:
        if self.cap is None or not self.cap.isOpened():
            raise VideoReadError("Video not opened")

        if frame_index < 0 or frame_index >= self.total_frames:
            log_warning("VideoReader.read_frame", f"Frame index {frame_index} out of range")
            return None

        try:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, frame_index)
            ret, frame = self.cap.read()
            if not ret:
                raise CorruptFrameError(f"Failed to read frame {frame_index}")
            return frame
        except Exception as e:
            log_error("VideoReader.read_frame", e)
            return None

    def close(self) -> None:
        if self.cap is not None:
            self.cap.release()
            self.cap = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
