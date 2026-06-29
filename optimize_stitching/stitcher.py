import cv2
import numpy as np
import time
from dataclasses import dataclass, field
from typing import List
from .config import StitcherConfig
from .video_reader import VideoReader
from .adaptive_sampler import AdaptiveSampler
from .content_detector import compute_change_score
from .image_enhancer import enhance
from .degrade_manager import DegradeManager
from .optical_flow import estimate_flow_translation, crop_black
from .logger import log_info, log_warning, log_error
from .exceptions import (
    VideoReadError,
    BlackFrameError,
    WhiteFrameError,
    BlendError
)


@dataclass
class ProcessingStats:
    total_frames_read: int = 0
    frames_used: int = 0
    frames_skipped: int = 0
    frames_invalid: int = 0
    early_stopped: bool = False
    total_time_s: float = 0.0
    avg_interval: float = 0.0
    median_displacement: float = 0.0


@dataclass
class StitchResult:
    images: List[np.ndarray] = field(default_factory=list)
    stats: ProcessingStats = field(default_factory=ProcessingStats)
    config: StitcherConfig = None


class AdaptiveStitcher:
    def __init__(self, config: StitcherConfig = None):
        self.config = config or StitcherConfig()
        self.video_reader = None
        self.sampler = AdaptiveSampler(self.config)
        self.degrade_manager = DegradeManager(self.config)
        self.stats = ProcessingStats()

    def _initialize_components(self, video_path: str) -> bool:
        self.video_reader = VideoReader(video_path)
        if not self.video_reader.open():
            raise VideoReadError(f"Failed to open video: {video_path}")

        log_info("AdaptiveStitcher",
                 f"Video opened: {self.video_reader.frame_width}x{self.video_reader.frame_height}, "
                 f"{self.video_reader.total_frames} frames, {self.video_reader.fps:.1f} FPS")

        return True

    def stitch(self, video_path: str) -> StitchResult:
        start_time = time.time()

        try:
            if not self._initialize_components(video_path):
                return StitchResult(stats=self.stats, config=self.config)

            frames = []
            current_frame_index = 0
            prev_frame = None

            while current_frame_index < self.video_reader.total_frames:
                if self.stats.frames_used >= self.config.max_frames:
                    log_info("AdaptiveStitcher", f"Reached max frames limit: {self.config.max_frames}")
                    break

                frame = self.video_reader.read_frame(current_frame_index)
                self.stats.total_frames_read += 1

                if frame is None:
                    self.stats.frames_invalid += 1
                    current_frame_index += 1
                    continue

                frame = self._resize_frame(frame)

                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                mean_val = np.mean(gray)

                if mean_val < 5:
                    self.stats.frames_invalid += 1
                    log_warning("AdaptiveStitcher", f"Black frame at index {current_frame_index}")
                    current_frame_index += self.config.base_interval
                    continue
                elif mean_val > 250:
                    self.stats.frames_invalid += 1
                    log_warning("AdaptiveStitcher", f"White frame at index {current_frame_index}")
                    current_frame_index += self.config.base_interval
                    continue

                if prev_frame is not None:
                    change_score = compute_change_score(prev_frame, frame)

                    if change_score < 0:
                        self.stats.frames_invalid += 1
                        current_frame_index += 1
                        continue

                    interval = self.sampler.get_next_interval(change_score)

                    if self.sampler.should_early_stop(
                            0 if change_score >= self.config.low_threshold else 1,
                            self.stats.frames_used):
                        log_info("AdaptiveStitcher", "Early stopping due to low change")
                        self.stats.early_stopped = True
                        break

                    self.sampler.add_record(
                        frame_index=current_frame_index,
                        interval=interval,
                        change_score=change_score,
                        decision='dense' if change_score > self.config.high_threshold else
                                 'sparse' if change_score < self.config.low_threshold else 'normal'
                    )

                frames.append(frame)
                prev_frame = frame.copy()
                self.stats.frames_used += 1

                if self.stats.frames_used % 25 == 0:
                    progress = (current_frame_index / self.video_reader.total_frames) * 100
                    log_info("AdaptiveStitcher",
                             f"Progress: {progress:.1f}% ({self.stats.frames_used} frames used)")

                current_frame_index += self.config.base_interval
                self.degrade_manager.check_memory()

            result_images = []
            if len(frames) > 0:
                panorama = self._stitch_vertical_longshot(frames)
                if panorama is not None:
                    panorama = enhance(panorama, self.config, self.degrade_manager.should_sharpen())
                    panorama = crop_black(panorama)
                    result_images = self._split_into_segments(panorama)

            self.stats.total_time_s = time.time() - start_time
            self.stats.avg_interval = (current_frame_index / max(self.stats.frames_used, 1))

            log_info("AdaptiveStitcher",
                     f"Processing complete: {self.stats.frames_used} frames used, "
                     f"{self.stats.total_time_s:.1f}s elapsed, "
                     f"median displacement: {self.stats.median_displacement:.1f}px")

        except VideoReadError as e:
            log_error("AdaptiveStitcher.stitch", e)
            result_images = []
        except (BlackFrameError, WhiteFrameError) as e:
            log_warning("AdaptiveStitcher.stitch", str(e))
            result_images = []
        except BlendError as e:
            log_error("AdaptiveStitcher.stitch", e)
            result_images = []
        except Exception as e:
            log_error("AdaptiveStitcher.stitch", e)
            result_images = []
        finally:
            if self.video_reader:
                self.video_reader.close()

        return StitchResult(images=result_images, stats=self.stats, config=self.config)

    def _resize_frame(self, frame: np.ndarray) -> np.ndarray:
        h, w = frame.shape[:2]
        if w != self.config.output_width:
            ratio = self.config.output_width / w
            new_h = int(h * ratio)
            frame = cv2.resize(frame, (self.config.output_width, new_h))
        return frame

    def _stitch_vertical_longshot(self, frames: List[np.ndarray]) -> np.ndarray:
        if len(frames) == 0:
            return None

        if len(frames) == 1:
            log_info("AdaptiveStitcher", "Single frame, returning as-is")
            return frames[0].copy()

        frame_h, frame_w = frames[0].shape[:2]
        y_offsets = [0.0]
        displacements = []

        for i in range(1, len(frames)):
            dx, dy, conf, method = estimate_flow_translation(frames[i-1], frames[i])
            displacements.append((dx, dy, conf, method))
            y_offsets.append(y_offsets[-1] + dy)

        valid_dys = [d[1] for d in displacements if abs(d[1]) > 0.5 and abs(d[1]) < frame_h * 0.8]

        if len(valid_dys) == 0:
            log_warning("AdaptiveStitcher", "No valid displacement detected, using default")
            median_dy = frame_h * 0.5
        else:
            median_dy = np.median(valid_dys)

        self.stats.median_displacement = median_dy

        log_info("AdaptiveStitcher", f"Median vertical displacement: {median_dy:.1f} px")

        smoothed_offsets = [0.0]
        for dx, dy, conf, method in displacements:
            if abs(dy) < 0.5 or abs(dy) > frame_h * 0.8:
                dy = median_dy
            smoothed_offsets.append(smoothed_offsets[-1] + dy)

        min_y = min(smoothed_offsets)
        max_y = max(smoothed_offsets) + frame_h
        canvas_h = int(np.ceil(max_y - min_y))

        norm_off = [y - min_y for y in smoothed_offsets]

        log_info("AdaptiveStitcher", f"Canvas: {frame_w} x {canvas_h} px")

        canvas = np.zeros((canvas_h, frame_w, 3), dtype=np.float64)
        weight = np.zeros((canvas_h, frame_w), dtype=np.float64)

        overlap_px = max(15, int(abs(median_dy) * 0.3))

        for i, (frame, y_off) in enumerate(zip(frames, norm_off)):
            y0 = int(round(y_off))
            y1 = min(y0 + frame_h, canvas_h)

            if y0 < 0 or y1 <= y0 or y0 >= canvas_h:
                continue

            actual_h = y1 - y0
            region = frame[:actual_h].astype(np.float64)

            w_arr = np.ones((actual_h, frame_w), dtype=np.float64)

            if i > 0:
                prev_y_end = int(round(norm_off[i-1])) + frame_h
                overlap_start = max(0, prev_y_end - y0 - overlap_px)
                overlap_end = min(actual_h, prev_y_end - y0 + overlap_px)

                if overlap_end > overlap_start and overlap_start >= 0:
                    blend_len = overlap_end - overlap_start
                    alpha = np.linspace(0.05, 0.95, blend_len)
                    w_arr[overlap_start:overlap_end, :] = np.broadcast_to(
                        alpha.reshape(-1, 1), (blend_len, frame_w)
                    )

            canvas[y0:y1] += region * w_arr[:, :, np.newaxis]
            weight[y0:y1] += w_arr

        mask = weight > 1e-6
        result = np.zeros_like(canvas, dtype=np.uint8)
        result[mask] = (canvas[mask] / weight[mask, np.newaxis]).clip(0, 255).astype(np.uint8)

        return result

    def _split_into_segments(self, panorama: np.ndarray) -> List[np.ndarray]:
        if panorama is None or len(panorama.shape) < 2:
            return []

        h, w = panorama.shape[:2]
        segments = []

        for i in range(0, h, self.config.segment_height):
            end = min(i + self.config.segment_height, h)
            segment = panorama[i:end, :]
            if segment.size > 0:
                segments.append(segment)

        return segments
