import psutil
import time
from .config import StitcherConfig
from .logger import log_warning


class DegradeManager:
    def __init__(self, config: StitcherConfig):
        self.config = config
        self.seam_fail_count = 0
        self.disable_seam = False

        self.frame_process_times = []
        self.disable_sharpen = False

        self.last_memory_check = 0
        self.memory_warned = False

    def record_seam_failure(self) -> bool:
        self.seam_fail_count += 1
        if self.seam_fail_count >= self.config.max_seam_failures and not self.disable_seam:
            self.disable_seam = True
            log_warning("DegradeManager", "Disabling seam blending due to repeated failures")
            return True
        return self.disable_seam

    def should_use_seam(self) -> bool:
        return self.config.seamless_blend and not self.disable_seam

    def record_frame_time(self, elapsed_s: float) -> bool:
        self.frame_process_times.append(elapsed_s)
        if len(self.frame_process_times) > 10:
            self.frame_process_times.pop(0)

        avg_time = sum(self.frame_process_times) / len(self.frame_process_times)
        if avg_time > self.config.max_process_time_s and not self.disable_sharpen:
            self.disable_sharpen = True
            log_warning("DegradeManager", "Disabling sharpening due to performance issues")
            return True
        return self.disable_sharpen

    def should_sharpen(self) -> bool:
        return self.config.enhance_sharpen and not self.disable_sharpen

    def check_memory(self) -> bool:
        now = time.time()
        if now - self.last_memory_check < self.config.memory_check_interval_s:
            return False

        self.last_memory_check = now
        process = psutil.Process()
        memory_mb = process.memory_info().rss / (1024 * 1024)

        if memory_mb > self.config.max_memory_mb:
            if not self.memory_warned:
                log_warning("DegradeManager", f"Memory usage {memory_mb:.1f}MB exceeds threshold")
                self.memory_warned = True
            return True

        return False

    def suggest_output_width_reduction(self) -> float:
        process = psutil.Process()
        memory_mb = process.memory_info().rss / (1024 * 1024)
        if memory_mb > self.config.max_memory_mb:
            excess_ratio = (memory_mb - self.config.max_memory_mb) / memory_mb
            return min(0.2, excess_ratio)
        return 0.0

    def reset(self) -> None:
        self.seam_fail_count = 0
        self.disable_seam = False
        self.frame_process_times = []
        self.disable_sharpen = False
        self.memory_warned = False
