from dataclasses import dataclass, field
import os


@dataclass
class StitcherConfig:
    base_interval: int = 60
    high_threshold: float = 60.0
    low_threshold: float = 20.0
    min_interval: int = 5
    max_interval: int = 200

    overlap_ratio: float = 0.15
    seamless_blend: bool = True
    fallback_to_center: bool = True

    output_width: int = 600
    enhance_sharpen: bool = True
    sharpen_amount: float = 1.5
    enhance_contrast: bool = True
    clahe_clip: float = 3.5

    max_frames: int = 500
    min_frames_to_process: int = 10
    max_silent_frames: int = 15

    jpeg_quality: int = 97
    segment_height: int = 8000

    max_seam_failures: int = 3
    max_process_time_s: float = 2.0
    max_memory_mb: int = 1536
    memory_check_interval_s: float = 5.0

    _validated: bool = field(default=False, init=False)

    def __post_init__(self):
        self._load_env_vars()
        self.validate()

    def _load_env_vars(self):
        env_map = {
            'OPENSTITCH_BASE_INTERVAL': ('base_interval', int),
            'OPENSTITCH_OUTPUT_WIDTH': ('output_width', int),
            'OPENSTITCH_JPEG_QUALITY': ('jpeg_quality', int),
            'OPENSTITCH_MAX_FRAMES': ('max_frames', int),
        }
        for env_name, (attr_name, cast_func) in env_map.items():
            value = os.environ.get(env_name)
            if value is not None:
                try:
                    setattr(self, attr_name, cast_func(value))
                except ValueError:
                    pass

    def validate(self) -> None:
        errors = []

        if not (1 <= self.base_interval <= 500):
            errors.append(f"base_interval must be in [1, 500], got {self.base_interval}")
        if not (0 <= self.high_threshold <= 100):
            errors.append(f"high_threshold must be in [0, 100], got {self.high_threshold}")
        if not (0 <= self.low_threshold <= 100):
            errors.append(f"low_threshold must be in [0, 100], got {self.low_threshold}")
        if self.high_threshold <= self.low_threshold:
            errors.append(f"high_threshold({self.high_threshold}) must be > low_threshold({self.low_threshold})")
        if not (1 <= self.min_interval <= 100):
            errors.append(f"min_interval must be in [1, 100], got {self.min_interval}")
        if not (100 <= self.max_interval <= 1000):
            errors.append(f"max_interval must be in [100, 1000], got {self.max_interval}")
        if self.min_interval >= self.max_interval:
            errors.append(f"min_interval({self.min_interval}) must be < max_interval({self.max_interval})")

        if not (0.01 <= self.overlap_ratio <= 0.5):
            errors.append(f"overlap_ratio must be in [0.01, 0.5], got {self.overlap_ratio}")

        if not (100 <= self.output_width <= 2000):
            errors.append(f"output_width must be in [100, 2000], got {self.output_width}")
        if not (0.5 <= self.sharpen_amount <= 3.0):
            errors.append(f"sharpen_amount must be in [0.5, 3.0], got {self.sharpen_amount}")
        if not (1.0 <= self.clahe_clip <= 10.0):
            errors.append(f"clahe_clip must be in [1.0, 10.0], got {self.clahe_clip}")

        if not (10 <= self.max_frames <= 2000):
            errors.append(f"max_frames must be in [10, 2000], got {self.max_frames}")
        if not (1 <= self.min_frames_to_process <= 100):
            errors.append(f"min_frames_to_process must be in [1, 100], got {self.min_frames_to_process}")
        if not (5 <= self.max_silent_frames <= 50):
            errors.append(f"max_silent_frames must be in [5, 50], got {self.max_silent_frames}")

        if not (1 <= self.jpeg_quality <= 100):
            errors.append(f"jpeg_quality must be in [1, 100], got {self.jpeg_quality}")
        if not (1000 <= self.segment_height <= 50000):
            errors.append(f"segment_height must be in [1000, 50000], got {self.segment_height}")
        if not (1 <= self.max_seam_failures <= 20):
            errors.append(f"max_seam_failures must be in [1, 20], got {self.max_seam_failures}")
        if not (0.1 <= self.max_process_time_s <= 30.0):
            errors.append(f"max_process_time_s must be in [0.1, 30.0], got {self.max_process_time_s}")
        if not (256 <= self.max_memory_mb <= 8192):
            errors.append(f"max_memory_mb must be in [256, 8192], got {self.max_memory_mb}")
        if not (1.0 <= self.memory_check_interval_s <= 60.0):
            errors.append(f"memory_check_interval_s must be in [1.0, 60.0], got {self.memory_check_interval_s}")

        if errors:
            raise ValueError("\n".join(errors))

        self._validated = True

    def copy(self) -> 'StitcherConfig':
        return StitcherConfig(**{k: v for k, v in vars(self).items() if not k.startswith('_')})
