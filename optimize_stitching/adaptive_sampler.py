from dataclasses import dataclass
import time
from typing import List
from .config import StitcherConfig


@dataclass
class SampleRecord:
    frame_index: int
    interval: int
    change_score: float
    decision: str
    timestamp: float


class AdaptiveSampler:
    def __init__(self, config: StitcherConfig):
        self.config = config
        self.records: List[SampleRecord] = []

    def get_next_interval(self, change_score: float) -> int:
        if change_score < 0:
            return self.config.min_interval

        change_score = max(0, min(100, change_score))

        if change_score > self.config.high_threshold:
            interval = int(self.config.base_interval * 0.5)
        elif change_score > self.config.low_threshold:
            interval = self.config.base_interval
        else:
            interval = int(self.config.base_interval * 1.5)

        interval = max(self.config.min_interval, min(interval, self.config.max_interval))

        return interval

    def should_early_stop(self, silent_count: int, processed_count: int) -> bool:
        if processed_count < self.config.min_frames_to_process:
            return False
        if silent_count >= self.config.max_silent_frames:
            return True
        return False

    def add_record(self, frame_index: int, interval: int, change_score: float,
                   decision: str) -> None:
        record = SampleRecord(
            frame_index=frame_index,
            interval=interval,
            change_score=change_score,
            decision=decision,
            timestamp=time.time()
        )
        self.records.append(record)

    def get_summary(self) -> dict:
        if not self.records:
            return {'total': 0}

        dense_count = sum(1 for r in self.records if r.decision == 'dense')
        sparse_count = sum(1 for r in self.records if r.decision == 'sparse')
        normal_count = sum(1 for r in self.records if r.decision == 'normal')

        avg_score = sum(r.change_score for r in self.records) / len(self.records)
        avg_interval = sum(r.interval for r in self.records) / len(self.records)

        return {
            'total': len(self.records),
            'dense': dense_count,
            'sparse': sparse_count,
            'normal': normal_count,
            'avg_change_score': avg_score,
            'avg_interval': avg_interval,
        }
