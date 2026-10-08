"""One physical schedule shared by capture, transport and scoring."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Mission:
    warmup_s: float = 13

    def __post_init__(self):
        if (
            not math.isfinite(self.warmup_s)
            or not 13 <= self.warmup_s <= 600
            or abs(self.warmup_s * 100 - round(self.warmup_s * 100)) > 1e-8
        ):
            raise ValueError(
                "Warmup must be 13-600 seconds on the 100 Hz reporting grid"
            )

    @property
    def offset_s(self):
        return self.warmup_s - 13

    @property
    def end_s(self):
        return self.warmup_s + 47

    @property
    def loss_s(self):
        return self.warmup_s + 12

    @property
    def return_s(self):
        return self.warmup_s + 27

    @property
    def windows(self):
        return (
            ("before_takeoff", self.warmup_s - 3, self.warmup_s),
            ("flight", self.warmup_s, self.end_s - 0.3),
            ("outage", self.loss_s, self.return_s),
            ("after_return", self.return_s, self.end_s - 0.3),
        )
