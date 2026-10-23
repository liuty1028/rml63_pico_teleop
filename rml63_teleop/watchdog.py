from dataclasses import dataclass


@dataclass
class TimestampWatchdog:
    timeout_s: float
    last_timestamp_ns: int | None = None
    last_change_time: float | None = None

    def observe(self, timestamp_ns: int, now: float) -> None:
        if int(timestamp_ns) <= 0:
            raise ValueError("XR 时间戳必须为正数")
        if self.last_timestamp_ns is not None and int(timestamp_ns) < self.last_timestamp_ns:
            raise ValueError("XR 时间戳倒退，可能发生数据流重启")
        if self.last_timestamp_ns != int(timestamp_ns):
            self.last_timestamp_ns = int(timestamp_ns)
            self.last_change_time = float(now)

    def healthy(self, now: float) -> bool:
        return self.last_change_time is not None and float(now) - self.last_change_time <= self.timeout_s
