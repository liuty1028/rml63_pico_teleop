from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Callable

import numpy as np
import xrobotoolkit_sdk as xrt


@dataclass
class XRInput:
    timestamp_ns: int
    right_pose: np.ndarray
    right_grip: float
    right_trigger: float


class XRTrackingError(ValueError):
    """Raised when the SDK has no valid tracked pose for the controller."""


class XrClient:
    def __init__(self):
        xrt.init()
        self.closed = False

    def read(self) -> XRInput:
        pose = np.asarray(xrt.get_right_controller_pose(), dtype=float).reshape(7)
        grip = float(xrt.get_right_grip())
        trigger = float(xrt.get_right_trigger())
        timestamp = int(xrt.get_time_stamp_ns())
        if not np.all(np.isfinite(pose)) or not np.isfinite(grip) or not np.isfinite(trigger):
            raise XRTrackingError("XR SDK returned non-finite controller data; check headset/controller tracking")
        quaternion_norm = float(np.linalg.norm(pose[3:]))
        if quaternion_norm < 1e-6:
            raise XRTrackingError(
                "XR SDK returned an invalid right-controller pose (zero quaternion); "
                "make sure the PICO is connected, the right controller is awake and tracked, "
                "and XRoboToolkit PC Service is streaming data"
            )
        if not 0.0 <= grip <= 1.0 or not 0.0 <= trigger <= 1.0:
            raise XRTrackingError("XR grip/trigger values are outside [0, 1]; controller tracking may be unavailable")
        if timestamp <= 0:
            raise XRTrackingError("XR 时间戳尚未有效")
        return XRInput(timestamp, pose, grip, trigger)

    def wait_until_ready(
        self,
        timeout_s: float = 15.0,
        poll_interval_s: float = 0.02,
        on_wait: Callable[[str], None] | None = None,
        release_grip_threshold: float | None = None,
        release_trigger_threshold: float = 0.02,
    ) -> XRInput:
        """等待有效姿态与递增时间戳；只在控制循环启动前使用。"""
        if not np.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("XR 启动等待时间必须为有限正数")
        if not np.isfinite(poll_interval_s) or poll_interval_s <= 0:
            raise ValueError("XR 轮询间隔必须为有限正数")
        deadline = time.monotonic() + timeout_s
        previous_timestamp = None
        last_report = float("-inf")
        reason = "尚未收到有效的右手柄数据"
        while time.monotonic() < deadline:
            try:
                frame = self.read()
            except XRTrackingError as exc:
                previous_timestamp = None
                reason = str(exc)
            else:
                if frame.timestamp_ns <= 0:
                    previous_timestamp = None
                    reason = "XR 时间戳尚未有效"
                elif release_grip_threshold is not None and (
                    frame.right_grip >= release_grip_threshold or frame.right_trigger > release_trigger_threshold
                ):
                    previous_timestamp = None
                    reason = "请先松开右手柄 Grip 和扳机，再等待输入就绪"
                elif previous_timestamp is not None and frame.timestamp_ns > previous_timestamp:
                    return frame
                else:
                    previous_timestamp = frame.timestamp_ns
                    reason = "已收到有效姿态，正在确认 XR 时间戳持续更新"
            now = time.monotonic()
            if on_wait is not None and now - last_report >= 1.0:
                on_wait(reason)
                last_report = now
            time.sleep(min(poll_interval_s, max(0.0, deadline - now)))
        raise TimeoutError(
            f"等待 PICO 右手柄有效数据超时（{timeout_s:g} 秒）：{reason}。"
            "请确认 PICO 已连接 XRoboToolkit PC Service、头显未休眠且右手柄已唤醒并处于追踪状态。"
        )

    def close(self) -> None:
        if not self.closed:
            xrt.close()
            self.closed = True
