from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .geometry import Pose


@dataclass
class SafetyResult:
    accepted: bool
    joint_rad: np.ndarray | None
    reason: str = ""


class SafetyFilter:
    def __init__(self, config: dict, joint_lower: np.ndarray, joint_upper: np.ndarray):
        self.config = config
        self.lower = np.asarray(joint_lower, dtype=float)
        self.upper = np.asarray(joint_upper, dtype=float)

    def check_current_joint(self, current: np.ndarray) -> SafetyResult:
        q = np.asarray(current, dtype=float).reshape(-1)
        if q.shape != self.lower.shape or not np.all(np.isfinite(q)):
            return SafetyResult(False, None, "invalid current joint state")
        if np.any(q < self.lower) or np.any(q > self.upper):
            return SafetyResult(False, None, "current joint state is outside the URDF limits")
        return SafetyResult(True, q.copy())

    def check_tracking(self, command: np.ndarray, current: np.ndarray) -> SafetyResult:
        command = np.asarray(command, dtype=float).reshape(-1)
        current = np.asarray(current, dtype=float).reshape(-1)
        if command.shape != self.lower.shape or current.shape != self.lower.shape or not np.all(np.isfinite(command)) or not np.all(np.isfinite(current)):
            return SafetyResult(False, None, "invalid joint tracking state")
        limit = np.deg2rad(float(self.config.get("max_joint_tracking_error_deg", 5.0)))
        if np.any(np.abs(command - current) > limit):
            axis = int(np.argmax(np.abs(command - current)))
            return SafetyResult(False, None, (
                "joint command is too far ahead of measured joints: "
                f"J{axis + 1} 目标={np.rad2deg(command[axis]):.3f}°，"
                f"实测={np.rad2deg(current[axis]):.3f}°，"
                f"差={np.rad2deg(abs(command[axis] - current[axis])):.3f}°，"
                f"上限={np.rad2deg(limit):.3f}°"
            ))
        return SafetyResult(True, command.copy())

    def check_pose(self, pose: Pose, require_workspace: bool = True) -> SafetyResult:
        p = np.asarray(pose.position, dtype=float).reshape(3)
        q = np.asarray(pose.quaternion, dtype=float).reshape(4)
        if not np.all(np.isfinite(p)) or not np.all(np.isfinite(q)) or abs(float(np.linalg.norm(q)) - 1.0) > 1e-3:
            return SafetyResult(False, None, "invalid target pose")
        bounds = self.config["workspace"]
        workspace_missing = False
        for i, axis in enumerate(("x", "y", "z")):
            axis_bounds = bounds.get(axis, {})
            if axis_bounds.get("min") is None or axis_bounds.get("max") is None:
                if require_workspace:
                    return SafetyResult(False, None, f"workspace {axis} bounds are not configured")
                workspace_missing = True
                continue
            if p[i] < float(axis_bounds["min"]) or p[i] > float(axis_bounds["max"]):
                return SafetyResult(False, None, f"target outside workspace on {axis}")
        return SafetyResult(True, None, "workspace unconfigured; preview only" if workspace_missing else "")

    def filter_joint(self, candidate: np.ndarray, previous: np.ndarray, dt: float) -> SafetyResult:
        candidate = np.asarray(candidate, dtype=float).reshape(-1)
        previous = np.asarray(previous, dtype=float).reshape(-1)
        if candidate.shape != previous.shape or candidate.shape != self.lower.shape:
            return SafetyResult(False, None, "joint vector shape mismatch")
        if not np.all(np.isfinite(candidate)) or not np.all(np.isfinite(previous)):
            return SafetyResult(False, None, "non-finite joint target")
        if np.any(candidate < self.lower) or np.any(candidate > self.upper):
            return SafetyResult(False, None, "joint limit violation")
        if dt <= 0 or not np.isfinite(dt):
            return SafetyResult(False, None, "invalid control period")
        safety = self.config
        max_step = np.deg2rad(float(safety["max_joint_step_deg"]))
        max_velocity = np.deg2rad(float(safety["max_joint_velocity_deg_s"]))
        delta = candidate - previous
        step_cap = min(max_step, max_velocity * dt)
        filtered = previous + np.clip(delta, -step_cap, step_cap)
        if np.any(filtered < self.lower) or np.any(filtered > self.upper):
            return SafetyResult(False, None, "filtered command exceeds joint limits")
        return SafetyResult(True, filtered)
