from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
import yaml

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PROJECT_CONFIG_PATH = PACKAGE_ROOT / "config" / "teleop.yaml"
DEFAULT_CONFIG_PATH = PROJECT_CONFIG_PATH if PROJECT_CONFIG_PATH.exists() else Path(__file__).resolve().parent / "defaults" / "teleop.yaml"


def load_config(path: str | Path = DEFAULT_CONFIG_PATH) -> dict[str, Any]:
    with Path(path).expanduser().open("r", encoding="utf-8") as f:
        config = yaml.safe_load(f)
    if not isinstance(config, dict):
        raise ValueError(f"Configuration must be a YAML mapping: {path}")
    return config


def validate_config(config: dict[str, Any], require_motion_ready: bool = False) -> list[str]:
    issues: list[str] = []
    sections = {}
    for name in ("robot", "teleop", "mount", "canfd", "gripper", "safety", "debug"):
        value = config.get(name, {})
        if not isinstance(value, dict):
            issues.append(f"{name} must be a mapping")
            value = {}
        sections[name] = value

    def number(section, key, default=None, *, positive=True, integer=False, maximum=None):
        raw = sections[section].get(key, default)
        path = f"{section}.{key}"
        try:
            value = float(raw)
            valid = not isinstance(raw, bool) and np.isfinite(value)
            valid = valid and (not positive or value > 0)
            valid = valid and (not integer or value.is_integer())
            valid = valid and (maximum is None or value <= maximum)
        except (TypeError, ValueError, OverflowError):
            valid = False
        if not valid:
            issues.append(f"{path} must be a valid finite {'positive ' if positive else ''}{'integer' if integer else 'number'}")
            return None
        return value

    robot = sections["robot"]
    if not isinstance(robot.get("ip"), str) or not robot["ip"].strip():
        issues.append("robot.ip is not configured")
    if robot.get("model", "RML63II-6F-V") != "RML63II-6F-V":
        issues.append("robot.model must match the bundled RML63II-6F-V kinematics model")
    controller_model = robot.get("controller_model", "RML63III-6FB")
    supported_controller_models = {
        "RML63II-6F-V",
        "RML63III-6FB",
    }
    if not isinstance(controller_model, str) or controller_model not in supported_controller_models:
        issues.append("robot.controller_model must match the controller product_version")
    if robot.get("dof", 6) != 6:
        issues.append("robot.dof must be 6")
    number("robot", "port", 8080, integer=True, maximum=65535)

    mount = sections["mount"]
    if mount.get("calibration_confirmed") is not True:
        issues.append("mount.calibration_confirmed must be true after verifying XR-to-base mapping")
    rotation = mount.get("xr_to_base_rotation")
    if rotation is None:
        issues.append("mount.xr_to_base_rotation is not configured")
    else:
        try:
            matrix = np.asarray(rotation, dtype=float)
            determinant = float(np.linalg.det(matrix)) if matrix.shape == (3, 3) else float("nan")
            if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)) or not np.allclose(matrix.T @ matrix, np.eye(3), atol=1e-3) or not np.isclose(abs(determinant), 1.0, atol=1e-3):
                issues.append("mount.xr_to_base_rotation must be a finite orthogonal axis map with determinant +1 or -1")
        except (TypeError, ValueError):
            issues.append("mount.xr_to_base_rotation must be a 3x3 matrix")

    safety = sections["safety"]
    for key, default in (("xr_startup_timeout_s", 15.0), ("xr_timeout_ms", None),
                         ("max_joint_step_deg", None), ("max_joint_velocity_deg_s", None),
                         ("max_joint_tracking_error_deg", 12.0),
                         ("tracking_error_slowdown_start_deg", 5.0),
                         ("tracking_error_stop_duration_ms", 250.0)):
        number("safety", key, default)
    if type(safety.get("tracking_error_protection_enabled", True)) is not bool:
        issues.append("safety.tracking_error_protection_enabled must be a boolean")
    try:
        slowdown_start = float(safety.get("tracking_error_slowdown_start_deg", 5.0))
        stop_error = float(safety.get("max_joint_tracking_error_deg", 12.0))
        if np.isfinite(slowdown_start) and np.isfinite(stop_error) and slowdown_start >= stop_error:
            issues.append("safety.tracking_error_slowdown_start_deg must be below max_joint_tracking_error_deg")
    except (TypeError, ValueError, OverflowError):
        pass
    number("safety", "ik_failure_limit", integer=True)
    if sections["canfd"].get("follow", False) is not False:
        issues.append("CANFD high-follow is disabled in this first version; set canfd.follow: false")
    for key in ("trajectory_mode", "radio"):
        if sections["canfd"].get(key, 0) != 0:
            issues.append(f"canfd.{key} is fixed to 0 in the current adapter")
    teleop = sections["teleop"]
    for key, expected in (("controller", "right_controller"), ("clutch", "right_grip"), ("gripper_trigger", "right_trigger")):
        if teleop.get(key, expected) != expected:
            issues.append(f"teleop.{key} is fixed to {expected}")
    number("teleop", "control_rate_hz")
    number("teleop", "grip_threshold", maximum=1)
    number("teleop", "scale_factor")
    number("teleop", "rotation_deadband_deg", 1.0, positive=False)
    number("teleop", "rotation_smoothing_alpha", 0.25, maximum=1.0)
    gripper = sections["gripper"]
    if type(gripper.get("enabled", True)) is not bool:
        issues.append("gripper.enabled must be a boolean")
    for key in ("open_position", "close_position"):
        number("gripper", key, integer=True, maximum=1000)
    number("gripper", "deadband", maximum=1)
    number("gripper", "command_rate_hz")
    number("debug", "log_rate_hz")
    workspace = safety.get("workspace", {})
    if not isinstance(workspace, dict):
        issues.append("safety.workspace must be a mapping")
        workspace = {}
    axes = ("x", "y", "z")
    for axis in axes:
        bounds = workspace.get(axis, {})
        if not isinstance(bounds, dict):
            issues.append(f"safety.workspace.{axis} must be a mapping")
            continue
        if bounds.get("min") is None or bounds.get("max") is None:
            issues.append(f"safety.workspace.{axis} min/max are required before motion")
        else:
            try:
                lower, upper = float(bounds["min"]), float(bounds["max"])
                valid = np.isfinite(lower) and np.isfinite(upper) and lower < upper
            except (TypeError, ValueError, OverflowError):
                valid = False
            if not valid:
                issues.append(f"safety.workspace.{axis} requires finite min below max")
    if require_motion_ready and mount.get("emergency_stop_confirmed") is not True:
        issues.append("mount.emergency_stop_confirmed must be true after confirming the physical E-stop is reachable")
    return issues
