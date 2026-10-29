from __future__ import annotations

import json
import logging
import time
from datetime import datetime
from pathlib import Path

import numpy as np

from .config import validate_config
from .geometry import (
    ClutchMapper,
    Pose,
    QUEST_REFERENCE_ROTATION,
    quat_inverse,
    quat_multiply,
    quat_slerp,
    transform_controller_pose,
)
from .realman_arm import RealManArm
from .safety import SafetyFilter
from .state_machine import TeleopState
from .watchdog import TimestampWatchdog
from .xr_client import XrClient

LOG = logging.getLogger("rml63_pose_teleop")

# rm_movep_canfd is a pass-through API, not a trajectory planner. Keep pose
# increments deliberately small even if XR reports a sudden pose jump.
MAX_LINEAR_SPEED_M_S = 0.05
MAX_ANGULAR_SPEED_RAD_S = float(np.deg2rad(15.0))
MAX_POSITION_LEAD_M = 0.10
MAX_ORIENTATION_LEAD_RAD = float(np.deg2rad(30.0))


class PoseCommandLimiter:
    """Limit Cartesian target speed and reject targets far ahead of feedback."""

    def __init__(self):
        self.last_command: Pose | None = None

    def reset(self, measured: Pose) -> None:
        self.last_command = measured.normalized()

    def update(
        self,
        requested: Pose,
        measured: Pose,
        dt: float,
        check_feedback_lead: bool = True,
    ) -> tuple[Pose, bool]:
        requested = requested.normalized()
        measured = measured.normalized()
        if not np.isfinite(dt) or dt <= 0.0:
            raise ValueError("invalid pose command period")

        position_lead = float(np.linalg.norm(requested.position - measured.position))
        lead_delta = quat_multiply(requested.quaternion, quat_inverse(measured.quaternion))
        orientation_lead = _shortest_quaternion_angle(lead_delta)
        if check_feedback_lead and position_lead > MAX_POSITION_LEAD_M:
            raise ValueError(f"pose target is {position_lead:.3f} m ahead of feedback (limit {MAX_POSITION_LEAD_M:.3f} m)")
        if check_feedback_lead and orientation_lead > MAX_ORIENTATION_LEAD_RAD:
            raise ValueError(
                f"pose target is {np.rad2deg(orientation_lead):.1f}° ahead of feedback "
                f"(limit {np.rad2deg(MAX_ORIENTATION_LEAD_RAD):.1f}°)"
            )

        if self.last_command is None:
            self.reset(measured)
        assert self.last_command is not None
        # Bound increments using the nominal configured cycle, so a delayed
        # scheduler tick cannot turn into a larger one-shot command.
        linear_step = MAX_LINEAR_SPEED_M_S * dt
        delta_position = requested.position - self.last_command.position
        distance = float(np.linalg.norm(delta_position))
        position_limited = distance > linear_step
        if position_limited:
            command_position = self.last_command.position + delta_position * (linear_step / distance)
        else:
            command_position = requested.position.copy()

        delta_quaternion = quat_multiply(requested.quaternion, quat_inverse(self.last_command.quaternion))
        angle = _shortest_quaternion_angle(delta_quaternion)
        angular_step = MAX_ANGULAR_SPEED_RAD_S * dt
        orientation_limited = angle > angular_step
        if orientation_limited and angle > 1e-12:
            command_quaternion = quat_slerp(self.last_command.quaternion, requested.quaternion, angular_step / angle)
        else:
            command_quaternion = requested.quaternion.copy()
        command = Pose(command_position, command_quaternion).normalized()
        self.last_command = command
        return command, position_limited or orientation_limited


def _shortest_quaternion_angle(quaternion: np.ndarray) -> float:
    q = np.asarray(quaternion, dtype=float).reshape(4)
    if q[0] < 0.0:
        q = -q
    return 2.0 * float(np.arccos(np.clip(q[0], -1.0, 1.0)))


def controller_pose_to_pose(raw_pose: object) -> Pose:
    """Parse SDK state pose as [x,y,z,rx,ry,rz] or a nested rm_pose_t dict."""
    if isinstance(raw_pose, (list, tuple, np.ndarray)):
        values = np.asarray(raw_pose, dtype=float).reshape(-1)
        if not np.all(np.isfinite(values)):
            raise ValueError("Controller returned a non-finite pose")
        if values.size == 6:
            p = values[:3]
            q = euler_xyz_to_quaternion(values[3:])
            return Pose(p, q).normalized()
        if values.size == 7:
            return Pose(values[:3], values[3:]).normalized()
        raise ValueError(f"Controller returned a pose list of unsupported length {values.size}")

    if not isinstance(raw_pose, dict):
        raise ValueError(f"Controller returned an invalid pose structure: {raw_pose!r}")
    position = raw_pose.get("position")
    quaternion = raw_pose.get("quaternion")
    if not isinstance(position, dict) or not isinstance(quaternion, dict):
        raise ValueError("Controller pose must contain position and quaternion dictionaries")
    p = np.array([position.get("x"), position.get("y"), position.get("z")], dtype=float)
    q = np.array([quaternion.get("w"), quaternion.get("x"), quaternion.get("y"), quaternion.get("z")], dtype=float)
    return Pose(p, q).normalized()


def euler_xyz_to_quaternion(euler_rx_ry_rz: np.ndarray) -> np.ndarray:
    """Convert RealMan [rx, ry, rz] radians (roll, pitch, yaw) to [w,x,y,z]."""
    rx, ry, rz = np.asarray(euler_rx_ry_rz, dtype=float).reshape(3)
    if not np.all(np.isfinite([rx, ry, rz])):
        raise ValueError("Controller returned non-finite Euler angles")
    cr, sr = np.cos(rx / 2.0), np.sin(rx / 2.0)
    cp, sp = np.cos(ry / 2.0), np.sin(ry / 2.0)
    cy, sy = np.cos(rz / 2.0), np.sin(rz / 2.0)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


class PoseTeleopRuntime:
    """XR teleoperation using RealMan Cartesian pose CANFD pass-through."""

    def __init__(self, config: dict, dry_run: bool):
        self.config = config
        self.dry_run = dry_run
        robot_cfg = config["robot"]
        self.robot = RealManArm(
            robot_cfg.get("ip"),
            robot_cfg.get("port", 8080),
            robot_cfg.get("dof", 6),
            robot_cfg.get("controller_model", "RML63III-6FB"),
        )
        mount_cfg = config.get("mount", {})
        axis_map = mount_cfg.get("xr_to_base_rotation") or QUEST_REFERENCE_ROTATION.tolist()
        teleop_cfg = config["teleop"]
        self.mapper = ClutchMapper(
            np.asarray(axis_map, dtype=float),
            teleop_cfg["scale_factor"],
            teleop_cfg.get("rotation_deadband_deg", 1.0),
            teleop_cfg.get("rotation_smoothing_alpha", 0.25),
        )
        self.xr: XrClient | None = None
        self.state = TeleopState.READY
        self.pose_limiter = PoseCommandLimiter()

    def run(self, enable_motion: bool = False) -> None:
        if enable_motion and self.dry_run:
            raise RuntimeError("干运行实例禁止启用真机运动")
        if enable_motion:
            issues = validate_config(self.config, require_motion_ready=True)
            if issues:
                raise RuntimeError("Motion is locked until configuration is complete:\n- " + "\n- ".join(issues))

        motion_confirmed = False
        try:
            robot_info = self.robot.connect()
            LOG.info("Connected to %s:%s; API software info: %s", self.robot.ip, self.robot.port, robot_info)
            if enable_motion:
                execution_status = self.robot.get_execution_status()
                LOG.info("Controller execution status: %s", execution_status)
                self._require_real_enabled_arm(execution_status, int(self.config["robot"].get("dof", 6)))
            lower, upper = self.robot.get_joint_limits_rad()
            safety = SafetyFilter(self.config["safety"], lower, upper)
            initial = self.robot.get_state()
            current_q = initial.joint_rad
            current_pose = controller_pose_to_pose(initial.pose)
            self.pose_limiter.reset(current_pose)
            self._check_current_joints(current_q, lower, upper)
            LOG.info("当前控制器位姿（米,wxyz）：p=%s q=%s", np.round(current_pose.position, 5).tolist(), np.round(current_pose.quaternion, 5).tolist())
            LOG.info("当前控制器关节角（度）：%s", np.round(initial.joint_deg, 3).tolist())
            LOG.info("Command mode: RealMan Cartesian pose pass-through; host-side IK is disabled")

            if enable_motion:
                print("\n*** REAL ROBOT POSE PASS-THROUGH ENABLED ***")
                print(f"Robot: {self.robot.ip}:{self.robot.port}")
                print(f"Controller model: {robot_info.get('product_version')}")
                print(f"Current joints (deg): {np.round(initial.joint_deg, 3).tolist()}")
                workspace = self.config["safety"]["workspace"]
                print(
                    "Workspace bounds (m): "
                    + " ".join(
                        f"{axis}=[{workspace[axis]['min']}, {workspace[axis]['max']}]"
                        for axis in ("x", "y", "z")
                    )
                )
                print("Pose target is interpreted in the controller's active work/tool frames.")
                print(
                    f"Cartesian caps: {MAX_LINEAR_SPEED_M_S:.3f} m/s, "
                    f"{np.rad2deg(MAX_ANGULAR_SPEED_RAD_S):.1f} deg/s; "
                    f"feedback lead limit: {MAX_POSITION_LEAD_M:.2f} m / "
                    f"{np.rad2deg(MAX_ORIENTATION_LEAD_RAD):.0f} deg"
                )
                print("Keep the physical E-stop within reach. Grip release holds the measured pose.")
                answer = input("Type ENABLE to continue: ").strip()
                if answer != "ENABLE":
                    LOG.info("Operator did not enable pose motion")
                    return
                motion_confirmed = True

            if enable_motion:
                start_check = safety.check_pose(current_pose, require_workspace=True)
                if not start_check.accepted:
                    raise RuntimeError(f"启动控制器位姿不在配置工作空间内：{start_check.reason}")

            startup_timeout_s = float(self.config["safety"].get("xr_startup_timeout_s", 15.0))
            self.xr = XrClient()
            LOG.info("等待 PICO 右手柄有效姿态和更新中的时间戳，最长 %.1f 秒", startup_timeout_s)
            startup_frame = self.xr.wait_until_ready(
                timeout_s=startup_timeout_s,
                on_wait=lambda reason: LOG.info("等待 XR 数据：%s", reason),
                release_grip_threshold=float(self.config["teleop"]["grip_threshold"]) if enable_motion else None,
                release_trigger_threshold=float(self.config["gripper"]["deadband"]),
            )
            xr_ready_time = time.monotonic()
            LOG.info("XR 输入已就绪，开始%s", "位姿透传真机遥操作" if enable_motion else "位姿透传干运行")

            # Re-read after operator confirmation and XR startup waiting.
            if enable_motion:
                execution_status = self.robot.get_execution_status()
                LOG.info("XR 就绪后的控制器状态复查：%s", execution_status)
                self._require_real_enabled_arm(execution_status, int(self.config["robot"].get("dof", 6)))
            initial = self.robot.get_state()
            current_q = initial.joint_rad
            current_pose = controller_pose_to_pose(initial.pose)
            self.pose_limiter.reset(current_pose)
            self._check_current_joints(current_q, lower, upper)
            start_check = safety.check_pose(current_pose, require_workspace=enable_motion)
            if not start_check.accepted:
                raise RuntimeError(f"启动控制器位姿检查失败：{start_check.reason}")

            log_dir = Path(__file__).resolve().parents[1] / "logs"
            log_dir.mkdir(parents=True, exist_ok=True)
            log_path = log_dir / f"pose_teleop_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jsonl"
            log_file = log_path.open("a", encoding="utf-8")
            LOG.info("Writing pose telemetry to %s", log_path)

            rate = float(self.config["teleop"]["control_rate_hz"])
            period = 1.0 / rate
            grip_threshold = float(self.config["teleop"]["grip_threshold"])
            timeout_s = float(self.config["safety"]["xr_timeout_ms"]) / 1000.0
            gripper_cfg = self.config["gripper"]
            previous_trigger: float | None = None
            previous_a = startup_frame.right_a
            previous_b = startup_frame.right_b
            last_gripper_time = 0.0
            watchdog = TimestampWatchdog(timeout_s)
            watchdog.observe(startup_frame.timestamp_ns, xr_ready_time)
            last_log_time = 0.0
            previous_tick = time.monotonic()
            arm_commands_sent = 0
            gripper_commands_sent = 0
            last_pose_return_code = None
            last_gripper_return_code = None
            hold_latched = False

            while True:
                tick_start = time.monotonic()
                dt = max(tick_start - previous_tick, 1e-4)
                previous_tick = tick_start
                try:
                    xr_frame = self.xr.read()
                    watchdog.observe(xr_frame.timestamp_ns, tick_start)
                    robot_state = self.robot.get_state()
                except Exception as exc:
                    self.state = TeleopState.FAULT
                    LOG.exception("Input/state read failed; entering FAULT: %s", exc)
                    if enable_motion:
                        self._best_effort_stop()
                    break

                current_q = robot_state.joint_rad
                current_pose = controller_pose_to_pose(robot_state.pose)
                if not watchdog.healthy(time.monotonic()):
                    self.state = TeleopState.FAULT
                    LOG.error("XR watchdog timed out; entering FAULT")
                    if enable_motion:
                        self._best_effort_stop()
                    break
                try:
                    self._check_current_joints(current_q, lower, upper)
                except ValueError as exc:
                    self.state = TeleopState.FAULT
                    LOG.error("Current robot state failed controller joint-limit check: %s", exc)
                    if enable_motion:
                        self._best_effort_stop()
                    break

                grip_active = xr_frame.right_grip >= grip_threshold
                transformed_xyz, transformed_quat = transform_controller_pose(xr_frame.right_pose, self.mapper.rotation)
                was_mapper_active = self.mapper.active
                requested_pose = None
                if hold_latched:
                    if not grip_active:
                        hold_latched = False
                        self.state = TeleopState.HOLD
                        self.mapper.release()
                    else:
                        self.mapper.release()
                else:
                    requested_pose = self.mapper.update(xr_frame.right_pose, grip_active, current_pose)
                clutch_engaged = requested_pose is not None and not was_mapper_active
                target_pose = None
                pose_return_code = None
                gripper_return_code = None
                pose_status = "inactive"

                if requested_pose is None:
                    self.pose_limiter.reset(current_pose)
                    if self.state == TeleopState.ACTIVE:
                        self.state = TeleopState.HOLD
                        self.mapper.release()
                    elif self.state == TeleopState.READY:
                        self.state = TeleopState.HOLD
                    if enable_motion and not hold_latched:
                        try:
                            self._require_fresh_command(watchdog, tick_start, timeout_s)
                            pose_return_code = self.robot.servo_pose(current_pose.position, current_pose.quaternion)
                            last_pose_return_code = pose_return_code
                            arm_commands_sent += 1
                            pose_status = "holding_measured_pose"
                        except Exception as exc:
                            self.state = TeleopState.FAULT
                            LOG.exception("Could not send measured-pose hold: %s", exc)
                            self._best_effort_stop()
                            break
                    elif hold_latched:
                        pose_status = "protective_hold_latched"
                elif requested_pose is not None:
                    self.state = TeleopState.ACTIVE
                    pose_check = safety.check_pose(requested_pose, require_workspace=enable_motion)
                    pose_status = pose_check.reason or ("accepted" if pose_check.accepted else "rejected")
                    if not pose_check.accepted:
                        hold_latched = True
                        self.state = TeleopState.HOLD
                        self.mapper.release()
                        pose_status = pose_check.reason
                        if enable_motion:
                            self._best_effort_stop()
                    elif clutch_engaged:
                        target_pose = current_pose
                        self.pose_limiter.reset(current_pose)
                        pose_status = "clutch_reference"
                    else:
                        try:
                            # A delayed loop must not permit a larger pass-through step.
                            target_pose, rate_limited = self.pose_limiter.update(
                                requested_pose,
                                current_pose,
                                min(dt, period),
                                check_feedback_lead=enable_motion,
                            )
                            filtered_check = safety.check_pose(target_pose, require_workspace=enable_motion)
                            if not filtered_check.accepted:
                                raise ValueError(filtered_check.reason)
                        except ValueError as exc:
                            hold_latched = True
                            self.state = TeleopState.HOLD
                            self.mapper.release()
                            self.pose_limiter.reset(current_pose)
                            pose_status = str(exc)
                            if enable_motion:
                                self._best_effort_stop()
                        else:
                            pose_status = "rate_limited" if rate_limited else "accepted"
                        if enable_motion and not hold_latched and not clutch_engaged:
                            try:
                                self._require_fresh_command(watchdog, tick_start, timeout_s)
                                pose_return_code = self.robot.servo_pose(target_pose.position, target_pose.quaternion)
                                last_pose_return_code = pose_return_code
                                arm_commands_sent += 1
                            except Exception as exc:
                                self.state = TeleopState.FAULT
                                LOG.exception("Cartesian pose pass-through failed: %s", exc)
                                self._best_effort_stop()
                                break

                gripper_position = int(round(
                    gripper_cfg["open_position"]
                    + xr_frame.right_trigger * (gripper_cfg["close_position"] - gripper_cfg["open_position"])
                ))
                a_pressed = xr_frame.right_a and not previous_a
                b_pressed = xr_frame.right_b and not previous_b
                previous_a = xr_frame.right_a
                previous_b = xr_frame.right_b
                gripper_action = None
                button_position = None
                if a_pressed and not xr_frame.right_b:
                    gripper_action = "open"
                    button_position = int(gripper_cfg["open_position"])
                elif b_pressed and not xr_frame.right_a:
                    gripper_action = "close"
                    button_position = int(gripper_cfg["close_position"])
                elif a_pressed and b_pressed:
                    gripper_action = "conflict_ignored"
                if previous_trigger is None:
                    previous_trigger = xr_frame.right_trigger
                trigger_changed = abs(xr_frame.right_trigger - previous_trigger) >= float(gripper_cfg["deadband"])
                gripper_rate_due = tick_start - last_gripper_time >= 1.0 / float(gripper_cfg["command_rate_hz"])
                if gripper_cfg.get("enabled", True) and not hold_latched and (button_position is not None or (trigger_changed and gripper_rate_due)):
                    if button_position is None:
                        button_position = gripper_position
                    previous_trigger = xr_frame.right_trigger
                    last_gripper_time = tick_start
                    if enable_motion:
                        try:
                            self._require_fresh_command(watchdog, tick_start, timeout_s)
                            gripper_return_code = self.robot.set_gripper_position(button_position)
                            last_gripper_return_code = gripper_return_code
                            gripper_commands_sent += 1
                        except Exception as exc:
                            self.state = TeleopState.FAULT
                            LOG.exception("Gripper command failed: %s", exc)
                            self._best_effort_stop()
                            break

                if tick_start - last_log_time >= 1.0 / float(self.config["debug"]["log_rate_hz"]):
                    pose_error_position = None if target_pose is None else (target_pose.position - current_pose.position).tolist()
                    pose_error_angle = None
                    if target_pose is not None:
                        dq = quat_multiply(target_pose.quaternion, quat_inverse(current_pose.quaternion))
                        pose_error_angle = float(np.rad2deg(2.0 * np.arccos(np.clip(abs(dq[0]), 0.0, 1.0))))
                    entry = {
                        "timestamp_ns": xr_frame.timestamp_ns,
                        "mode": "pose_motion" if enable_motion else "pose_dry_run",
                        "motion_enabled": enable_motion,
                        "state": self.state.value,
                        "hold_latched": hold_latched,
                        "grip": xr_frame.right_grip,
                        "trigger": xr_frame.right_trigger,
                        "right_a": xr_frame.right_a,
                        "right_b": xr_frame.right_b,
                        "gripper_button_action": gripper_action,
                        "xr_pose_raw": xr_frame.right_pose.tolist(),
                        "xr_position_base_m": transformed_xyz.tolist(),
                        "xr_quaternion_base_wxyz": transformed_quat.tolist(),
                        "ee_target_position_m": None if target_pose is None else target_pose.position.tolist(),
                        "ee_target_quaternion_wxyz": None if target_pose is None else target_pose.quaternion.tolist(),
                        "ee_current_position_m": current_pose.position.tolist(),
                        "ee_current_quaternion_wxyz": current_pose.quaternion.tolist(),
                        "pose_error_position_m": pose_error_position,
                        "pose_error_angle_deg": pose_error_angle,
                        "q_current_deg": robot_state.joint_deg.tolist(),
                        "gripper_position": gripper_position,
                        "pose_status": pose_status,
                        "pose_return_code": pose_return_code,
                        "gripper_return_code": gripper_return_code,
                        "arm_commands_sent": arm_commands_sent,
                        "gripper_commands_sent": gripper_commands_sent,
                        "last_pose_return_code": last_pose_return_code,
                        "last_gripper_return_code": last_gripper_return_code,
                        "control_dt_s": dt,
                    }
                    log_file.write(json.dumps(entry, separators=(",", ":")) + "\n")
                    log_file.flush()
                    LOG.info(
                        "mode=%s state=%s grip=%.2f trigger=%.2f\n"
                        "  xr_base_m=%s\n"
                        "  ee_current_m=%s ee_target_m=%s position_error_m=%s rotation_error_deg=%s\n"
                        "  q_deg=%s\n"
                        "  pose_status=%s hold_latched=%s\n"
                        "  pose_tx=%d last_pose_rc=%s gripper_tx=%d last_gripper_rc=%s gripper_position=%d",
                        "MOTION" if enable_motion else "DRY_RUN",
                        self.state.value,
                        xr_frame.right_grip,
                        xr_frame.right_trigger,
                        np.round(transformed_xyz, 3).tolist(),
                        np.round(current_pose.position, 3).tolist(),
                        None if target_pose is None else np.round(target_pose.position, 3).tolist(),
                        None if pose_error_position is None else np.round(pose_error_position, 3).tolist(),
                        None if pose_error_angle is None else round(pose_error_angle, 2),
                        np.round(robot_state.joint_deg, 2).tolist(),
                        pose_status,
                        hold_latched,
                        arm_commands_sent,
                        last_pose_return_code,
                        gripper_commands_sent,
                        last_gripper_return_code,
                        gripper_position,
                    )
                    last_log_time = tick_start

                sleep_for = period - (time.monotonic() - tick_start)
                if sleep_for > 0:
                    time.sleep(sleep_for)
        except KeyboardInterrupt:
            self.state = TeleopState.HOLD
            LOG.info("Keyboard interrupt; stopping pose teleoperation")
        except Exception:
            self.state = TeleopState.FAULT
            if enable_motion and motion_confirmed:
                self._best_effort_stop()
            raise
        finally:
            if enable_motion and motion_confirmed:
                self._best_effort_stop()
            if "arm_commands_sent" in locals():
                LOG.info(
                    "位姿透传运行汇总：mode=%s pose_tx=%d gripper_tx=%d 最近位姿返回码=%s 最近夹爪返回码=%s",
                    "MOTION" if enable_motion else "DRY_RUN",
                    arm_commands_sent,
                    gripper_commands_sent,
                    last_pose_return_code,
                    last_gripper_return_code,
                )
            if "log_file" in locals():
                log_file.close()
            if self.xr is not None:
                try:
                    self.xr.close()
                finally:
                    self.robot.disconnect()
            else:
                self.robot.disconnect()

    @staticmethod
    def _check_current_joints(current: np.ndarray, lower: np.ndarray, upper: np.ndarray) -> None:
        q = np.asarray(current, dtype=float).reshape(-1)
        if q.shape != lower.shape or not np.all(np.isfinite(q)):
            raise ValueError("invalid current joint state")
        if np.any(q < lower) or np.any(q > upper):
            axis = int(np.argmax(np.maximum(lower - q, q - upper)))
            raise ValueError(
                f"J{axis + 1}={np.rad2deg(q[axis]):.3f}° outside controller limits "
                f"[{np.rad2deg(lower[axis]):.3f}°, {np.rad2deg(upper[axis]):.3f}°]"
            )

    @staticmethod
    def _require_real_enabled_arm(status: dict, dof: int) -> None:
        """Fail closed unless read-only controller checks report real mode and all axes enabled."""
        run_mode = status.get("run_mode", {})
        joint_enable = status.get("joint_enable", {})
        if run_mode.get("return_code") != 0 or joint_enable.get("return_code") != 0:
            raise RuntimeError(
                "无法确认机械臂真实模式/关节使能状态；"
                f"只读查询结果：run_mode={run_mode}, joint_enable={joint_enable}"
            )
        try:
            mode = int(run_mode.get("value"))
            enabled = [int(value) for value in joint_enable.get("value", [])]
        except (TypeError, ValueError):
            raise RuntimeError(f"控制器运行状态格式无效：{status}") from None
        if mode != 1:
            raise RuntimeError(f"控制器当前不是实机模式（run_mode={mode}，1 才表示真实模式）")
        if len(enabled) != dof or any(value != 1 for value in enabled):
            raise RuntimeError(f"机械臂关节未全部使能：期望 {dof} 轴均为 1，实际为 {enabled}")

    @staticmethod
    def _require_fresh_command(watchdog: TimestampWatchdog, tick_start: float, timeout_s: float) -> None:
        now = time.monotonic()
        if not watchdog.healthy(now) or now - tick_start > timeout_s:
            raise RuntimeError("控制计算或通信耗时过长，禁止下发过期动作目标")

    def _best_effort_stop(self) -> None:
        try:
            code = self.robot.slow_stop()
            LOG.warning("RealMan slow-stop returned %s", code)
        except Exception:
            LOG.exception("Best-effort slow-stop failed; use the physical E-stop if motion continues")
