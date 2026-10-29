from __future__ import annotations

import logging
import json
import time
from datetime import datetime
from pathlib import Path

import numpy as np

from .config import validate_config
from .geometry import ClutchMapper, QUEST_REFERENCE_ROTATION, transform_controller_pose
from .placo_ik import PlacoIK
from .realman_arm import RealManArm
from .safety import SafetyFilter
from .state_machine import TeleopState
from .watchdog import TimestampWatchdog
from .xr_client import XrClient

LOG = logging.getLogger("rml63_teleop")


class TeleopRuntime:
    def __init__(self, config: dict, dry_run: bool):
        self.config = config
        self.dry_run = dry_run
        robot_cfg = config["robot"]
        self.robot = RealManArm(robot_cfg.get("ip"), robot_cfg.get("port", 8080), robot_cfg.get("dof", 6), robot_cfg.get("controller_model", "RML63III-6FB"))
        urdf_path = Path(__file__).resolve().parent / "assets" / "rml63" / "robot_ik.urdf"
        self.ik = PlacoIK(urdf_path, robot_cfg.get("dof", 6))
        self.ik.solver.dt = 1.0 / float(config["teleop"]["control_rate_hz"])
        self.safety = SafetyFilter(config["safety"], self.ik.lower, self.ik.upper)
        mount_cfg = config.get("mount", {})
        rotation = mount_cfg.get("xr_to_base_rotation") or QUEST_REFERENCE_ROTATION.tolist()
        self.mapper = ClutchMapper(
            np.asarray(rotation, dtype=float),
            config["teleop"]["scale_factor"],
            config["teleop"].get("rotation_deadband_deg", 1.0),
            config["teleop"].get("rotation_smoothing_alpha", 0.25),
        )
        self.xr: XrClient | None = None
        self.state = TeleopState.READY

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
            controller_lower, controller_upper = self.robot.get_joint_limits_rad()
            self.ik.restrict_joint_limits(controller_lower, controller_upper)
            self.safety.lower = self.ik.lower.copy()
            self.safety.upper = self.ik.upper.copy()
            LOG.info("有效关节限位（度）：min=%s max=%s", np.round(np.rad2deg(self.safety.lower), 3).tolist(), np.round(np.rad2deg(self.safety.upper), 3).tolist())
            self.xr = XrClient()
            initial = self.robot.get_state()
            current_q = initial.joint_rad
            LOG.info("Initial joint angles (deg): %s", np.round(initial.joint_deg, 3).tolist())
            LOG.info("Mode: %s", "MOTION ENABLED" if enable_motion else "DRY RUN — NO MOTION COMMANDS")
            if enable_motion:
                print("\n*** REAL ROBOT MOTION ENABLED ***")
                print(f"Robot: {self.robot.ip}:{self.robot.port}")
                print(f"Current joints (deg): {np.round(initial.joint_deg, 3).tolist()}")
                print(
                    f"Scale: {self.config['teleop']['scale_factor']}; "
                    f"rate: {self.config['teleop']['control_rate_hz']} Hz; CANFD follow: false"
                )
                print("Keep the physical E-stop within reach. Grip release holds current position.")
                answer = input("Type ENABLE to continue: ").strip()
                if answer != "ENABLE":
                    LOG.info("Operator did not enable motion")
                    return
                motion_confirmed = True

            startup_timeout_s = float(self.config["safety"].get("xr_startup_timeout_s", 15.0))
            LOG.info("等待 PICO 右手柄有效姿态和更新中的时间戳，最长 %.1f 秒；Ctrl+C 可退出", startup_timeout_s)
            startup_frame = self.xr.wait_until_ready(
                timeout_s=startup_timeout_s,
                on_wait=lambda reason: LOG.info("等待 XR 数据：%s", reason),
                release_grip_threshold=float(self.config["teleop"]["grip_threshold"]) if enable_motion else None,
                release_trigger_threshold=float(self.config["gripper"]["deadband"]),
            )
            xr_ready_time = time.monotonic()
            LOG.info("XR 输入已就绪，开始%s", "真机遥操作" if enable_motion else "干运行（不发送动作命令）")
            # 操作者确认和 XR 等待期间状态可能变化，重新读取以确定控制初始状态。
            current_q = self.robot.get_state().joint_rad
            initial_check = self.safety.check_current_joint(current_q)
            if not initial_check.accepted:
                raise RuntimeError(f"启动关节状态不安全：{initial_check.reason}")
            initial_ee = self.ik.forward(current_q)
            LOG.info("模型工具末端 gripper_eef 的启动位置（米）：%s", np.round(initial_ee.position, 5).tolist())
            if enable_motion:
                initial_pose_check = self.safety.check_pose(initial_ee)
                if not initial_pose_check.accepted:
                    raise RuntimeError(f"启动末端位置不在配置工作空间内：{initial_pose_check.reason}；请核对工具坐标和现场边界")

            log_dir = Path(__file__).resolve().parents[1] / "logs"
            log_dir.mkdir(parents=True, exist_ok=True)
            log_path = log_dir / f"teleop_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jsonl"
            log_file = log_path.open("a", encoding="utf-8")
            LOG.info("Writing telemetry to %s", log_path)
            rate = float(self.config["teleop"]["control_rate_hz"])
            period = 1.0 / rate
            grip_threshold = float(self.config["teleop"]["grip_threshold"])
            timeout_s = float(self.config["safety"]["xr_timeout_ms"]) / 1000.0
            trigger_cfg = self.config["gripper"]
            previous_trigger = None
            previous_a = startup_frame.right_a
            previous_b = startup_frame.right_b
            last_gripper_command_time = 0.0
            xr_watchdog = TimestampWatchdog(timeout_s)
            xr_watchdog.observe(startup_frame.timestamp_ns, xr_ready_time)
            last_log_time = 0.0
            previous_tick = time.monotonic()
            last_q_command = current_q.copy()
            consecutive_ik_failures = 0
            ik_failure_limit = int(self.config["safety"]["ik_failure_limit"])
            hold_latched = False
            tracking_protection_enabled = bool(self.config["safety"].get("tracking_error_protection_enabled", True))
            tracking_error_since: float | None = None
            arm_commands_sent = 0
            gripper_commands_sent = 0
            last_canfd_return_code = None
            last_gripper_return_code = None
            while True:
                tick_start = time.monotonic()
                dt = max(tick_start - previous_tick, 1e-4)
                previous_tick = tick_start
                try:
                    xr_frame = self.xr.read()
                    xr_watchdog.observe(xr_frame.timestamp_ns, tick_start)
                    robot_state = self.robot.get_state()
                except Exception as exc:
                    self.state = TeleopState.FAULT
                    LOG.exception("Input/state read failed; entering FAULT: %s", exc)
                    if enable_motion:
                        self._best_effort_stop()
                    break

                current_q = robot_state.joint_rad
                if not xr_watchdog.healthy(time.monotonic()):
                    self.state = TeleopState.FAULT
                    LOG.error("XR watchdog timed out; entering FAULT")
                    if enable_motion:
                        self._best_effort_stop()
                    break
                current_joint_check = self.safety.check_current_joint(current_q)
                if not current_joint_check.accepted:
                    self.state = TeleopState.FAULT
                    LOG.error("Current robot state failed safety check: %s", current_joint_check.reason)
                    if enable_motion:
                        self._best_effort_stop()
                    break
                current_ee = self.ik.forward(current_q)
                tracking_hold_triggered = False
                tracking_error_deg = float(np.max(np.rad2deg(np.abs(last_q_command - current_q))))
                tracking_scale = 1.0
                if enable_motion and tracking_protection_enabled and not hold_latched:
                    stop_error_deg = float(self.config["safety"].get("max_joint_tracking_error_deg", 12.0))
                    stop_duration_s = float(self.config["safety"].get("tracking_error_stop_duration_ms", 250.0)) / 1000.0
                    if tracking_error_deg > stop_error_deg:
                        if tracking_error_since is None:
                            tracking_error_since = tick_start
                        elif tick_start - tracking_error_since >= stop_duration_s:
                            hold_latched = True
                            tracking_hold_triggered = True
                            self._protective_hold(
                                f"关节跟踪误差持续超限（最大 {tracking_error_deg:.2f}°，阈值 {stop_error_deg:.2f}°）",
                                enable_motion,
                            )
                            tracking_error_since = None
                    else:
                        tracking_error_since = None
                grip_active = xr_frame.right_grip >= grip_threshold
                transformed_xyz, transformed_quat = transform_controller_pose(
                    xr_frame.right_pose, self.mapper.rotation
                )
                ik_status = "inactive"
                safety_status = "not evaluated"
                canfd_return_code = None
                gripper_return_code = None

                if hold_latched:
                    previous_trigger = xr_frame.right_trigger
                    if not grip_active and not tracking_hold_triggered:
                        hold_latched = False
                        self.state = TeleopState.HOLD
                        consecutive_ik_failures = 0
                    target_ee = None
                    clutch_engaged = False
                else:
                    was_active = self.mapper.active
                    target_ee = self.mapper.update(xr_frame.right_pose, grip_active, current_ee)
                    clutch_engaged = target_ee is not None and not was_active
                if target_ee is None:
                    if self.state == TeleopState.ACTIVE:
                        self.state = TeleopState.HOLD
                        self.mapper.release()
                        last_q_command = current_q.copy()
                        if enable_motion:
                            try:
                                self._require_fresh_command(xr_watchdog, tick_start, timeout_s)
                                canfd_return_code = self.robot.servo_joint_rad(current_q, follow=False)
                                last_canfd_return_code = canfd_return_code
                                arm_commands_sent += 1
                            except Exception as exc:
                                self.state = TeleopState.FAULT
                                LOG.exception("Could not establish HOLD target: %s", exc)
                                self._best_effort_stop()
                                break
                    elif self.state == TeleopState.READY:
                        self.state = TeleopState.HOLD
                    last_q_command = current_q.copy()
                    consecutive_ik_failures = 0
                else:
                    self.state = TeleopState.ACTIVE
                    pose_result = self.safety.check_pose(target_ee, require_workspace=enable_motion)
                    safety_status = pose_result.reason or ("accepted" if pose_result.accepted else "rejected")
                    if not pose_result.accepted:
                        hold_latched = True
                        self._protective_hold(pose_result.reason, enable_motion)
                    elif clutch_engaged:
                        # 接合第一帧只捕获参考点，不运行优化器，也不发送关节目标。
                        last_q_command = current_q.copy()
                        ik_status = "clutch_reference"
                    else:
                        try:
                            candidate = self.ik.solve(target_ee, last_q_command)
                        except Exception as exc:
                            consecutive_ik_failures += 1
                            ik_status = "failed"
                            LOG.warning("IK failed (%d/%d); keeping previous safe target: %s", consecutive_ik_failures, ik_failure_limit, exc)
                            if consecutive_ik_failures >= ik_failure_limit:
                                hold_latched = True
                                self._protective_hold("连续 IK 失败", enable_motion)
                        else:
                            consecutive_ik_failures = 0
                            ik_status = "success"
                            joint_result = self.safety.filter_joint(candidate, last_q_command, min(dt, period))
                            safety_status = "accepted" if joint_result.accepted else joint_result.reason
                            if joint_result.accepted and joint_result.joint_rad is not None:
                                candidate_command = joint_result.joint_rad
                                if enable_motion and tracking_protection_enabled:
                                    candidate_command, projected_error_deg, joint_scales = self.safety.apply_tracking_slowdown(
                                        candidate_command, last_q_command, current_q
                                    )
                                    tracking_scale = float(np.min(joint_scales))
                                    if tracking_scale < 0.999:
                                        tracking_error_deg = max(tracking_error_deg, float(np.max(projected_error_deg)))
                                        safety_status = f"tracking slowdown scale={tracking_scale:.3f}"
                                # 关节限速会改变实际发送的末端位置，需对限速后的目标再做边界检查。
                                command_pose_check = self.safety.check_pose(self.ik.forward(candidate_command), require_workspace=enable_motion)
                                if not command_pose_check.accepted:
                                    hold_latched = True
                                    safety_status = command_pose_check.reason
                                    self._protective_hold(command_pose_check.reason, enable_motion)
                                if enable_motion:
                                    if not hold_latched:
                                        try:
                                            self._require_fresh_command(xr_watchdog, tick_start, timeout_s)
                                            canfd_return_code = self.robot.servo_joint_rad(candidate_command, follow=False)
                                            last_canfd_return_code = canfd_return_code
                                            arm_commands_sent += 1
                                        except Exception as exc:
                                            self.state = TeleopState.FAULT
                                            LOG.exception("CANFD command failed: %s", exc)
                                            self._best_effort_stop()
                                            break
                                if not hold_latched:
                                    last_q_command = candidate_command
                            else:
                                hold_latched = True
                                self._protective_hold(joint_result.reason, enable_motion)

                if hold_latched:
                    last_q_command = current_q.copy()
                    previous_trigger = xr_frame.right_trigger
                    tracking_error_since = None

                gripper_position = int(round(trigger_cfg["open_position"] + xr_frame.right_trigger * (trigger_cfg["close_position"] - trigger_cfg["open_position"])))
                a_pressed = xr_frame.right_a and not previous_a
                b_pressed = xr_frame.right_b and not previous_b
                previous_a = xr_frame.right_a
                previous_b = xr_frame.right_b
                gripper_button_action = None
                if a_pressed and not xr_frame.right_b:
                    gripper_button_action = "open"
                    button_position = int(trigger_cfg["open_position"])
                elif b_pressed and not xr_frame.right_a:
                    gripper_button_action = "close"
                    button_position = int(trigger_cfg["close_position"])
                else:
                    button_position = None
                    if a_pressed and b_pressed:
                        gripper_button_action = "conflict_ignored"
                if previous_trigger is None:
                    # Establish a no-motion input baseline at startup.
                    previous_trigger = xr_frame.right_trigger
                trigger_changed = abs(xr_frame.right_trigger - previous_trigger) >= float(trigger_cfg["deadband"])
                gripper_rate_due = tick_start - last_gripper_command_time >= 1.0 / float(trigger_cfg["command_rate_hz"])
                if trigger_cfg.get("enabled", True) and not hold_latched and button_position is not None:
                    # A/B are discrete press-edge commands; releasing either button sends no command.
                    previous_trigger = xr_frame.right_trigger
                    last_gripper_command_time = tick_start
                    gripper_position = button_position
                    if enable_motion:
                        try:
                            self._require_fresh_command(xr_watchdog, tick_start, timeout_s)
                            gripper_return_code = self.robot.set_gripper_position(button_position)
                            last_gripper_return_code = gripper_return_code
                            gripper_commands_sent += 1
                        except Exception as exc:
                            self.state = TeleopState.FAULT
                            LOG.exception("Gripper button command failed: %s", exc)
                            self._best_effort_stop()
                            break
                elif trigger_cfg.get("enabled", True) and not hold_latched and trigger_changed and gripper_rate_due:
                    previous_trigger = xr_frame.right_trigger
                    last_gripper_command_time = tick_start
                    if enable_motion:
                        try:
                            self._require_fresh_command(xr_watchdog, tick_start, timeout_s)
                            gripper_return_code = self.robot.set_gripper_position(gripper_position)
                            last_gripper_return_code = gripper_return_code
                            gripper_commands_sent += 1
                        except Exception as exc:
                            self.state = TeleopState.FAULT
                            LOG.exception("Gripper command failed: %s", exc)
                            self._best_effort_stop()
                            break

                if tick_start - last_log_time >= 1.0 / float(self.config["debug"]["log_rate_hz"]):
                    entry = {
                        "timestamp_ns": xr_frame.timestamp_ns,
                        "mode": "motion" if enable_motion else "dry_run",
                        "motion_enabled": enable_motion,
                        "state": self.state.value,
                        "hold_latched": hold_latched,
                        "gripper_enabled": trigger_cfg.get("enabled", True),
                        "grip": xr_frame.right_grip,
                        "trigger": xr_frame.right_trigger,
                        "right_a": xr_frame.right_a,
                        "right_b": xr_frame.right_b,
                        "gripper_button_action": gripper_button_action,
                        "xr_pose_raw": xr_frame.right_pose.tolist(),
                        "xr_position_base_m": transformed_xyz.tolist(),
                        "xr_quaternion_base_wxyz": transformed_quat.tolist(),
                        "ee_target_position_m": None if target_ee is None else target_ee.position.tolist(),
                        "ee_target_quaternion_wxyz": None if target_ee is None else target_ee.quaternion.tolist(),
                        "ee_current_position_m": current_ee.position.tolist(),
                        "ee_current_quaternion_wxyz": current_ee.quaternion.tolist(),
                        "ee_delta_position_m": None if target_ee is None else (target_ee.position - current_ee.position).tolist(),
                        "q_current_rad": current_q.tolist(),
                        "q_target_rad": last_q_command.tolist(),
                        "q_target_deg": np.rad2deg(last_q_command).tolist(),
                        "tracking_error_max_deg": tracking_error_deg,
                        "tracking_scale": tracking_scale,
                        "gripper_position": gripper_position,
                        "ik_status": ik_status,
                        "safety_status": safety_status,
                        "canfd_return_code": canfd_return_code,
                        "gripper_return_code": gripper_return_code,
                        "arm_commands_sent": arm_commands_sent,
                        "gripper_commands_sent": gripper_commands_sent,
                        "last_canfd_return_code": last_canfd_return_code,
                        "last_gripper_return_code": last_gripper_return_code,
                        "control_dt_s": dt,
                    }
                    log_file.write(json.dumps(entry, separators=(",", ":")) + "\n")
                    log_file.flush()
                    LOG.info(
                        "mode=%s state=%s grip=%.2f trigger=%.2f ee_target=%s q_deg=%s gripper=%d IK=%s safety=%s arm_tx=%d gripper_tx=%d hold_latched=%s",
                        "MOTION" if enable_motion else "DRY_RUN",
                        self.state.value,
                        xr_frame.right_grip,
                        xr_frame.right_trigger,
                        None if target_ee is None else np.round(target_ee.position, 3).tolist(),
                        np.round(np.rad2deg(last_q_command), 2).tolist(),
                        gripper_position,
                        ik_status,
                        safety_status,
                        arm_commands_sent,
                        gripper_commands_sent,
                        hold_latched,
                    )
                    last_log_time = tick_start

                sleep_for = period - (time.monotonic() - tick_start)
                if sleep_for > 0:
                    time.sleep(sleep_for)
        except KeyboardInterrupt:
            self.state = TeleopState.HOLD
            LOG.info("Keyboard interrupt; stopping teleoperation")
        except Exception:
            self.state = TeleopState.FAULT
            if enable_motion and motion_confirmed:
                self._best_effort_stop()
            raise
        finally:
            if enable_motion and motion_confirmed:
                self._best_effort_stop()
            if "arm_commands_sent" in locals():
                LOG.info("本次运行汇总：mode=%s arm_tx=%d gripper_tx=%d 最近机械臂返回码=%s 最近夹爪返回码=%s", "MOTION" if enable_motion else "DRY_RUN", arm_commands_sent, gripper_commands_sent, last_canfd_return_code, last_gripper_return_code)
            if "log_file" in locals():
                log_file.close()
            if self.xr is not None:
                try:
                    self.xr.close()
                finally:
                    self.robot.disconnect()
            else:
                self.robot.disconnect()

    def _best_effort_stop(self) -> None:
        try:
            code = self.robot.slow_stop()
            LOG.warning("RealMan slow-stop returned %s", code)
        except Exception:
            LOG.exception("Best-effort slow-stop failed; use the physical E-stop if motion continues")

    def _protective_hold(self, reason: str, enable_motion: bool) -> None:
        self.state = TeleopState.HOLD
        self.mapper.release()
        LOG.warning("进入保护保持：%s；请松开 Grip 后重新接合", reason)
        if enable_motion:
            self._best_effort_stop()

    @staticmethod
    def _require_fresh_command(watchdog: TimestampWatchdog, tick_start: float, timeout_s: float) -> None:
        now = time.monotonic()
        if not watchdog.healthy(now) or now - tick_start > timeout_s:
            raise RuntimeError("控制计算或通信耗时过长，禁止下发过期动作目标")
