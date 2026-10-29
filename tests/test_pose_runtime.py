import unittest
from unittest.mock import Mock, mock_open, patch

import numpy as np

from rml63_teleop.config import PACKAGE_ROOT, load_config
from rml63_teleop.geometry import ClutchMapper, Pose
from rml63_teleop.pose_runtime import (
    MAX_ANGULAR_SPEED_RAD_S,
    MAX_LINEAR_SPEED_M_S,
    PoseCommandLimiter,
    PoseTeleopRuntime,
    controller_pose_to_pose,
)
from rml63_teleop.realman_arm import RealManArm, RobotState
from rml63_teleop.state_machine import TeleopState
from rml63_teleop.xr_client import XRInput


class PoseRuntimeTests(unittest.TestCase):
    def test_motion_readiness_requires_real_mode_and_all_axes_enabled(self):
        status = {
            "run_mode": {"return_code": 0, "value": 1},
            "joint_enable": {"return_code": 0, "value": [1] * 6},
            "gripper": {"return_code": -2, "value": None},
        }
        PoseTeleopRuntime._require_real_enabled_arm(status, 6)
        status["run_mode"]["value"] = 0
        with self.assertRaisesRegex(RuntimeError, "不是实机模式"):
            PoseTeleopRuntime._require_real_enabled_arm(status, 6)
        status["run_mode"]["value"] = 1
        status["joint_enable"]["value"][-1] = 0
        with self.assertRaisesRegex(RuntimeError, "未全部使能"):
            PoseTeleopRuntime._require_real_enabled_arm(status, 6)

    def test_controller_pose_dict_is_read_as_position_and_wxyz(self):
        pose = controller_pose_to_pose({
            "position": {"x": 0.1, "y": -0.2, "z": 0.3},
            "quaternion": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
        })
        np.testing.assert_allclose(pose.position, [0.1, -0.2, 0.3])
        np.testing.assert_allclose(pose.quaternion, [1.0, 0.0, 0.0, 0.0])

    def test_controller_six_value_pose_converts_realman_euler_angles(self):
        pose = controller_pose_to_pose([0.289716, 0.005562, 0.498139, 3.022, -1.241, -0.223])
        np.testing.assert_allclose(pose.position, [0.289716, 0.005562, 0.498139])
        np.testing.assert_allclose(
            pose.quaternion,
            [0.11289954, 0.80322438, -0.12489700, 0.57138878],
            atol=1e-7,
        )

    def test_pose_command_uses_seven_element_wxyz_sdk_payload(self):
        class FakeSdk:
            def rm_movep_canfd(self, pose, **kwargs):
                self.pose = pose
                self.kwargs = kwargs
                return 0

        arm = RealManArm("127.0.0.1")
        arm.arm = FakeSdk()
        arm.connected = True
        arm.servo_pose(np.array([0.1, 0.2, 0.3]), np.array([2.0, 0.0, 0.0, 0.0]))
        np.testing.assert_allclose(arm.arm.pose, [0.1, 0.2, 0.3, 1.0, 0.0, 0.0, 0.0])
        self.assertEqual(arm.arm.kwargs, {"follow": False, "trajectory_mode": 0, "radio": 0})

    def test_pose_command_limiter_caps_each_nominal_cycle(self):
        measured = Pose(np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]))
        requested = Pose(np.array([0.05, 0.0, 0.0]), np.array([np.cos(0.2), 0.0, 0.0, np.sin(0.2)]))
        limiter = PoseCommandLimiter()
        limiter.reset(measured)
        command, limited = limiter.update(requested, measured, 0.02)
        self.assertTrue(limited)
        self.assertAlmostEqual(np.linalg.norm(command.position), MAX_LINEAR_SPEED_M_S * 0.02)
        self.assertAlmostEqual(2 * np.arccos(command.quaternion[0]), MAX_ANGULAR_SPEED_RAD_S * 0.02)

    def test_pose_command_limiter_rejects_target_too_far_ahead(self):
        measured = Pose(np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]))
        requested = Pose(np.array([0.11, 0.0, 0.0]), np.array([1.0, 0.0, 0.0, 0.0]))
        limiter = PoseCommandLimiter()
        limiter.reset(measured)
        with self.assertRaisesRegex(ValueError, "ahead of feedback"):
            limiter.update(requested, measured, 0.02)

    def test_dry_run_limiter_does_not_compare_preview_with_stationary_robot(self):
        measured = Pose(np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]))
        requested = Pose(np.array([0.2, 0.0, 0.0]), np.array([1.0, 0.0, 0.0, 0.0]))
        limiter = PoseCommandLimiter()
        limiter.reset(measured)
        command, limited = limiter.update(requested, measured, 0.02, check_feedback_lead=False)
        self.assertTrue(limited)
        self.assertAlmostEqual(float(np.linalg.norm(command.position)), MAX_LINEAR_SPEED_M_S * 0.02)

    def test_runtime_sends_no_pose_on_clutch_and_limits_following_target(self):
        config = load_config(PACKAGE_ROOT / "rml63_teleop" / "defaults" / "teleop.yaml")
        config["robot"]["ip"] = "mock-robot"
        config["mount"]["calibration_confirmed"] = True
        config["mount"]["emergency_stop_confirmed"] = True
        config["mount"]["xr_to_base_rotation"] = np.eye(3).tolist()
        config["safety"]["workspace"] = {axis: {"min": -1.0, "max": 1.0} for axis in "xyz"}

        pose_dict = {
            "position": {"x": 0.0, "y": 0.0, "z": 0.0},
            "quaternion": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
        }
        robot = Mock()
        robot.ip = "mock-robot"
        robot.port = 8080
        robot.connect.return_value = {"product_version": "RML63III-6FB"}
        robot.get_execution_status.return_value = {
            "run_mode": {"return_code": 0, "value": 1},
            "joint_enable": {"return_code": 0, "value": [1] * 6},
            "gripper": {"return_code": 0, "value": {}},
        }
        robot.get_joint_limits_rad.return_value = (np.full(6, -6.0), np.full(6, 6.0))
        robot.get_state.return_value = RobotState(np.zeros(6), pose_dict, {})
        robot.servo_pose.return_value = 0
        robot.slow_stop.return_value = 0

        def xr_frame(timestamp, x, grip):
            return XRInput(timestamp, np.array([x, 0, 0, 0, 0, 0, 1.0]), grip, 0.0)

        xr = Mock()
        xr.wait_until_ready.return_value = xr_frame(100, 0.0, 0.0)
        xr.read.side_effect = [xr_frame(101, 0.0, 1.0), xr_frame(102, 0.05, 1.0), KeyboardInterrupt]

        runtime = object.__new__(PoseTeleopRuntime)
        runtime.config = config
        runtime.dry_run = False
        runtime.robot = robot
        runtime.mapper = ClutchMapper(np.eye(3), config["teleop"]["scale_factor"])
        runtime.xr = None
        runtime.state = TeleopState.READY
        runtime.pose_limiter = PoseCommandLimiter()

        with patch("rml63_teleop.pose_runtime.XrClient", return_value=xr), \
             patch("rml63_teleop.pose_runtime.Path.open", mock_open()), \
             patch("builtins.input", return_value="ENABLE"), \
             patch("builtins.print"):
            runtime.run(enable_motion=True)

        robot.servo_pose.assert_called_once()
        sent_position = robot.servo_pose.call_args.args[0]
        self.assertAlmostEqual(float(np.linalg.norm(sent_position)), MAX_LINEAR_SPEED_M_S / config["teleop"]["control_rate_hz"], places=6)
        robot.slow_stop.assert_called_once()
        robot.disconnect.assert_called_once()


if __name__ == "__main__":
    unittest.main()
