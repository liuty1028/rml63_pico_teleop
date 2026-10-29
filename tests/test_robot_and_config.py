import unittest
import json
from copy import deepcopy
from unittest.mock import Mock, mock_open, patch

import numpy as np

from rml63_teleop.cli import teleop_main
from rml63_teleop.config import PACKAGE_ROOT, load_config, validate_config
from rml63_teleop.realman_arm import RealManArm
from rml63_teleop.runtime import TeleopRuntime
from rml63_teleop.xr_client import XRInput, XRTrackingError, XrClient


def ready_config():
    config = load_config(PACKAGE_ROOT / "rml63_teleop" / "defaults" / "teleop.yaml")
    config["robot"]["ip"] = "mock-robot"
    config["mount"]["calibration_confirmed"] = True
    config["mount"]["emergency_stop_confirmed"] = True
    config["safety"]["workspace"] = {axis: {"min": -1, "max": 1} for axis in "xyz"}
    return config


class RobotAdapterTests(unittest.TestCase):
    def setUp(self):
        patcher = patch("rml63_teleop.realman_arm.RoboticArm")
        sdk_class = patcher.start()
        self.addCleanup(patcher.stop)
        self.sdk = sdk_class.return_value
        self.sdk.rm_create_robot_arm.return_value = Mock(id=1)
        self.sdk.arm_dof = 6
        self.sdk.rm_get_arm_software_info.return_value = (0, {"product_version": "RML63III-6FB"})
        self.sdk.rm_get_current_arm_state.return_value = (0, {
            "joint": [0, 0, 0, 0, 0, -185.405], "pose": [0] * 6,
            "err": {"err_len": 1, "err": ["0"]},
        })
        self.robot = RealManArm("mock-robot")
        self.robot.connect()

    def test_zero_error_sentinel_is_not_a_fault(self):
        state = self.robot.get_state()
        self.assertAlmostEqual(state.joint_deg[5], -185.405)
        self.assertAlmostEqual(state.joint_rad[5], np.deg2rad(-185.405))

    def test_nonzero_or_malformed_error_list_cannot_pass(self):
        for error in ({"err_len": 1, "err": ["4096"]}, {"err_len": 2, "err": ["0"]},
                      {"err_len": 0, "err": ["4096"]}, None):
            with self.subTest(error=error):
                data = {"joint": [0] * 6, "err": error}
                self.sdk.rm_get_current_arm_state.return_value = (0, data)
                with self.assertRaises(RuntimeError):
                    self.robot.get_state()

    def test_sdk_read_failure_never_returns_zeroed_state_as_valid(self):
        self.sdk.rm_get_current_arm_state.return_value = (-2, {})
        with self.assertRaisesRegex(RuntimeError, "code -2"):
            self.robot.get_state()

    def test_mismatched_robot_model_is_rejected(self):
        self.sdk.rm_get_arm_software_info.return_value = (0, {"product_version": "RM65"})
        with self.assertRaisesRegex(RuntimeError, "型号"):
            self.robot.connect()

    def test_controller_limits_are_read_only_and_converted_to_radians(self):
        lower = [-178, -178, -178, -178, -178, -360]
        upper = [178, 178, 145, 178, 178, 360]
        self.sdk.rm_get_joint_min_pos.return_value = (0, lower)
        self.sdk.rm_get_joint_max_pos.return_value = (0, upper)
        actual_lower, actual_upper = self.robot.get_joint_limits_rad()
        np.testing.assert_allclose(actual_lower, np.deg2rad(lower))
        np.testing.assert_allclose(actual_upper, np.deg2rad(upper))
        self.sdk.rm_set_joint_min_pos.assert_not_called()
        self.sdk.rm_set_joint_max_pos.assert_not_called()

    def test_limit_query_failure_is_not_ignored(self):
        self.sdk.rm_get_joint_min_pos.return_value = (-2, [])
        with self.assertRaises(RuntimeError):
            self.robot.get_joint_limits_rad()

    def test_canfd_uses_degrees_and_fixed_low_follow_parameters(self):
        self.sdk.rm_movej_canfd.return_value = 0
        q = np.array([0.1, -0.2, 0.3, 0.0, 0.0, -3.2])
        self.robot.servo_joint_rad(q)
        args, kwargs = self.sdk.rm_movej_canfd.call_args
        np.testing.assert_allclose(args[0], np.rad2deg(q))
        self.assertEqual(kwargs, {"follow": False, "expand": 0, "trajectory_mode": 0, "radio": 0})
        with self.assertRaises(ValueError):
            self.robot.servo_joint_rad(q, follow=True)

    def test_slow_stop_failure_is_reported(self):
        self.sdk.rm_set_arm_slow_stop.return_value = -2
        with self.assertRaisesRegex(RuntimeError, "physical E-stop"):
            self.robot.slow_stop()

    def test_execution_status_reads_modes_and_enable_without_writing(self):
        self.sdk.rm_get_arm_run_mode.return_value = (0, 1)
        self.sdk.rm_get_joint_en_state.return_value = (0, [1] * 6)
        self.sdk.rm_get_gripper_state.return_value = (0, {"actpos": 1000})
        status = self.robot.get_execution_status()
        self.assertEqual(status["run_mode"], {"return_code": 0, "value": 1})
        self.assertEqual(status["joint_enable"]["value"], [1] * 6)
        self.assertEqual(status["gripper"]["value"]["actpos"], 1000)
        self.sdk.rm_set_arm_run_mode.assert_not_called()
        self.sdk.rm_set_joint_en_state.assert_not_called()
        self.sdk.rm_movej_canfd.assert_not_called()
        self.sdk.rm_set_gripper_position.assert_not_called()

    def test_execution_status_keeps_failed_query_code_visible(self):
        self.sdk.rm_get_arm_run_mode.return_value = (-2, 0)
        self.sdk.rm_get_joint_en_state.return_value = (0, [1] * 6)
        self.sdk.rm_get_gripper_state.return_value = (-2, {})
        status = self.robot.get_execution_status()
        self.assertEqual(status["run_mode"]["return_code"], -2)
        self.assertEqual(status["gripper"]["return_code"], -2)


class ConfigAndEntryTests(unittest.TestCase):
    def test_nonfinite_parameters_and_quoted_booleans_cannot_unlock_motion(self):
        base = ready_config()
        self.assertEqual(validate_config(base, require_motion_ready=True), [])
        for section, key, value in (
            ("teleop", "scale_factor", float("nan")),
            ("teleop", "control_rate_hz", float("inf")),
            ("safety", "max_joint_tracking_error_deg", "bad"),
            ("safety", "ik_failure_limit", 1.5),
            ("mount", "calibration_confirmed", "false"),
            ("mount", "emergency_stop_confirmed", "true"),
            ("canfd", "follow", "false"),
            ("canfd", "trajectory_mode", 1),
            ("gripper", "open_position", 1.5),
            ("gripper", "enabled", "false"),
            ("debug", "log_rate_hz", 0),
        ):
            config = deepcopy(base)
            config[section][key] = value
            with self.subTest(section=section, key=key):
                self.assertTrue(validate_config(config, require_motion_ready=True))

    def test_infinite_workspace_is_rejected(self):
        config = ready_config()
        config["safety"]["workspace"]["x"]["max"] = float("inf")
        self.assertTrue(any("workspace.x" in issue for issue in validate_config(config, True)))

    def test_entry_without_enable_flag_is_dry_run_even_when_confirmations_are_true(self):
        with patch("sys.argv", ["run_teleop.py"]), patch("rml63_teleop.cli.load_config", return_value=ready_config()), \
             patch("rml63_teleop.cli.TeleopRuntime") as runtime, patch("builtins.print"):
            teleop_main()
        self.assertTrue(runtime.call_args.kwargs["dry_run"])
        runtime.return_value.run.assert_called_once_with(enable_motion=False)

    def test_entry_blocks_unconfirmed_motion_before_creating_robot_client(self):
        config = ready_config()
        config["mount"]["calibration_confirmed"] = False
        with patch("sys.argv", ["run_teleop.py", "--enable-motion"]), \
             patch("rml63_teleop.cli.load_config", return_value=config), \
             patch("rml63_teleop.cli.TeleopRuntime") as runtime:
            with self.assertRaises(SystemExit):
                teleop_main()
        runtime.assert_not_called()

    def test_raw_zero_quaternion_is_rejected_before_control_math(self):
        client = object.__new__(XrClient)
        with patch("rml63_teleop.xr_client.xrt.get_right_controller_pose", return_value=np.zeros(7)), \
             patch("rml63_teleop.xr_client.xrt.get_right_grip", return_value=0.0), \
             patch("rml63_teleop.xr_client.xrt.get_right_trigger", return_value=0.0), \
             patch("rml63_teleop.xr_client.xrt.get_time_stamp_ns", return_value=100):
            with self.assertRaisesRegex(XRTrackingError, "zero quaternion"):
                client.read()

    def test_full_runtime_with_real_ik_and_simulated_sdk_only(self):
        # 运行真实 SDK 适配器、URDF、Placo、离合和安全过滤；SDK 与 XR 的外部通信全部模拟。
        config = ready_config()
        joint_deg = np.array([-0.546, -21.167, 78.528, 4.222, 122.58, -185.403])
        original_joint_deg = joint_deg.copy()

        def state():
            return 0, {"joint": joint_deg.tolist(), "pose": [0] * 6, "err": {"err_len": 1, "err": ["0"]}}

        def send(joint, **kwargs):
            joint_deg[:] = joint
            return 0

        def xr_frame(timestamp, x=0.0, grip=0.0, trigger=0.0):
            return XRInput(timestamp, np.array([x, 0, 0, 0, 0, 0, 1.0]), grip, trigger)

        with patch("rml63_teleop.realman_arm.RoboticArm") as sdk_class, \
             patch("rml63_teleop.runtime.XrClient") as xr_class, \
             patch("rml63_teleop.runtime.Path.open", mock_open()) as telemetry_open, \
             patch("builtins.input", return_value="ENABLE"), patch("builtins.print"):
            sdk = sdk_class.return_value
            sdk.arm_dof = 6
            sdk.rm_create_robot_arm.return_value = Mock(id=1)
            sdk.rm_get_arm_software_info.return_value = (0, {"product_version": "RML63III-6FB"})
            sdk.rm_get_joint_min_pos.return_value = (0, [-178, -178, -178, -178, -178, -360])
            sdk.rm_get_joint_max_pos.return_value = (0, [178, 178, 145, 178, 178, 360])
            sdk.rm_get_current_arm_state.side_effect = state
            sdk.rm_movej_canfd.side_effect = send
            sdk.rm_set_gripper_position.return_value = 0
            sdk.rm_set_arm_slow_stop.return_value = 0
            xr = xr_class.return_value
            xr.wait_until_ready.return_value = xr_frame(90)
            xr.read.side_effect = ([xr_frame(100), xr_frame(101, grip=1.0)] +
                                   [xr_frame(102 + i, x=0.005 * (i + 1), grip=1.0) for i in range(5)] +
                                   [xr_frame(107), xr_frame(108, trigger=1.0)] +
                                   [xr_frame(109 + i, trigger=1.0) for i in range(12)] + [KeyboardInterrupt])
            with self.assertLogs("rml63_teleop", level="INFO"):
                runtime = TeleopRuntime(config, dry_run=False)
                runtime.run(enable_motion=True)
        self.assertGreater(sdk.rm_movej_canfd.call_count, 1)
        self.assertGreater(float(np.max(np.abs(joint_deg - original_joint_deg))), 0)
        for call in sdk.rm_movej_canfd.call_args_list:
            self.assertIs(call.kwargs["follow"], False)
            self.assertTrue(np.all(np.abs(np.array(call.args[0]) - original_joint_deg) < 5.0))
        sdk.rm_set_gripper_position.assert_called_once_with(1, block=False, timeout=0)
        sdk.rm_delete_robot_arm.assert_called_once()
        rows = [json.loads(call.args[0]) for call in telemetry_open.return_value.write.call_args_list]
        self.assertTrue(rows)
        for row in rows:
            self.assertEqual(row["mode"], "motion")
            self.assertTrue(row["motion_enabled"])
        self.assertGreater(max(row["arm_commands_sent"] for row in rows), 0)
        self.assertTrue(any(row["last_canfd_return_code"] == 0 for row in rows))


if __name__ == "__main__":
    unittest.main()
