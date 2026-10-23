import unittest
import json
from unittest.mock import Mock, mock_open, patch

import numpy as np

from rml63_teleop.config import PACKAGE_ROOT, load_config
from rml63_teleop.geometry import ClutchMapper, Pose
from rml63_teleop.realman_arm import RobotState
from rml63_teleop.runtime import TeleopRuntime
from rml63_teleop.safety import SafetyFilter
from rml63_teleop.state_machine import TeleopState
from rml63_teleop.xr_client import XRInput, XRTrackingError, XrClient


def frame(timestamp, grip=0.0, trigger=0.0):
    return XRInput(timestamp, np.array([0, 0, 0, 0, 0, 0, 1.0]), grip, trigger)


class FakeClock:
    def __init__(self):
        self.value = 0.0

    def monotonic(self):
        return self.value

    def sleep(self, duration):
        self.value += duration


class XRStartupTests(unittest.TestCase):
    def setUp(self):
        # 不初始化 SDK，不连接 PC Service。
        self.client = object.__new__(XrClient)
        self.clock = FakeClock()
        self.time_patch = patch("rml63_teleop.xr_client.time", self.clock)
        self.time_patch.start()
        self.addCleanup(self.time_patch.stop)

    def test_zero_pose_then_live_frames_become_ready(self):
        live_frame = frame(101)
        self.client.read = Mock(side_effect=[XRTrackingError("zero quaternion"), frame(100), live_frame])
        messages = []
        result = self.client.wait_until_ready(timeout_s=1, on_wait=messages.append)
        self.assertIs(result, live_frame)
        self.assertEqual(self.client.read.call_count, 3)
        self.assertEqual(len(messages), 1)

    def test_valid_pose_with_frozen_timestamp_times_out(self):
        self.client.read = Mock(return_value=frame(100))
        with self.assertRaisesRegex(TimeoutError, "时间戳"):
            self.client.wait_until_ready(timeout_s=0.1)

    def test_zero_timestamp_cannot_mark_input_ready(self):
        self.client.read = Mock(return_value=frame(0))
        with self.assertRaises(TimeoutError):
            self.client.wait_until_ready(timeout_s=0.1)

    def test_invalid_frame_resets_timestamp_confirmation(self):
        ready = frame(102)
        self.client.read = Mock(side_effect=[frame(100), XRTrackingError("lost"), frame(101), ready])
        self.assertIs(self.client.wait_until_ready(timeout_s=1), ready)
        self.assertEqual(self.client.read.call_count, 4)

    def test_unexpected_sdk_error_is_not_hidden_by_wait(self):
        self.client.read = Mock(side_effect=RuntimeError("SDK failure"))
        with self.assertRaisesRegex(RuntimeError, "SDK failure"):
            self.client.wait_until_ready(timeout_s=1)

    def test_motion_startup_waits_for_grip_and_trigger_release(self):
        self.client.read = Mock(side_effect=[
            frame(100, grip=1.0), frame(101, trigger=1.0), frame(102), frame(103),
        ])
        ready = self.client.wait_until_ready(timeout_s=1, release_grip_threshold=0.9)
        self.assertEqual(ready.timestamp_ns, 103)
        self.assertEqual(self.client.read.call_count, 4)


class RuntimeStartupTests(unittest.TestCase):
    def setUp(self):
        config = load_config(PACKAGE_ROOT / "rml63_teleop" / "defaults" / "teleop.yaml")
        config["robot"]["ip"] = "mock-robot"
        config["mount"]["calibration_confirmed"] = True
        config["mount"]["emergency_stop_confirmed"] = True
        config["safety"]["workspace"] = {axis: {"min": -1, "max": 1} for axis in "xyz"}
        self.runtime = object.__new__(TeleopRuntime)
        self.runtime.config = config
        self.runtime.dry_run = False
        self.runtime.robot = Mock()
        self.runtime.robot.ip = "mock-robot"
        self.runtime.robot.port = 8080
        self.runtime.robot.servo_joint_rad.return_value = 0
        self.runtime.robot.get_state.return_value = RobotState(np.zeros(6), {}, {})
        self.runtime.robot.get_joint_limits_rad.return_value = (np.full(6, -6.0), np.full(6, 6.0))
        self.runtime.robot.slow_stop.return_value = 0
        self.runtime.ik = Mock()
        self.runtime.ik.lower = np.full(6, -6.0)
        self.runtime.ik.upper = np.full(6, 6.0)
        self.runtime.ik.forward.return_value = Pose(np.zeros(3), np.array([1.0, 0, 0, 0]))
        self.runtime.ik.solve.return_value = np.zeros(6)
        self.runtime.safety = SafetyFilter(config["safety"], np.full(6, -6.0), np.full(6, 6.0))
        self.runtime.mapper = ClutchMapper(np.eye(3), 0.5)
        self.runtime.xr = None
        self.runtime.state = TeleopState.READY
        self.xr = Mock()
        self.xr.wait_until_ready.return_value = frame(90)
        self.telemetry_open = mock_open()
        patches = (
            patch("rml63_teleop.runtime.XrClient", return_value=self.xr),
            patch("rml63_teleop.runtime.Path.open", self.telemetry_open),
            patch("builtins.input", return_value="ENABLE RML63 MOTION"),
            patch("builtins.print"),
        )
        for patcher in patches:
            patcher.start()
            self.addCleanup(patcher.stop)

    def test_startup_timeout_never_sends_arm_or_gripper_targets(self):
        self.xr.wait_until_ready.side_effect = TimeoutError("XR startup timeout")
        with self.assertLogs("rml63_teleop", level="INFO"):
            with self.assertRaises(TimeoutError):
                self.runtime.run(enable_motion=True)
        self.runtime.robot.servo_joint_rad.assert_not_called()
        self.runtime.robot.set_gripper_position.assert_not_called()
        self.xr.read.assert_not_called()
        self.xr.close.assert_called_once()
        self.runtime.robot.disconnect.assert_called_once()

    def test_connection_refusal_before_confirmation_sends_no_stop_command(self):
        self.runtime.robot.connect.side_effect = RuntimeError("型号不匹配")
        with self.assertRaisesRegex(RuntimeError, "型号"):
            self.runtime.run(enable_motion=True)
        self.runtime.robot.slow_stop.assert_not_called()
        self.runtime.robot.servo_joint_rad.assert_not_called()
        self.runtime.robot.disconnect.assert_called_once()

    def test_tracking_loss_after_startup_still_faults_and_stops(self):
        events = []
        def ready(**kwargs):
            events.append("ready")
            return frame(90)
        self.xr.wait_until_ready.side_effect = ready
        self.xr.read.side_effect = [frame(100, grip=1.0), frame(101, grip=1.0), XRTrackingError("tracking lost")]
        self.runtime.robot.servo_joint_rad.side_effect = lambda *args, **kwargs: events.append("servo")
        with self.assertLogs("rml63_teleop", level="INFO"):
            self.runtime.run(enable_motion=True)
        self.assertEqual(events, ["ready", "servo"])
        self.assertEqual(self.runtime.state, TeleopState.FAULT)
        self.xr.wait_until_ready.assert_called_once()
        self.runtime.robot.slow_stop.assert_called()
        self.runtime.robot.set_gripper_position.assert_not_called()

    def test_dry_run_waits_and_never_sends_actions(self):
        self.xr.read.side_effect = [frame(100, grip=1.0), frame(101, grip=1.0, trigger=1.0), KeyboardInterrupt]
        with self.assertLogs("rml63_teleop", level="INFO"):
            self.runtime.run(enable_motion=False)
        self.xr.wait_until_ready.assert_called_once()
        self.runtime.robot.servo_joint_rad.assert_not_called()
        self.runtime.robot.set_gripper_position.assert_not_called()
        self.runtime.robot.slow_stop.assert_not_called()
        rows = [json.loads(call.args[0]) for call in self.telemetry_open.return_value.write.call_args_list]
        self.assertTrue(rows)
        for row in rows:
            self.assertEqual(row["mode"], "dry_run")
            self.assertFalse(row["motion_enabled"])
            self.assertEqual(row["arm_commands_sent"], 0)
            self.assertEqual(row["gripper_commands_sent"], 0)

    def test_stale_frame_is_rejected_before_any_new_action(self):
        self.xr.read.side_effect = [frame(100, grip=1.0), frame(100, grip=1.0, trigger=1.0)]
        with patch("rml63_teleop.runtime.TimestampWatchdog") as watchdog:
            watchdog.return_value.healthy.side_effect = [True, False]
            with self.assertLogs("rml63_teleop", level="INFO"):
                self.runtime.run(enable_motion=True)
        self.assertEqual(self.runtime.robot.servo_joint_rad.call_count, 0)
        self.runtime.robot.set_gripper_position.assert_not_called()
        self.assertEqual(self.runtime.state, TeleopState.FAULT)
        self.runtime.robot.slow_stop.assert_called()

    def test_first_clutch_frame_does_not_run_ik_or_send_joint_target(self):
        self.xr.read.side_effect = [frame(100, grip=1.0), KeyboardInterrupt]
        with self.assertLogs("rml63_teleop", level="INFO"):
            self.runtime.run(enable_motion=True)
        self.runtime.ik.solve.assert_not_called()
        self.runtime.robot.servo_joint_rad.assert_not_called()

    def test_workspace_rejection_latches_hold_and_blocks_gripper(self):
        outside = frame(101, grip=1.0, trigger=1.0)
        outside.right_pose[0] = 4.0
        self.xr.read.side_effect = [frame(100, grip=1.0), outside, KeyboardInterrupt]
        with self.assertLogs("rml63_teleop", level="INFO"):
            self.runtime.run(enable_motion=True)
        self.runtime.robot.servo_joint_rad.assert_not_called()
        self.runtime.robot.set_gripper_position.assert_not_called()
        self.runtime.ik.solve.assert_not_called()
        self.runtime.robot.slow_stop.assert_called()

    def test_filtered_joint_target_fk_is_checked_before_transmission(self):
        outside_pose = Pose(np.array([2.0, 0, 0]), np.array([1.0, 0, 0, 0]))
        inside_pose = Pose(np.zeros(3), np.array([1.0, 0, 0, 0]))
        self.runtime.ik.forward.side_effect = [inside_pose, inside_pose, inside_pose, outside_pose]
        self.xr.read.side_effect = [frame(100, grip=1.0), frame(101, grip=1.0), KeyboardInterrupt]
        with self.assertLogs("rml63_teleop", level="INFO"):
            self.runtime.run(enable_motion=True)
        self.runtime.robot.servo_joint_rad.assert_not_called()
        self.runtime.robot.slow_stop.assert_called()

    def test_gripper_can_be_disabled_for_arm_only_check(self):
        self.runtime.config["gripper"]["enabled"] = False
        self.xr.read.side_effect = [frame(100), frame(101, trigger=1.0), KeyboardInterrupt]
        with self.assertLogs("rml63_teleop", level="INFO"):
            self.runtime.run(enable_motion=True)
        self.runtime.robot.set_gripper_position.assert_not_called()

    def test_canfd_failure_stops_and_does_not_send_gripper_command(self):
        self.runtime.robot.servo_joint_rad.side_effect = RuntimeError("CANFD failure")
        self.xr.read.side_effect = [frame(100, grip=1.0), frame(101, grip=1.0, trigger=1.0)]
        with self.assertLogs("rml63_teleop", level="INFO"):
            self.runtime.run(enable_motion=True)
        self.assertEqual(self.runtime.state, TeleopState.FAULT)
        self.runtime.robot.slow_stop.assert_called()
        self.runtime.robot.set_gripper_position.assert_not_called()

    def test_start_pose_outside_workspace_refuses_control_loop(self):
        self.runtime.ik.forward.return_value = Pose(np.array([2.0, 0, 0]), np.array([1.0, 0, 0, 0]))
        with self.assertLogs("rml63_teleop", level="INFO"):
            with self.assertRaisesRegex(RuntimeError, "启动末端位置"):
                self.runtime.run(enable_motion=True)
        self.xr.read.assert_not_called()
        self.runtime.robot.servo_joint_rad.assert_not_called()

    def test_reengagement_after_ik_hold_uses_fresh_measured_seed(self):
        self.runtime.config["safety"]["ik_failure_limit"] = 1
        measured = np.full(6, 0.02)
        self.runtime.ik.solve.side_effect = [RuntimeError("IK failure"), measured.copy()]
        old_state = RobotState(np.zeros(6), [], {})
        fresh_state = RobotState(np.rad2deg(measured), [], {})
        self.runtime.robot.get_state.side_effect = [old_state] * 4 + [fresh_state] * 3
        self.xr.read.side_effect = [frame(100, grip=1.0), frame(101, grip=1.0),
                                   frame(102), frame(103, grip=1.0), frame(104, grip=1.0), KeyboardInterrupt]
        with self.assertLogs("rml63_teleop", level="INFO"):
            self.runtime.run(enable_motion=True)
        self.assertEqual(self.runtime.ik.solve.call_count, 2)
        np.testing.assert_allclose(self.runtime.ik.solve.call_args.args[1], measured)
        self.runtime.robot.servo_joint_rad.assert_called_once()

    def test_command_lead_limit_faults_before_sending_arm_target(self):
        self.runtime.config["safety"]["max_joint_tracking_error_deg"] = 0.1
        self.runtime.ik.solve.return_value = np.full(6, 0.2)
        self.xr.read.side_effect = [frame(100, grip=1.0), frame(101, grip=1.0)]
        with self.assertLogs("rml63_teleop", level="INFO"):
            with self.assertRaisesRegex(RuntimeError, "ahead"):
                self.runtime.run(enable_motion=True)
        self.runtime.robot.servo_joint_rad.assert_not_called()
        self.runtime.robot.slow_stop.assert_called()

    def test_gripper_api_failure_faults_and_stops_arm(self):
        self.runtime.robot.set_gripper_position.side_effect = RuntimeError("gripper failure")
        self.xr.read.side_effect = [frame(100), frame(101, trigger=1.0)]
        with self.assertLogs("rml63_teleop", level="INFO"):
            self.runtime.run(enable_motion=True)
        self.assertEqual(self.runtime.state, TeleopState.FAULT)
        self.runtime.robot.servo_joint_rad.assert_not_called()
        self.runtime.robot.slow_stop.assert_called()

    def test_expired_computation_cannot_transmit_command(self):
        watchdog = Mock()
        watchdog.healthy.return_value = True
        with patch("rml63_teleop.runtime.time.monotonic", return_value=1.3):
            with self.assertRaisesRegex(RuntimeError, "过期"):
                TeleopRuntime._require_fresh_command(watchdog, tick_start=1.0, timeout_s=0.2)


if __name__ == "__main__":
    unittest.main()
