import unittest

import numpy as np

from rml63_teleop.geometry import ClutchMapper, Pose, QUEST_REFERENCE_ROTATION, matrix_to_quaternion, quaternion_to_matrix, transform_quaternion
from rml63_teleop.config import load_config, validate_config
from rml63_teleop.safety import SafetyFilter
from rml63_teleop.watchdog import TimestampWatchdog
from rml63_teleop.placo_ik import EE_LINK, JOINT_NAMES, PlacoIK
from rml63_teleop.config import PACKAGE_ROOT


class GeometryTests(unittest.TestCase):
    def test_reference_translation_axis_mapping(self):
        mapped = QUEST_REFERENCE_ROTATION @ np.array([0.1, 0.0, 0.0])
        np.testing.assert_allclose(mapped, [0.0, 0.1, 0.0])

    def test_quaternion_round_trip(self):
        q = np.array([0.9238795325, 0.0, 0.0, 0.3826834324])
        np.testing.assert_allclose(quaternion_to_matrix(matrix_to_quaternion(quaternion_to_matrix(q))), quaternion_to_matrix(q), atol=1e-8)

    def test_same_orientation_produces_zero_rotation(self):
        mapper = ClutchMapper(QUEST_REFERENCE_ROTATION, 0.5)
        ee = Pose(np.array([0.4, 0.0, 0.3]), np.array([1.0, 0.0, 0.0, 0.0]))
        xr = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        mapper.update(xr, True, ee)
        target = mapper.update(xr, True, ee)
        np.testing.assert_allclose(target.quaternion, ee.quaternion, atol=1e-8)

    def test_translation_with_fixed_controller_orientation_preserves_ee_orientation(self):
        mapper = ClutchMapper(np.eye(3), 0.5)
        ee_q = matrix_to_quaternion(quaternion_to_matrix(np.array([0.9659258263, 0.0, 0.2588190451, 0.0])))
        ee = Pose(np.array([0.4, 0.0, 0.3]), ee_q)
        xr = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        mapper.update(xr, True, ee)
        moved = xr.copy()
        moved[:3] = [0.2, -0.1, 0.3]
        target = mapper.update(moved, True, ee)
        np.testing.assert_allclose(target.position, ee.position + 0.5 * moved[:3], atol=1e-8)
        np.testing.assert_allclose(quaternion_to_matrix(target.quaternion), quaternion_to_matrix(ee.quaternion), atol=1e-8)

    def test_reflection_axis_map_converts_orientation(self):
        axis_map = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
        xr_q_xyzw = np.array([np.sin(np.pi / 4), 0.0, 0.0, np.cos(np.pi / 4)])
        mapped_wxyz = transform_quaternion(xr_q_xyzw, axis_map)
        expected = np.array([np.cos(np.pi / 4), 0.0, np.sin(np.pi / 4), 0.0])
        np.testing.assert_allclose(quaternion_to_matrix(mapped_wxyz), quaternion_to_matrix(expected), atol=1e-8)

    def test_config_accepts_orthogonal_left_to_right_handed_axis_map(self):
        config = load_config(PACKAGE_ROOT / "rml63_teleop" / "defaults" / "teleop.yaml")
        config["robot"]["ip"] = "mock-robot"
        config["mount"]["xr_to_base_rotation"] = [[0, 0, 1], [-1, 0, 0], [0, 1, 0]]
        config["mount"]["calibration_confirmed"] = True
        config["mount"]["emergency_stop_confirmed"] = True
        config["safety"]["workspace"] = {axis: {"min": -1, "max": 1} for axis in "xyz"}
        self.assertEqual(validate_config(config, require_motion_ready=True), [])

    def test_rotation_deadband_and_smoothing(self):
        mapper = ClutchMapper(np.eye(3), 0.5, rotation_deadband_deg=1.0, rotation_smoothing_alpha=0.25)
        ee = Pose(np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]))
        xr = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        mapper.update(xr, True, ee)
        small = xr.copy()
        small[5], small[6] = np.sin(np.deg2rad(0.5) / 2), np.cos(np.deg2rad(0.5) / 2)
        target = mapper.update(small, True, ee)
        np.testing.assert_allclose(target.quaternion, ee.quaternion, atol=1e-8)
        larger = xr.copy()
        larger[5], larger[6] = np.sin(np.deg2rad(10.0) / 2), np.cos(np.deg2rad(10.0) / 2)
        target = mapper.update(larger, True, ee)
        angle = 2 * np.arccos(np.clip(abs(target.quaternion[0]), -1.0, 1.0))
        self.assertAlmostEqual(np.rad2deg(angle), 2.5, places=5)

    def test_clutch_press_and_reengage_have_no_jump(self):
        mapper = ClutchMapper(QUEST_REFERENCE_ROTATION, 0.5)
        ee = Pose(np.array([0.4, 0.0, 0.3]), np.array([1.0, 0.0, 0.0, 0.0]))
        xr = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        np.testing.assert_allclose(mapper.update(xr, True, ee).position, ee.position)
        moved_xr = xr.copy()
        moved_xr[0] = 0.1
        first_target = mapper.update(moved_xr, True, ee)
        np.testing.assert_allclose(first_target.position, [0.4, 0.05, 0.3])
        mapper.update(xr, False, ee)
        moved_xr[0] = 0.8
        second_target = mapper.update(moved_xr, True, ee)
        np.testing.assert_allclose(second_target.position, ee.position)

    def test_local_rml63_model_loads_with_six_arm_joints(self):
        urdf = PACKAGE_ROOT / "rml63_teleop" / "assets" / "rml63" / "robot_ik.urdf"
        ik = PlacoIK(urdf)
        self.assertEqual(list(ik.robot.joint_names()), JOINT_NAMES)
        np.testing.assert_allclose(
            ik.lower,
            np.deg2rad([-178, -178, -178, -178, -178, -360]),
            atol=1e-6,
        )
        np.testing.assert_allclose(
            ik.upper,
            np.deg2rad([178, 178, 145, 178, 178, 360]),
            atol=1e-6,
        )
        pose = ik.forward(np.zeros(6))
        self.assertEqual(pose.position.shape, (3,))
        self.assertEqual(pose.quaternion.shape, (4,))

    def test_stationary_target_has_no_joint_drift_at_recorded_robot_pose(self):
        ik = PlacoIK(PACKAGE_ROOT / "rml63_teleop" / "assets" / "rml63" / "robot_ik.urdf")
        q = np.deg2rad([-0.546, -21.167, 78.528, 4.222, 122.58, -185.403])
        target = ik.forward(q)
        np.testing.assert_array_equal(ik.solve(target, q), q)
        # 微小追踪抖动也不能触发原辅助任务造成的毫米级漂移。
        target.position[0] += 1e-7
        command = ik.solve(target, q)
        self.assertLess(float(np.max(np.abs(command - q))), 1e-5)
        self.assertLess(float(np.linalg.norm(ik.forward(command).position - target.position)), 1e-6)

    def test_small_cartesian_target_progresses_without_leaving_joint_limits(self):
        ik = PlacoIK(PACKAGE_ROOT / "rml63_teleop" / "assets" / "rml63" / "robot_ik.urdf")
        q = np.deg2rad([-0.546, -21.167, 78.528, 4.222, 122.58, -185.403])
        target = ik.forward(q)
        target.position[0] += 0.002
        command = ik.solve(target, q)
        self.assertTrue(np.all(command >= ik.lower) and np.all(command <= ik.upper))
        self.assertLess(float(np.linalg.norm(ik.forward(command).position - target.position)), 0.0001)

    def test_effective_joint_limits_never_widen_urdf_limits(self):
        ik = PlacoIK(PACKAGE_ROOT / "rml63_teleop" / "assets" / "rml63" / "robot_ik.urdf")
        lower, upper = ik.lower.copy(), ik.upper.copy()
        ik.restrict_joint_limits(lower - 1, upper + 1)
        np.testing.assert_array_equal(ik.lower, lower)
        np.testing.assert_array_equal(ik.upper, upper)
        ik.restrict_joint_limits(lower + 0.1, upper - 0.1)
        np.testing.assert_allclose(ik.robot.model.lowerPositionLimit[ik.q_slice], lower + 0.1)
        np.testing.assert_allclose(ik.robot.model.upperPositionLimit[ik.q_slice], upper - 0.1)

    def test_6fb_flange_fk_agrees_with_realman_offline_algorithm(self):
        # 只初始化离线算法，不创建机械臂连接，不发送任何硬件指令。
        from Robotic_Arm.rm_robot_interface import Algo, rm_robot_arm_model_e, rm_force_type_e

        algorithm = Algo(rm_robot_arm_model_e.RM_MODEL_RM_63_III_E, rm_force_type_e.RM_MODEL_RM_ISF_E)
        ik = PlacoIK(PACKAGE_ROOT / "rml63_teleop" / "assets" / "rml63" / "robot_ik.urdf")
        cases = ([0] * 6, [-0.546, -21.167, 78.528, 4.222, 122.58, -185.403],
                 [15, -30, 60, 20, 90, 30], [-25, 40, -50, 60, 100, -30])
        for joint_deg in cases:
            with self.subTest(joint_deg=joint_deg):
                expected = algorithm.rm_algo_forward_kinematics(list(joint_deg), flag=0)
                ik.update(np.deg2rad(joint_deg))
                flange = ik.robot.get_T_world_frame("link6")
                # 本地资源的尺寸和 RPY 有小量舍入误差，容差分别为 0.2 mm 和 0.05 度。
                self.assertLess(float(np.linalg.norm(flange[:3, 3] - np.array(expected[:3]))), 0.0002)
                rotation_delta = flange[:3, :3] @ quaternion_to_matrix(np.array(expected[3:])).T
                angle = np.arccos(np.clip((np.trace(rotation_delta) - 1) / 2, -1, 1))
                self.assertLess(float(angle), np.deg2rad(0.05))


class SafetyTests(unittest.TestCase):
    def setUp(self):
        self.safety = SafetyFilter(
            {
                "workspace": {"x": {"min": -1, "max": 1}, "y": {"min": -1, "max": 1}, "z": {"min": 0, "max": 1}},
                "max_joint_step_deg": 1.0,
                "max_joint_velocity_deg_s": 20.0,
            },
            np.full(6, -np.pi),
            np.full(6, np.pi),
        )

    def test_rejects_nonfinite_joint_target(self):
        result = self.safety.filter_joint(np.array([np.nan] * 6), np.zeros(6), 0.02)
        self.assertFalse(result.accepted)

    def test_rejects_joint_limit_violation(self):
        q = np.zeros(6)
        q[0] = 4.0
        result = self.safety.filter_joint(q, np.zeros(6), 0.02)
        self.assertFalse(result.accepted)

    def test_limits_per_cycle_delta(self):
        candidate = np.full(6, np.deg2rad(10))
        result = self.safety.filter_joint(candidate, np.zeros(6), 0.02)
        self.assertTrue(result.accepted)
        self.assertLessEqual(float(np.max(np.abs(result.joint_rad))), np.deg2rad(0.400001))

    def test_rejects_workspace_violation(self):
        result = self.safety.check_pose(Pose(np.array([0, 0, 1.1]), np.array([1, 0, 0, 0])))
        self.assertFalse(result.accepted)

    def test_xr_watchdog_trips_on_stale_timestamp(self):
        watchdog = TimestampWatchdog(timeout_s=0.2)
        watchdog.observe(10, now=1.0)
        self.assertTrue(watchdog.healthy(1.1))
        watchdog.observe(10, now=1.25)
        self.assertFalse(watchdog.healthy(1.25))
        watchdog.observe(11, now=1.3)
        self.assertTrue(watchdog.healthy(1.3))

    def test_motion_gate_requires_site_configuration(self):
        config = load_config(PACKAGE_ROOT / "rml63_teleop" / "defaults" / "teleop.yaml")
        issues = validate_config(config, require_motion_ready=True)
        self.assertTrue(any("robot.ip" in issue for issue in issues))
        self.assertTrue(any("workspace.x" in issue for issue in issues))
        self.assertTrue(any("calibration_confirmed" in issue for issue in issues))

    def test_watchdog_rejects_zero_or_regressing_timestamp(self):
        watchdog = TimestampWatchdog(timeout_s=0.2)
        self.assertFalse(watchdog.healthy(1.0))
        with self.assertRaises(ValueError):
            watchdog.observe(0, 1.0)
        watchdog.observe(100, 1.0)
        with self.assertRaises(ValueError):
            watchdog.observe(99, 1.1)


if __name__ == "__main__":
    unittest.main()
