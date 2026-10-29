from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from Robotic_Arm.rm_robot_interface import RoboticArm, rm_thread_mode_e


@dataclass
class RobotState:
    joint_deg: np.ndarray
    pose: list[float]
    error: object

    @property
    def joint_rad(self) -> np.ndarray:
        return np.deg2rad(self.joint_deg)


class RealManArm:
    """Thin adapter for the installed RealMan Python API; no IK/XR logic."""

    def __init__(self, ip: str, port: int = 8080, dof: int = 6, expected_controller_model: str = "RML63III-6FB"):
        if not ip:
            raise ValueError("Robot IP is required")
        self.ip = ip
        self.port = int(port)
        self.dof = int(dof)
        self.expected_controller_model = expected_controller_model
        self.arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
        self.connected = False

    def connect(self) -> dict:
        handle = self.arm.rm_create_robot_arm(self.ip, self.port)
        if handle is None or getattr(handle, "id", -1) == -1:
            raise ConnectionError(f"Could not connect to RealMan controller at {self.ip}:{self.port}")
        self.connected = True
        if int(getattr(self.arm, "arm_dof", 0)) not in (0, self.dof):
            raise RuntimeError(f"Controller reports {self.arm.arm_dof} DoF; expected {self.dof}")
        code, info = self.arm.rm_get_arm_software_info()
        if code != 0:
            raise RuntimeError(f"rm_get_arm_software_info failed with code {code}")
        if info.get("product_version") != self.expected_controller_model:
            raise RuntimeError(
                f"控制器型号为 {info.get('product_version')!r}，配置要求 {self.expected_controller_model!r}"
            )
        return info

    def get_joint_limits_rad(self) -> tuple[np.ndarray, np.ndarray]:
        if not self.connected:
            raise RuntimeError("Robot is not connected")
        limits = []
        for name in ("rm_get_joint_min_pos", "rm_get_joint_max_pos"):
            code, values = getattr(self.arm, name)()
            if code != 0:
                raise RuntimeError(f"{name} failed with code {code}")
            values = np.asarray(values, dtype=float).reshape(-1)
            if values.size != self.dof or not np.all(np.isfinite(values)):
                raise RuntimeError(f"Invalid controller joint limits: {values}")
            limits.append(np.deg2rad(values))
        lower, upper = limits
        if np.any(lower >= upper):
            raise RuntimeError("控制器返回了无效的关节软限位")
        return lower, upper

    def get_state(self) -> RobotState:
        if not self.connected:
            raise RuntimeError("Robot is not connected")
        code, data = self.arm.rm_get_current_arm_state()
        if code != 0:
            raise RuntimeError(f"rm_get_current_arm_state failed with code {code}")
        joints = np.asarray(data.get("joint", []), dtype=float).reshape(-1)
        if joints.size < self.dof or not np.all(np.isfinite(joints[: self.dof])):
            raise RuntimeError(f"Invalid joint state from controller: {joints}")
        error = data.get("err")
        if not isinstance(error, dict) or "err_len" not in error or not isinstance(error.get("err"), list):
            raise RuntimeError(f"Malformed robot error state from controller: {error}")
        if isinstance(error, dict):
            error_len = int(error.get("err_len", 0))
            error_codes = error.get("err", [])
            if error_len < 0 or error_len != len(error_codes) or error_len > 24:
                raise RuntimeError(f"Malformed robot error state from controller: {error}")
            active_codes = []
            for code in error_codes[:error_len]:
                try:
                    is_clear_code = int(code) == 0
                except (TypeError, ValueError):
                    is_clear_code = False
                if not is_clear_code:
                    active_codes.append(code)
            # Some controller/SDK combinations report the normal-state sentinel
            # as err_len=1, err=[0]. Only nonzero error codes indicate a fault.
            if active_codes:
                raise RuntimeError(f"Robot controller reports errors: {active_codes}")
        return RobotState(joints[: self.dof].copy(), data.get("pose", {}), error)

    def get_execution_status(self) -> dict:
        """只读查询运行模式、关节使能和夹爪；不自动切换模式或上使能。"""
        if not self.connected:
            raise RuntimeError("Robot is not connected")
        result = {}
        for name, api in (("run_mode", "rm_get_arm_run_mode"),
                          ("joint_enable", "rm_get_joint_en_state"),
                          ("gripper", "rm_get_gripper_state")):
            code, value = getattr(self.arm, api)()
            result[name] = {"return_code": int(code), "value": value}
        return result

    def servo_joint_rad(self, joint_rad: np.ndarray, follow: bool = False) -> int:
        if follow is not False:
            raise ValueError("当前项目禁止 CANFD 高跟随模式")
        q = np.asarray(joint_rad, dtype=float).reshape(-1)
        if q.size != self.dof or not np.all(np.isfinite(q)):
            raise ValueError("CANFD command must contain six finite joint angles in radians")
        result = int(self.arm.rm_movej_canfd(np.rad2deg(q).tolist(), follow=follow, expand=0, trajectory_mode=0, radio=0))
        if result != 0:
            raise RuntimeError(f"rm_movej_canfd failed with code {result}")
        return result

    def servo_pose(self, position_m: np.ndarray, quaternion_wxyz: np.ndarray, follow: bool = False) -> int:
        """Pass a Cartesian target to RealMan CANFD; the controller computes IK."""
        if follow is not False:
            raise ValueError("当前项目禁止 CANFD 高跟随模式")
        position = np.asarray(position_m, dtype=float).reshape(3)
        quaternion = np.asarray(quaternion_wxyz, dtype=float).reshape(4)
        if not np.all(np.isfinite(position)) or not np.all(np.isfinite(quaternion)):
            raise ValueError("CANFD pose must contain finite position and quaternion values")
        norm = float(np.linalg.norm(quaternion))
        if norm < 1e-9:
            raise ValueError("CANFD pose quaternion must be non-zero")
        quaternion = quaternion / norm
        # SDK rm_movep_canfd uses [x, y, z, qw, qx, qy, qz] for length 7.
        pose = np.concatenate([position, quaternion]).tolist()
        result = int(self.arm.rm_movep_canfd(pose, follow=follow, trajectory_mode=0, radio=0))
        if result != 0:
            raise RuntimeError(f"rm_movep_canfd failed with code {result}")
        return result

    def set_gripper_position(self, position: int) -> int:
        if not 1 <= int(position) <= 1000:
            raise ValueError("Gripper position must be in [1, 1000]")
        result = int(self.arm.rm_set_gripper_position(int(position), block=False, timeout=0))
        if result != 0:
            raise RuntimeError(f"rm_set_gripper_position failed with code {result}")
        return result

    def slow_stop(self) -> int:
        if not self.connected:
            return -1
        code = int(self.arm.rm_set_arm_slow_stop())
        if code != 0:
            raise RuntimeError(f"rm_set_arm_slow_stop failed with code {code}; use the physical E-stop")
        return code

    def disconnect(self) -> None:
        if self.connected:
            self.arm.rm_delete_robot_arm()
            self.connected = False
