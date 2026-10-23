from __future__ import annotations

from pathlib import Path

import numpy as np
import placo

from .geometry import Pose, matrix_to_quaternion, quaternion_to_matrix

JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]
EE_LINK = "gripper_eef"


class PlacoIK:
    def __init__(self, urdf_path: str | Path, dof: int = 6):
        self.robot = placo.RobotWrapper(str(urdf_path), placo.Flags.ignore_collisions)
        self.solver = placo.KinematicsSolver(self.robot)
        self.solver.mask_fbase(True)
        self.solver.enable_joint_limits(True)
        self.solver.dt = 0.02
        self.solver.add_kinetic_energy_regularization_task(1e-6)
        self.frame_task = self.solver.add_frame_task(EE_LINK, np.eye(4))
        self.frame_task.configure("teleop_ee", "soft", 1.0)
        self.dof = dof
        self.q_slice = slice(7, 7 + dof)  # Placo stores the floating base as xyz + quaternion.
        if len(self.robot.state.q) != 7 + dof:
            raise RuntimeError(f"URDF state has {len(self.robot.state.q)} coordinates; expected {7 + dof}")
        self.lower = np.asarray(self.robot.model.lowerPositionLimit[self.q_slice], dtype=float).copy()
        self.upper = np.asarray(self.robot.model.upperPositionLimit[self.q_slice], dtype=float).copy()

    def restrict_joint_limits(self, lower: np.ndarray, upper: np.ndarray) -> None:
        lower = np.maximum(self.lower, np.asarray(lower, dtype=float).reshape(self.dof))
        upper = np.minimum(self.upper, np.asarray(upper, dtype=float).reshape(self.dof))
        if not np.all(np.isfinite(lower)) or not np.all(np.isfinite(upper)) or np.any(lower >= upper):
            raise ValueError("模型限位与控制器软限位没有有效交集")
        self.lower, self.upper = lower, upper
        self.robot.model.lowerPositionLimit[self.q_slice] = lower
        self.robot.model.upperPositionLimit[self.q_slice] = upper

    def update(self, joint_rad: np.ndarray) -> None:
        q = np.asarray(joint_rad, dtype=float).reshape(self.dof)
        if not np.all(np.isfinite(q)):
            raise ValueError("Joint state contains non-finite values")
        self.robot.state.q[self.q_slice] = q
        self.robot.update_kinematics()

    def forward(self, joint_rad: np.ndarray) -> Pose:
        self.update(joint_rad)
        transform = self.robot.get_T_world_frame(EE_LINK)
        return Pose(transform[:3, 3].copy(), matrix_to_quaternion(transform[:3, :3]))

    def solve(self, target: Pose, seed_rad: np.ndarray) -> np.ndarray:
        self.update(seed_rad)
        target = target.normalized()
        current = self.forward(seed_rad)
        # 目标等于当前姿态时直接保持，不能用辅助优化目标驱动机械臂漂移。
        if np.linalg.norm(target.position - current.position) < 1e-9 and abs(float(np.dot(target.quaternion, current.quaternion))) > 1.0 - 1e-12:
            return np.asarray(seed_rad, dtype=float).copy()
        transform = np.eye(4)
        transform[:3, :3] = quaternion_to_matrix(target.quaternion)
        transform[:3, 3] = target.position
        self.frame_task.T_world_frame = transform
        result = self.solver.solve(True)
        q = np.asarray(self.robot.state.q[self.q_slice], dtype=float).copy()
        if result is False or q.size != self.dof or not np.all(np.isfinite(q)):
            raise RuntimeError("Placo returned an invalid IK solution")
        return q
