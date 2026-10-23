from __future__ import annotations

from dataclasses import dataclass

import numpy as np

QUEST_REFERENCE_ROTATION = np.array(
    [[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=float
)


def _normalize_quaternion(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float).reshape(4)
    norm = float(np.linalg.norm(q))
    if not np.all(np.isfinite(q)) or norm < 1e-9:
        raise ValueError("Quaternion must be finite and non-zero")
    return q / norm


def quat_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = _normalize_quaternion(a)
    bw, bx, by, bz = _normalize_quaternion(b)
    return _normalize_quaternion(
        np.array(
            [
                aw * bw - ax * bx - ay * by - az * bz,
                aw * bx + ax * bw + ay * bz - az * by,
                aw * by - ax * bz + ay * bw + az * bx,
                aw * bz + ax * by - ay * bx + az * bw,
            ]
        )
    )


def quat_inverse(q: np.ndarray) -> np.ndarray:
    w, x, y, z = _normalize_quaternion(q)
    return np.array([w, -x, -y, -z])


def quaternion_to_matrix(q: np.ndarray) -> np.ndarray:
    w, x, y, z = _normalize_quaternion(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def matrix_to_quaternion(matrix: np.ndarray) -> np.ndarray:
    """Convert a proper 3x3 rotation matrix to a [w, x, y, z] quaternion."""
    m = np.asarray(matrix, dtype=float).reshape(3, 3)
    if not np.all(np.isfinite(m)):
        raise ValueError("Rotation matrix must be finite")
    trace = float(np.trace(m))
    if trace > 0:
        s = np.sqrt(trace + 1.0) * 2
        q = np.array([0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s])
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        q = np.array([(m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s])
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        q = np.array([(m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s])
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        q = np.array([(m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s])
    return _normalize_quaternion(q)


def transform_quaternion(q_xr_xyzw: np.ndarray, rotation_xr_to_base: np.ndarray) -> np.ndarray:
    q_xyzw = np.asarray(q_xr_xyzw, dtype=float).reshape(4)
    q_xr = _normalize_quaternion(np.array([q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]]))
    q_map = matrix_to_quaternion(rotation_xr_to_base)
    return quat_multiply(quat_multiply(q_map, q_xr), quat_inverse(q_map))


def transform_controller_pose(pose_xyzw: np.ndarray, rotation_xr_to_base: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    pose = np.asarray(pose_xyzw, dtype=float).reshape(7)
    if not np.all(np.isfinite(pose)):
        raise ValueError("XR controller pose contains non-finite values")
    rotation = np.asarray(rotation_xr_to_base, dtype=float).reshape(3, 3)
    return rotation @ pose[:3], transform_quaternion(pose[3:], rotation)


@dataclass
class Pose:
    position: np.ndarray
    quaternion: np.ndarray  # [w, x, y, z]

    def normalized(self) -> "Pose":
        p = np.asarray(self.position, dtype=float).reshape(3)
        if not np.all(np.isfinite(p)):
            raise ValueError("Pose position must be finite")
        return Pose(p, _normalize_quaternion(self.quaternion))


class ClutchMapper:
    """Maps controller deltas to the robot EE with a no-jump clutch."""

    def __init__(self, rotation_xr_to_base: np.ndarray, scale_factor: float):
        self.rotation = np.asarray(rotation_xr_to_base, dtype=float).reshape(3, 3)
        self.scale_factor = float(scale_factor)
        self.controller_ref: Pose | None = None
        self.ee_ref: Pose | None = None

    @property
    def active(self) -> bool:
        return self.controller_ref is not None

    def release(self) -> None:
        self.controller_ref = None
        self.ee_ref = None

    def update(self, xr_pose_xyzw: np.ndarray, grip_active: bool, current_ee: Pose) -> Pose | None:
        current_ee = current_ee.normalized()
        if not grip_active:
            self.release()
            return None

        controller_position, controller_quaternion = transform_controller_pose(xr_pose_xyzw, self.rotation)
        if self.controller_ref is None:
            self.controller_ref = Pose(controller_position.copy(), controller_quaternion.copy())
            self.ee_ref = Pose(current_ee.position.copy(), current_ee.quaternion.copy())
            return current_ee

        assert self.ee_ref is not None
        delta_position = (controller_position - self.controller_ref.position) * self.scale_factor
        delta_quaternion = quat_multiply(controller_quaternion, quat_inverse(self.controller_ref.quaternion))
        target_quaternion = quat_multiply(delta_quaternion, self.ee_ref.quaternion)
        return Pose(self.ee_ref.position + delta_position, target_quaternion).normalized()
