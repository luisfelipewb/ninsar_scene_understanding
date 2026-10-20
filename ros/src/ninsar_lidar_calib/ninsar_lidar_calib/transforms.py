"""Small rigid-transform helpers, matching the C++ side's conventions."""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation


def isometry(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = translation
    return matrix


def from_params(params: np.ndarray) -> np.ndarray:
    """(tx, ty, tz, rx, ry, rz) with a rotation vector -> 4x4 matrix.

    A rotation vector is used throughout the solver because it has no gimbal
    lock and no wrap-around, which Euler angles would give the optimizer.
    """
    return isometry(Rotation.from_rotvec(params[3:6]).as_matrix(), params[0:3])


def to_params(matrix: np.ndarray) -> np.ndarray:
    """4x4 matrix -> (tx, ty, tz, rx, ry, rz)."""
    return np.concatenate(
        [matrix[:3, 3], Rotation.from_matrix(matrix[:3, :3]).as_rotvec()]
    )


def from_xyz_rpy(xyz, rpy) -> np.ndarray:
    """Fixed-axis roll/pitch/yaw, the same ZYX order as the C++ fromXyzRpy."""
    return isometry(Rotation.from_euler("ZYX", [rpy[2], rpy[1], rpy[0]]).as_matrix(),
                    np.asarray(xyz, float))


def to_xyz_rpy(matrix: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """4x4 matrix -> (xyz, rpy), inverse of from_xyz_rpy."""
    yaw, pitch, roll = Rotation.from_matrix(matrix[:3, :3]).as_euler("ZYX")
    return matrix[:3, 3].copy(), np.array([roll, pitch, yaw])


def rotation_log(matrix: np.ndarray) -> np.ndarray:
    """Rotation matrix -> rotation vector; the orientation residual."""
    return Rotation.from_matrix(matrix).as_rotvec()


def quaternion_from_matrix(matrix: np.ndarray) -> np.ndarray:
    """4x4 or 3x3 -> (x, y, z, w)."""
    return Rotation.from_matrix(matrix[:3, :3]).as_quat()
