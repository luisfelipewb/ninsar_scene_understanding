"""Degrees in, quaternion out - fixed-axis ZYX, the convention used repo-wide.

`R = Rz(yaw) * Ry(pitch) * Rx(roll)`, matching `quaternion_from_rpy` in
bagfix/transforms/tf_static.py and `from_xyz_rpy` in ninsar_lidar_calib. Kept
dependency-free on purpose so this package needs nothing but rclpy.
"""

from __future__ import annotations

import math

Quaternion = tuple[float, float, float, float]


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Fixed-axis roll/pitch/yaw in radians -> (x, y, z, w)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def quaternion_from_rpy_deg(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Fixed-axis roll/pitch/yaw in degrees -> (x, y, z, w)."""
    return quaternion_from_rpy(*(math.radians(a) for a in (roll, pitch, yaw)))


def rpy_from_quaternion(x: float, y: float, z: float, w: float) -> tuple[float, ...]:
    """(x, y, z, w) -> fixed-axis roll/pitch/yaw in radians.

    The asin argument is clamped and the gimbal-lock branch folds roll into
    yaw, the same guard the C++ `toXyzRpy` uses.
    """
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        raise ValueError("quaternion has zero norm")
    x, y, z, w = (component / norm for component in (x, y, z, w))

    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sin_pitch)
    if abs(sin_pitch) > 1.0 - 1e-12:
        # Straight up or down: roll and yaw are the same axis, report it as yaw.
        return (0.0, pitch, 2.0 * math.atan2(z, w))
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return (roll, pitch, yaw)


def rpy_deg_from_quaternion(x: float, y: float, z: float, w: float) -> tuple[float, ...]:
    """(x, y, z, w) -> fixed-axis roll/pitch/yaw in degrees."""
    return tuple(math.degrees(a) for a in rpy_from_quaternion(x, y, z, w))
