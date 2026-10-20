"""Angle conventions and settings validation - no ROS graph needed."""

from __future__ import annotations

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ninsar_tf_tools.angles import (  # noqa: E402
    quaternion_from_rpy,
    quaternion_from_rpy_deg,
    rpy_deg_from_quaternion,
)

IDENTITY = (0.0, 0.0, 0.0, 1.0)
HALF = math.sqrt(0.5)


def assert_same_rotation(actual, expected, tol=1e-9):
    """Quaternions are a double cover: q and -q are the same rotation."""
    flipped = tuple(-component for component in expected)
    assert (
        all(abs(a - b) < tol for a, b in zip(actual, expected))
        or all(abs(a - b) < tol for a, b in zip(actual, flipped))
    ), f"{actual} is not {expected}"


@pytest.mark.parametrize("rpy_deg,expected", [
    ((0.0, 0.0, 0.0), IDENTITY),
    ((0.0, 0.0, 90.0), (0.0, 0.0, HALF, HALF)),
    ((0.0, 90.0, 0.0), (0.0, HALF, 0.0, HALF)),
    ((90.0, 0.0, 0.0), (HALF, 0.0, 0.0, HALF)),
    ((180.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)),
    ((0.0, 0.0, 180.0), (0.0, 0.0, 1.0, 0.0)),
])
def test_known_rotations(rpy_deg, expected):
    assert_same_rotation(quaternion_from_rpy_deg(*rpy_deg), expected)


def test_unit_norm():
    quat = quaternion_from_rpy_deg(-37.0, 12.5, 143.0)
    assert math.isclose(sum(c * c for c in quat), 1.0, rel_tol=0.0, abs_tol=1e-12)


@pytest.mark.parametrize("roll", [-170.0, -90.0, 0.0, 45.0, 170.0])
@pytest.mark.parametrize("pitch", [-89.0, -30.0, 0.0, 37.0, 89.0])
@pytest.mark.parametrize("yaw", [-179.0, -60.0, 0.0, 90.0, 179.0])
def test_round_trip(roll, pitch, yaw):
    quat = quaternion_from_rpy_deg(roll, pitch, yaw)
    back = rpy_deg_from_quaternion(*quat)
    assert_same_rotation(quaternion_from_rpy_deg(*back), quat, tol=1e-6)
    for original, recovered in zip((roll, pitch, yaw), back):
        assert abs(original - recovered) < 1e-6


@pytest.mark.parametrize("pitch", [90.0, -90.0])
def test_gimbal_lock_reports_a_usable_rotation(pitch):
    """Roll and yaw are the same axis there; only the rotation has to survive."""
    quat = quaternion_from_rpy_deg(20.0, pitch, 35.0)
    back = rpy_deg_from_quaternion(*quat)
    assert back[0] == 0.0
    assert math.isclose(back[1], pitch, abs_tol=1e-6)
    assert_same_rotation(quaternion_from_rpy_deg(*back), quat, tol=1e-6)


def test_zero_quaternion_is_rejected():
    with pytest.raises(ValueError):
        rpy_deg_from_quaternion(0.0, 0.0, 0.0, 0.0)


def test_degrees_match_the_radians_used_by_bagfix():
    """The pom_4x4 lidar pitch, entered both ways, must agree."""
    assert_same_rotation(
        quaternion_from_rpy_deg(0.0, math.degrees(0.645772), 0.0),
        quaternion_from_rpy(0.0, 0.645772, 0.0),
    )


# ---- Settings ---------------------------------------------------------------

rclpy = pytest.importorskip("rclpy", reason="settings live in a node module")

from ninsar_tf_tools.tf_publisher import Settings  # noqa: E402

GOOD = dict(
    parent_frame="base_link", child_frame="lidar_link",
    x=1.0, y=0.0, z=0.5,
    roll_deg=0.0, pitch_deg=37.0, yaw_deg=0.0,
    publish_rate=30.0,
)


def test_good_settings_validate():
    assert Settings(**GOOD).validate() == ""


@pytest.mark.parametrize("override,expected", [
    ({"child_frame": "base_link"}, "own parent"),
    ({"parent_frame": "  "}, "parent_frame"),
    ({"publish_rate": 0.0}, "publish_rate"),
    ({"publish_rate": -5.0}, "publish_rate"),
    ({"pitch_deg": float("nan")}, "pitch_deg"),
    ({"x": float("inf")}, "x must be finite"),
])
def test_bad_settings_are_rejected(override, expected):
    assert expected in Settings(**{**GOOD, **override}).validate()


def test_snippets_carry_the_tuned_pose():
    settings = Settings(**GOOD)
    command = settings.static_transform_publisher_command()
    assert "--frame-id base_link --child-frame-id lidar_link" in command
    assert "--x 1.000000" in command

    snippet = settings.bagfix_snippet()
    assert "transform: tf_static" in snippet
    # bagfix takes radians, not the degrees shown in rqt
    assert f"{math.radians(37.0):.6f}" in snippet
