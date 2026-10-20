#!/usr/bin/env python3
"""Solver tests on synthetic detections with a known answer.

    python3 -m pytest src/ninsar_lidar_calib/test/test_solve.py -v
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ninsar_lidar_calib import detections as detections_mod  # noqa: E402
from ninsar_lidar_calib import solve as solve_mod  # noqa: E402
from ninsar_lidar_calib import transforms as tf  # noqa: E402

# The answer the solver has to find.
TRUE_EXTRINSIC = tf.from_xyz_rpy([0.35, -0.08, 1.42], [0.03, -0.06, 0.11])
TRUE_TARGET = tf.from_xyz_rpy([8.0, 3.0, 0.4], [0.0, 0.0, 0.7])
GROUND_Z = 0.0


def _make_detection(world_from_base: np.ndarray, with_ground: bool = True):
    """One perfect observation of the fixed target from a given robot pose."""
    world_from_lidar = world_from_base @ TRUE_EXTRINSIC
    lidar_from_world = np.linalg.inv(world_from_lidar)

    corner_point = (lidar_from_world @ np.append(TRUE_TARGET[:3, 3], 1.0))[:3]
    corner_axes = lidar_from_world[:3, :3] @ TRUE_TARGET[:3, :3]

    detection = detections_mod.Detection(
        stamp_ns=0,
        world_from_base=world_from_base,
        corner_point=corner_point,
        corner_axes=corner_axes,
    )
    if with_ground:
        # world +Z seen from the lidar, and the sensor's height above the ground
        detection.ground_normal = lidar_from_world[:3, :3] @ np.array([0.0, 0.0, 1.0])
        detection.ground_offset = world_from_lidar[2, 3] - GROUND_Z
    return detection


def _trajectory(bearings, radii, headings, centre=(8.0, 3.0)):
    """Robot poses around the target.

    `bearings` places the robot on the circle, `radii` sets its distance and
    `headings` is the offset from pointing straight at the target. Varying all
    three is what makes the extrinsic observable - see the degeneracy test.
    """
    poses = []
    for bearing, radius, heading in zip(bearings, radii, headings):
        x = centre[0] - radius * np.cos(bearing)
        y = centre[1] - radius * np.sin(bearing)
        poses.append(tf.from_xyz_rpy([x, y, 0.0], [0.0, 0.0, bearing + heading]))
    return poses


def _rich_detections(with_ground=True, count=14):
    """A good recording: the target lands somewhere different every scan."""
    bearings = np.linspace(-1.2, 1.2, count)
    radii = 4.0 + 3.0 * np.abs(np.sin(np.linspace(0.0, 2.5, count)))
    headings = 0.4 * np.cos(np.linspace(0.0, 4.0, count))
    poses = _trajectory(bearings, radii, headings)
    return [_make_detection(p, with_ground) for p in poses]


def _facing_at_fixed_distance(count=12):
    """A bad recording: same distance, always pointed at the target.

    The target then sits at an identical spot in base_link on every scan, which
    constrains the extrinsic no better than a single observation does.
    """
    bearings = np.linspace(-0.9, 0.9, count)
    poses = _trajectory(bearings, np.full(count, 6.0), np.zeros(count))
    return [_make_detection(p, True) for p in poses]


def test_recovers_the_known_extrinsic():
    """With base_height known, all six parameters come back."""
    dets = _rich_detections()
    guess = tf.from_xyz_rpy([0.0, 0.0, 1.0], [0.0, 0.0, 0.0])

    # base_link is at world z=0 in this synthetic setup and the ground is at
    # GROUND_Z, so base_link sits this far above it
    result = solve_mod.solve(dets, guess, base_height=-GROUND_Z)

    xyz, rpy = tf.to_xyz_rpy(result.extrinsic)
    true_xyz, true_rpy = tf.to_xyz_rpy(TRUE_EXTRINSIC)
    assert np.allclose(xyz, true_xyz, atol=1e-3), f"{xyz} != {true_xyz}"
    assert np.allclose(rpy, true_rpy, atol=1e-3), f"{rpy} != {true_rpy}"
    assert not result.unobservable, result.unobservable


def test_height_is_unobservable_without_base_height():
    """A real gauge freedom, and the solver must say so rather than guess.

    Raising the lidar, the ground plane and the target together leaves every
    measurement unchanged: a plane's normal does not move under translation, and
    a ground-driving robot only ever rotates about world z. So the lidar height
    is recoverable only once the ground's world height is pinned down.
    """
    dets = _rich_detections()
    guess = tf.from_xyz_rpy([0.0, 0.0, 1.0], [0.0, 0.0, 0.0])

    result = solve_mod.solve(dets, guess)  # no base_height

    assert "z" in result.unobservable
    # the fit still lands on an exact optimum - it is ambiguous, not wrong
    assert result.residual_rms < 1e-6
    # roll and pitch come from the ground plane and survive regardless
    _, rpy = tf.to_xyz_rpy(result.extrinsic)
    _, true_rpy = tf.to_xyz_rpy(TRUE_EXTRINSIC)
    assert np.allclose(rpy[:2], true_rpy[:2], atol=1e-3)

    report = solve_mod.format_report(result, "map", "base_link", "lidar")
    assert "--base-height" in report


def test_facing_at_fixed_distance_is_reported():
    """Circling the target at constant range while facing it teaches nothing.

    The target sits at the same base_link position every scan, so the fit is
    ambiguous even though it converges. It must be flagged, not trusted.
    """
    result = solve_mod.solve(
        _facing_at_fixed_distance(),
        tf.from_xyz_rpy([0.0, 0.0, 1.0], [0.0, 0.0, 0.0]),
        base_height=-GROUND_Z,
    )

    assert result.unobservable, (
        "a constant-range, always-facing trajectory should not report a "
        "fully observable fit"
    )


def test_corner_spread_collapses():
    dets = _rich_detections()
    guess = tf.from_xyz_rpy([0.0, 0.0, 1.0], [0.0, 0.0, 0.0])

    result = solve_mod.solve(dets, guess)

    # a wrong extrinsic scatters the corner; the right one collapses it
    assert result.spread_before > 0.3
    assert result.spread_after < 1e-3


def test_degenerate_trajectory_is_reported_not_answered():
    """A straight line with no yaw change cannot pin down every axis.

    Returning a confident-looking number there would be worse than saying so.
    """
    poses = [
        tf.from_xyz_rpy([x, 3.0, 0.0], [0.0, 0.0, 0.0])
        for x in np.linspace(0.0, 2.0, 8)
    ]
    dets = [_make_detection(p, with_ground=False) for p in poses]

    result = solve_mod.solve(dets, tf.from_xyz_rpy([0.0, 0.0, 1.0], [0.0, 0.0, 0.0]))

    assert result.unobservable, (
        "a pure-translation trajectory should not report a fully observable fit"
    )


def test_ground_plane_improves_conditioning():
    """The ground plane is what holds roll, pitch and height when motion is weak."""
    count = 6
    bearings = np.linspace(-0.15, 0.15, count)  # barely any motion around the target
    poses = _trajectory(bearings, np.full(count, 6.0), np.zeros(count))
    guess = tf.from_xyz_rpy([0.0, 0.0, 1.0], [0.0, 0.0, 0.0])

    without = solve_mod.solve([_make_detection(p, False) for p in poses], guess,
                              ground_weight=0.0)
    with_ground = solve_mod.solve([_make_detection(p, True) for p in poses], guess,
                                  ground_weight=10.0, base_height=-GROUND_Z)

    assert with_ground.condition_number < without.condition_number
    assert len(with_ground.unobservable) < len(without.unobservable)


def test_survives_realistic_measurement_noise():
    """A good recording with 1 cm detection noise must still calibrate."""
    rng = np.random.default_rng(1)
    dets = _rich_detections()
    for detection in dets:
        detection.corner_point = detection.corner_point + rng.normal(0.0, 0.01, 3)
        detection.ground_offset += rng.normal(0.0, 0.005)

    result = solve_mod.solve(dets, tf.from_xyz_rpy([0.0, 0.0, 1.0], [0.0, 0.0, 0.0]),
                             base_height=-GROUND_Z)

    xyz, rpy = tf.to_xyz_rpy(result.extrinsic)
    true_xyz, true_rpy = tf.to_xyz_rpy(TRUE_EXTRINSIC)
    assert np.allclose(xyz, true_xyz, atol=0.01), f"{xyz} != {true_xyz}"
    assert np.allclose(rpy, true_rpy, atol=0.005), f"{rpy} != {true_rpy}"
    # noise must not be mistaken for a degenerate recording
    assert not result.unobservable, result.unobservable


def test_needs_at_least_two_corner_detections():
    dets = [_make_detection(tf.from_xyz_rpy([0, 0, 0], [0, 0, 0]))]
    with pytest.raises(ValueError, match="at least 2"):
        solve_mod.solve(dets, np.eye(4))


def test_reads_the_detection_log(tmp_path):
    """The JSONL contract between the C++ detectors and this solver."""
    path = tmp_path / "detections.jsonl"
    record = {
        "stamp_ns": 1234,
        "world_from_base": {"t": [1.0, 2.0, 3.0],
                            "R": [1, 0, 0, 0, 1, 0, 0, 0, 1]},
        "ground": {"normal": [0.0, 0.0, 1.0], "offset": 1.4, "inliers": 900},
        "corner": {"point": [3.0, 0.1, -0.5],
                   "axes": [1, 0, 0, 0, 1, 0, 0, 0, 1],
                   "inliers": [300, 310, 290],
                   "extents": [0.6, 0.6, 0.6]},
    }
    path.write_text(json.dumps(record) + "\n")

    dets = detections_mod.load(path)

    assert len(dets) == 1
    assert dets[0].has_ground and dets[0].has_corner
    assert dets[0].ground_offset == 1.4
    assert np.allclose(dets[0].world_from_base[:3, 3], [1.0, 2.0, 3.0])
    assert np.allclose(dets[0].corner_point, [3.0, 0.1, -0.5])


def test_malformed_detections_are_an_error(tmp_path):
    path = tmp_path / "bad.jsonl"
    path.write_text('{"stamp_ns": 1}\n')  # missing world_from_base
    with pytest.raises(detections_mod.DetectionError, match="malformed"):
        detections_mod.load(path)


def test_report_mentions_the_bagfix_rule():
    result = solve_mod.solve(_rich_detections(),
                             tf.from_xyz_rpy([0.0, 0.0, 1.0], [0.0, 0.0, 0.0]))
    report = solve_mod.format_report(result, "map", "robot_base_link", "lidar")
    assert "transform: tf_static" in report
    assert "robot_base_link" in report
    assert "static_transform_publisher" in report
