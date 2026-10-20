#!/usr/bin/env python3
"""Trajectory sources for the free-flying sensor rig.

Plain numpy/scipy: no Isaac Sim, no USD, so this module can be exercised
outside the simulator.

A trajectory is an (N, 8) float64 array with one row per frame:

    t, x, y, z, qw, qx, qy, qz

Times start at 0 and are spaced 1/fps apart. Quaternions are W-first, which is
what both USD (Gf.Quatd) and the Isaac core APIs expect.

Waypoints come either untimed, paced by a speed and a yaw rate
(from_waypoints), or with a time per waypoint that is replayed as recorded
(from_timed_waypoints) -- which is how a path recovered from a rosbag by
bag_to_waypoints.py keeps its real pauses and accelerations.
"""

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

# Segments shorter than this get a floor on their duration, so that duplicate
# waypoints cannot collapse the interpolation knots.
MIN_SEGMENT_SECONDS = 1e-3


def from_waypoints(waypoints, fps=30.0, speed=1.0, yaw_rate_deg=90.0, close_loop=False):
    """Interpolate 6-DoF waypoints into a dense, constant-rate trajectory.

    `waypoints` is a sequence of (x, y, z, roll_deg, pitch_deg, yaw_deg). Position
    is interpolated linearly and orientation is slerped.

    Each segment's duration is whichever is slower: covering the distance at
    `speed` m/s, or covering the rotation at `yaw_rate_deg` deg/s. That keeps a
    constant ground speed while still allowing waypoints that only rotate --
    repeat a position with a different yaw to turn in place.
    """
    # A single waypoint is enough to close a loop on, degenerate as that is.
    wp = _validate_waypoints(waypoints, minimum=1 if close_loop else 2)
    if close_loop:
        wp = np.vstack([wp, wp[:1]])
    if speed <= 0.0 or yaw_rate_deg <= 0.0 or fps <= 0.0:
        raise ValueError("fps, speed and yaw_rate_deg must all be positive")

    positions = wp[:, :3]
    rotations = Rotation.from_euler("xyz", wp[:, 3:6], degrees=True)

    # Time-parameterise rather than arc-length-parameterise, so that a segment
    # with zero translation but non-zero rotation still gets a sane duration.
    distances = np.linalg.norm(np.diff(positions, axis=0), axis=1)
    angles = np.array([(rotations[i].inv() * rotations[i + 1]).magnitude() for i in range(len(wp) - 1)])
    durations = np.maximum(distances / speed, angles / np.deg2rad(yaw_rate_deg))
    durations = np.maximum(durations, MIN_SEGMENT_SECONDS)

    knots = np.concatenate([[0.0], np.cumsum(durations)])
    return _resample(positions, rotations, knots, fps)


def from_timed_waypoints(waypoints, times, fps=30.0):
    """Resample 6-DoF waypoints onto a fixed-rate clock, keeping their own timing.

    Where from_waypoints() derives each segment's duration from a speed and a
    yaw rate, this preserves the timing it is given: `times` is one time in
    seconds per waypoint, and the result is sampled every 1/fps from the first
    to the last. Pauses, accelerations and recording gaps therefore survive,
    which is what a trajectory recovered from a rosbag needs.

    Times only have to increase; the first is the origin, so it need not be 0.
    """
    wp, knots = _validate_waypoints(waypoints), np.asarray(times, dtype=np.float64)
    if knots.shape != (len(wp),):
        raise ValueError(f"got {len(wp)} waypoints but times of shape {knots.shape}")
    if fps <= 0.0:
        raise ValueError("fps must be positive")
    if np.any(np.diff(knots) <= 0.0):
        bad = int(np.argmin(np.diff(knots))) + 1
        raise ValueError(f"waypoint times must strictly increase; waypoint {bad} does not")

    return _resample(
        wp[:, :3], Rotation.from_euler("xyz", wp[:, 3:6], degrees=True), knots - knots[0], fps
    )


def _validate_waypoints(waypoints, minimum=2):
    """Coerce to an (N, 6) float array of at least `minimum` rows."""
    wp = np.asarray(waypoints, dtype=np.float64)
    if wp.ndim != 2 or wp.shape[1] != 6:
        raise ValueError(f"waypoints must be (N, 6) of x,y,z,roll,pitch,yaw; got {wp.shape}")
    if len(wp) < minimum:
        raise ValueError(f"need at least {minimum} waypoints, got {len(wp)}")
    return wp


def _resample(positions, rotations, knots, fps):
    """Sample a keyframed pose at 1/fps: linear on position, slerp on rotation.

    `knots` are the times of `positions`/`rotations`, starting at 0.
    """
    total = float(knots[-1])

    n_samples = max(int(round(total * fps)) + 1, 2)
    t = np.linspace(0.0, total, n_samples)

    pos = np.stack([np.interp(t, knots, positions[:, i]) for i in range(3)], axis=1)
    quat_xyzw = Slerp(knots, rotations)(t).as_quat()
    quat_wxyz = quat_xyzw[:, [3, 0, 1, 2]]

    return np.concatenate([t[:, None], pos, quat_wxyz], axis=1)


def save(path, traj):
    """Write a trajectory to .npy."""
    np.save(path, np.asarray(traj, dtype=np.float64))


def load(path):
    """Read a trajectory from .npy, validating its shape."""
    traj = np.load(path)
    if traj.ndim != 2 or traj.shape[1] != 8:
        raise ValueError(f"expected an (N, 8) trajectory, got {traj.shape}")
    return traj.astype(np.float64)


def describe(traj):
    """One-line human summary, for logging."""
    duration = traj[-1, 0]
    span = traj[:, 1:4].max(axis=0) - traj[:, 1:4].min(axis=0)
    path_length = np.linalg.norm(np.diff(traj[:, 1:4], axis=0), axis=1).sum()
    return (
        f"{len(traj)} frames, {duration:.1f}s, {path_length:.1f}m travelled, "
        f"bbox {span[0]:.1f} x {span[1]:.1f} x {span[2]:.1f}m"
    )
