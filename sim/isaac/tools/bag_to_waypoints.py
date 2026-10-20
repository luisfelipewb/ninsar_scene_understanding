#!/usr/bin/env python3
"""Recover a waypoint YAML from the TF tree of a ROS 2 bag.

Reads /tf and /tf_static, resolves root -> base at every timestamp the chain
between them is published, and writes the result as a timed waypoint config
that run_scene.py replays on the bag's own clock.

Only /tf and /tf_static are read, through a storage filter, so the size of the
bag's sensor data costs nothing -- a 17 GB mcap takes a few seconds.

This runs under the ROS 2 interpreter, NOT Isaac Sim's python.sh:

    source /opt/ros/jazzy/setup.bash
    python3 tools/bag_to_waypoints.py ~/path/to/bag --base robot_base_footprint \
        --out pom4x4_test.yaml

then, from the Isaac side:

    ./python.sh run_scene.py config/empty.yaml --waypoints config/waypoints/pom4x4_test.yaml

The path is recentred on its first pose by default, because a bag's map frame
is usually hundreds of metres from the origin of the scene USD, which would put
the rig outside it. --no-recentre keeps the original coordinates.

A real robot's z came from wheels on undulating ground, which will not line up
with the scene's terrain. --terrain replaces it with the ground height sampled
from a baked heightmap, so --z becomes a clearance above the terrain rather
than an absolute height:

    python3 tools/bag_to_waypoints.py ~/path/to/bag --base robot_base_footprint \
        --terrain assets/montoldre_heightmap.npz --z 1.0 --out pom4x4_test.yaml
"""

import argparse
import os
import sys

import numpy as np
import rclpy.duration
import rclpy.time
import tf2_ros
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
from scipy.spatial.transform import Rotation, Slerp
from tf2_msgs.msg import TFMessage

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PROJECT_DIR)

from rigsim import terrain_height, waypoint_config  # noqa: E402  (needs PROJECT_DIR on the path)

WAYPOINTS_DIR = os.path.join(PROJECT_DIR, "config", "waypoints")
REPO_DIR = os.path.dirname(os.path.dirname(PROJECT_DIR))


def shown(path):
    """A path for the file's provenance: repo-relative when inside the repo, so it's the same on every machine."""
    path = os.path.abspath(path)
    return os.path.relpath(path, REPO_DIR) if os.path.commonpath([path, REPO_DIR]) == REPO_DIR else path

TF_TOPICS = ["/tf", "/tf_static"]

# Defaults, in metres and degrees: how far the replayed path may drift from the
# recorded one once the waypoints in between have been dropped.
POS_TOL = 0.05
ANG_TOL_DEG = 1.0

# A gap longer than this is a recording dropout, not a sample interval. The path
# across it was never recorded, so it is reported and never decimated over.
GAP_SECONDS = 0.5


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bag", help="bag directory (the one holding metadata.yaml)")
    parser.add_argument("--root", default="map", help="frame the trajectory is expressed in")
    parser.add_argument("--base", required=True, help="frame to follow, e.g. robot_base_footprint")
    parser.add_argument("--out", required=True, help="output YAML; a bare name lands in config/waypoints/")
    parser.add_argument("--storage", default="mcap", help="rosbag2 storage plugin")
    parser.add_argument("--pos-tol", type=float, default=POS_TOL, help="metres of decimation error allowed")
    parser.add_argument("--ang-tol", type=float, default=ANG_TOL_DEG, help="degrees of decimation error allowed")
    parser.add_argument("--start", type=float, help="drop everything before this many seconds into the bag")
    parser.add_argument("--end", type=float, help="drop everything after this many seconds into the bag")
    parser.add_argument(
        "--z",
        type=float,
        help="override every z, e.g. to fly a ground-projected frame; with --terrain this "
        "becomes the clearance held above the ground instead of an absolute height",
    )
    parser.add_argument(
        "--terrain",
        metavar="HEIGHTMAP.npz",
        help="follow the scene's terrain: set every z to the ground height under it, plus --z "
        "(baked by tools/bake_heightmap.py, e.g. assets/montoldre_heightmap.npz)",
    )
    parser.add_argument(
        "--no-recentre",
        action="store_true",
        help="keep the bag's own coordinates instead of starting the path at the origin facing +x",
    )
    parser.add_argument(
        "--no-times",
        action="store_true",
        help="write geometry only, so the sim paces the path by --speed and --yaw-rate instead",
    )
    return parser.parse_args()


def log(message):
    print(f"[bag2wp] {message}", flush=True)


def read_tf(bag, storage_id):
    """Every /tf and /tf_static message in the bag, oldest first.

    Returns (transforms, statics): lists of TransformStamped. Both are small --
    a few thousand messages -- so holding them is cheaper than a second pass.
    """
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag, storage_id=storage_id), ConverterOptions("", ""))
    reader.set_filter(StorageFilter(topics=TF_TOPICS))

    transforms, statics = [], []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        message = deserialize_message(data, TFMessage)
        (statics if topic == "/tf_static" else transforms).extend(message.transforms)
    if not transforms and not statics:
        raise SystemExit(f"[bag2wp] {bag}: no {' or '.join(TF_TOPICS)} messages")
    return transforms, statics


def chain_frames(transforms, statics, root, base):
    """The frames from `base` up to `root`, and which links of it move.

    Raises if the two frames are not connected, naming what the bag does have --
    the usual cause is a frame name that is close but not right.
    """
    parents = {t.child_frame_id: t.header.frame_id for t in list(statics) + list(transforms)}
    moving = {t.child_frame_id for t in transforms}

    chain, dynamic, frame = [base], [], base
    while frame != root:
        if frame not in parents:
            known = sorted(set(parents) | set(parents.values()))
            raise SystemExit(
                f"[bag2wp] no transform chain from {root!r} to {base!r}: {frame!r} has no parent.\n"
                f"         frames in this bag: {', '.join(known)}"
            )
        if frame in moving:
            dynamic.append(frame)
        frame = parents[frame]
        if frame in chain:
            raise SystemExit(f"[bag2wp] frame cycle at {frame!r}")
        chain.append(frame)

    if not dynamic:
        raise SystemExit(
            f"[bag2wp] {root!r} -> {base!r} is entirely static, so there is no trajectory to extract"
        )
    return chain, dynamic


def sample_times(transforms, dynamic):
    """Timestamps to look the chain up at: where its moving links are published.

    Sampling on the moving links only, rather than on every /tf message, keeps
    unrelated high-rate frames (wheels, say) from padding the trajectory with
    interpolated duplicates.
    """
    moving = set(dynamic)
    stamps = {
        rclpy.time.Time.from_msg(t.header.stamp).nanoseconds
        for t in transforms
        if t.child_frame_id in moving
    }
    return sorted(stamps)


def lookup_poses(transforms, statics, root, base, stamps):
    """Resolve root -> base at each stamp, through tf2's own interpolation.

    The buffer caches the whole bag, so transforms can all go in before any
    lookup; letting tf2 walk the chain means a base link several static hops
    from the odometry frame costs nothing extra here.
    """
    span = (stamps[-1] - stamps[0]) * 1e-9
    buffer = tf2_ros.BufferCore(rclpy.duration.Duration(seconds=span + 60.0))
    for transform in statics:
        buffer.set_transform_static(transform, "bag")
    for transform in transforms:
        buffer.set_transform(transform, "bag")

    poses, skipped = [], 0
    for stamp in stamps:
        try:
            found = buffer.lookup_transform_core(root, base, rclpy.time.Time(nanoseconds=stamp))
        except tf2_ros.TransformException:
            # Usually the first or last stamp, where one link of the chain does
            # not yet (or no longer) bracket the time being asked for.
            skipped += 1
            continue
        p, q = found.transform.translation, found.transform.rotation
        poses.append([stamp * 1e-9, p.x, p.y, p.z, q.x, q.y, q.z, q.w])

    if len(poses) < 2:
        raise SystemExit(f"[bag2wp] only {len(poses)} of {len(stamps)} stamps resolved; nothing to write")
    return np.asarray(poses, dtype=np.float64), skipped


def to_waypoints(poses):
    """(t, x, y, z, qx, qy, qz, qw) rows -> times and (x, y, z, r, p, y) degrees.

    Yaw is unwrapped, so that a pair of waypoints straddling +-180 degrees is
    interpolated the short way round rather than spun through a full turn.
    """
    times = poses[:, 0] - poses[0, 0]
    euler = Rotation.from_quat(poses[:, 4:8]).as_euler("xyz", degrees=True)
    euler = np.unwrap(euler, period=360.0, axis=0)
    return times, np.concatenate([poses[:, 1:4], euler], axis=1)


def recentre(waypoints):
    """Move the path so it starts at the origin heading +x, keeping its shape.

    Extrinsic xyz euler is Rz*Ry*Rx, so pre-rotating about world z is exactly a
    subtraction on yaw; roll and pitch are untouched.
    """
    yaw0 = np.deg2rad(waypoints[0, 5])
    rotation = np.array([[np.cos(-yaw0), -np.sin(-yaw0)], [np.sin(-yaw0), np.cos(-yaw0)]])

    out = waypoints.copy()
    out[:, :3] -= waypoints[0, :3]
    out[:, :2] = out[:, :2] @ rotation.T
    out[:, 5] -= waypoints[0, 5]
    return out


def follow_terrain(heightmap, waypoints, clearance):
    """Every z replaced by the ground height under it, plus `clearance`.

    Call this before decimation, not after: --pos-tol then measures the
    replayed path against a terrain-following one, so the decimator keeps
    whatever waypoints the relief needs rather than cutting a straight line
    through a rise it never sampled.

    The recentring has to have happened first too -- a bag's map frame is
    usually hundreds of metres from the scene's origin, and sampling the
    terrain at raw bag coordinates would read the wrong part of the field.
    """
    terrain = terrain_height.TerrainHeight(heightmap)
    x, y = waypoints[:, 0], waypoints[:, 1]

    outside = terrain.outside(x, y)
    if outside.any():
        x0, x1, y0, y1 = terrain.bounds
        log(
            f"WARNING {outside.sum()} of {len(waypoints)} waypoints fall outside the terrain "
            f"(x {x0:.1f}..{x1:.1f}, y {y0:.1f}..{y1:.1f}) and were held at its nearest edge"
        )

    ground = terrain(x, y)
    log(f"terrain under the path: {ground.min():.2f}..{ground.max():.2f}m, {clearance:g}m clearance")
    return ground + clearance


def split_at_gaps(times, threshold):
    """Index ranges between recording dropouts, so decimation cannot span one."""
    breaks = [0, *(np.flatnonzero(np.diff(times) > threshold) + 1), len(times)]
    return [(breaks[i], breaks[i + 1]) for i in range(len(breaks) - 1)]


def decimate(times, waypoints, pos_tol, ang_tol_deg):
    """Drop every waypoint the interpolator can reconstruct within tolerance.

    Greedy: extend a segment while replaying it -- linear on position, slerp on
    rotation, on the same clock the sim will use -- stays within tolerance of
    every sample it swallows. The tolerances are therefore the actual error of
    the replayed path against the recorded one, not a proxy for it.
    """
    rotations = Rotation.from_euler("xyz", waypoints[:, 3:6], degrees=True)

    keep, start = [0], 0
    while start < len(waypoints) - 1:
        end, furthest = start + 2, start + 1
        while end < len(waypoints):
            if not within_tolerance(times, waypoints, rotations, start, end, pos_tol, ang_tol_deg):
                break
            furthest, end = end, end + 1
        keep.append(furthest)
        start = furthest
    return np.asarray(keep)


def within_tolerance(times, waypoints, rotations, start, end, pos_tol, ang_tol_deg):
    """Would waypoints start..end survive being replaced by their endpoints?"""
    span = times[start:end + 1]
    fraction = (span - span[0]) / (span[-1] - span[0])

    ends = waypoints[[start, end], :3]
    straight = ends[0] + np.outer(fraction, ends[1] - ends[0])
    if np.linalg.norm(waypoints[start:end + 1, :3] - straight, axis=1).max() > pos_tol:
        return False

    slerped = Slerp([0.0, 1.0], rotations[[start, end]])(fraction)
    error = np.degrees((slerped.inv() * rotations[start:end + 1]).magnitude()).max()
    return error <= ang_tol_deg


def describe(times, waypoints, label):
    steps = np.linalg.norm(np.diff(waypoints[:, :3], axis=0), axis=1)
    dt = np.diff(times)
    moving = dt > 0.0
    return (
        f"{label}: {len(waypoints)} poses, {times[-1] - times[0]:.1f}s, {steps.sum():.1f}m, "
        f"median speed {np.median(steps[moving] / dt[moving]):.2f} m/s"
    )


def main():
    args = parse_args()
    out_path = args.out if os.path.dirname(args.out) else os.path.join(WAYPOINTS_DIR, args.out)

    transforms, statics = read_tf(args.bag, args.storage)
    log(f"read {len(transforms)} /tf and {len(statics)} /tf_static transforms")

    chain, dynamic = chain_frames(transforms, statics, args.root, args.base)
    log(f"chain {' <- '.join(chain)}  (moving: {', '.join(dynamic)})")

    stamps = sample_times(transforms, dynamic)
    poses, skipped = lookup_poses(transforms, statics, args.root, args.base, stamps)
    if skipped:
        log(f"{skipped} of {len(stamps)} stamps did not resolve and were skipped")

    times, waypoints = to_waypoints(poses)
    log(describe(times, waypoints, "recorded"))

    window = (times >= (args.start or -np.inf)) & (times <= (args.end if args.end is not None else np.inf))
    if window.sum() < 2:
        raise SystemExit(f"[bag2wp] --start/--end keep only {window.sum()} of {len(times)} poses")
    if not window.all():
        times, waypoints = times[window] - times[window][0], waypoints[window]
        log(f"trimmed to {len(times)} poses over {times[-1]:.1f}s")

    gaps = [(times[i], times[i + 1] - times[i]) for i in np.flatnonzero(np.diff(times) > GAP_SECONDS)]
    for when, length in gaps:
        moved = np.linalg.norm(waypoints[np.searchsorted(times, when) + 1, :3] - waypoints[np.searchsorted(times, when), :3])
        log(f"WARNING gap at t={when:.1f}s: {length:.1f}s and {moved:.1f}m unrecorded, interpolated straight")

    if not args.no_recentre:
        waypoints = recentre(waypoints)
    if args.terrain is not None:
        waypoints[:, 2] = follow_terrain(args.terrain, waypoints, args.z or 0.0)
    elif args.z is not None:
        waypoints[:, 2] = args.z

    recorded = len(times)
    keep = np.concatenate(
        [
            lo + decimate(times[lo:hi], waypoints[lo:hi], args.pos_tol, args.ang_tol)
            for lo, hi in split_at_gaps(times, GAP_SECONDS)
        ]
    )
    times, waypoints = times[keep], waypoints[keep]
    log(f"decimated to {len(keep)} waypoints within {args.pos_tol}m / {args.ang_tol}deg")

    close = np.flatnonzero(np.diff(times) < waypoint_config.MIN_TIME_SPACING)
    if len(close):
        raise SystemExit(f"[bag2wp] waypoints {close[:5]} are under 1 ms apart; they cannot be written")

    # Only a chain worth spelling out earns a line; a single hop is its own.
    hops = f" via {' <- '.join(chain[1:-1])}" if len(chain) > 2 else ""
    comments = [
        f"Extracted by bag_to_waypoints.py from {shown(args.bag)}",
        f"{args.root} -> {args.base}{hops}",
        f"{recorded} recorded poses decimated to {len(keep)} waypoints "
        f"within {args.pos_tol}m / {args.ang_tol}deg",
        describe(times, waypoints, "replays as"),
    ]
    comments += [
        f"WARNING recording gap at t={when:.1f}s lasting {length:.1f}s; the path there is interpolated"
        for when, length in gaps
    ]
    source = {
        "bag": shown(args.bag),
        "root_frame": args.root,
        "base_frame": args.base,
        "recentred": not args.no_recentre,
        "timed": not args.no_times,
    }
    if args.terrain is not None:
        source["terrain"] = shown(args.terrain)
        source["clearance"] = args.z or 0.0
        comments.append(f"z follows {shown(args.terrain)} at {args.z or 0.0:g}m clearance")

    written = waypoint_config.save_waypoints(
        out_path, [tuple(w) for w in waypoints], None if args.no_times else list(times), comments, source
    )

    # Round-tripping through the reader the simulator uses is the only check
    # that matters: rounding to millimetres could break the times' ordering.
    reloaded, reloaded_times = waypoint_config.load_waypoints(written)
    if len(reloaded) != len(waypoints):
        raise SystemExit(f"[bag2wp] wrote {len(waypoints)} waypoints but read back {len(reloaded)}")
    error = np.abs(np.asarray(reloaded) - waypoints).max()
    log(f"verified reload of {len(reloaded)} waypoints, max rounding error {error:.4f}")

    log(f"wrote {written}")
    if args.no_times:
        log(f"untimed: replay with --speed {np.median(np.linalg.norm(np.diff(waypoints[:, :3], axis=0), axis=1) / np.diff(times)):.2f}")
    log(f"run: ./python.sh {os.path.join(PROJECT_DIR, 'run_scene.py')} config/<scene>.yaml --waypoints {written}")


if __name__ == "__main__":
    main()
