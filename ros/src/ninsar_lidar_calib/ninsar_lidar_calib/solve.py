#!/usr/bin/env python3
"""Solve the base_link -> lidar extrinsic from a detections file.

The corner target does not move in the world, so with the RTK pose known, any
apparent motion of the detected corner is extrinsic error. Minimizing it over
all scans recovers the extrinsic; the ground plane pins down roll, pitch and
height, which a poor trajectory would otherwise leave loose.

    ros2 run ninsar_lidar_calib solve detections.jsonl
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import least_squares

try:  # installed package, or run straight from the source tree
    from ninsar_lidar_calib import detections as detections_mod
    from ninsar_lidar_calib import transforms as tf
except ImportError:  # pragma: no cover - convenience for direct execution
    import detections as detections_mod  # type: ignore
    import transforms as tf  # type: ignore

WORLD_UP = np.array([0.0, 0.0, 1.0])


@dataclass
class Result:
    extrinsic: np.ndarray                  # 4x4, base_link <- lidar
    target: np.ndarray                     # 4x4, world <- corner target
    spread_before: float
    spread_after: float
    residual_rms: float
    condition_number: float
    stddev: np.ndarray                     # per-parameter, xyz then rotvec
    unobservable: list[str] = field(default_factory=list)
    used_ground: int = 0
    used_corner: int = 0
    #: True when base_height pinned the ground, making z observable
    fixed_ground: bool = False


def corner_spread(dets, extrinsic: np.ndarray) -> tuple[float, np.ndarray]:
    """Max distance of the corner estimates from their mean, in the world frame.

    This is the number the whole calibration is trying to drive to zero.
    """
    points = np.array([
        (d.world_from_base @ extrinsic @ np.append(d.corner_point, 1.0))[:3]
        for d in dets if d.has_corner
    ])
    if len(points) == 0:
        return 0.0, points
    mean = points.mean(axis=0)
    return float(np.max(np.linalg.norm(points - mean, axis=1))), points


def _residuals(params: np.ndarray, dets, ground_weight: float,
               base_height: float | None) -> np.ndarray:
    extrinsic = tf.from_params(params[0:6])
    target = tf.from_params(params[6:12])
    target_rotation_t = target[:3, :3].T
    target_translation = target[:3, 3]

    out: list[np.ndarray] = []
    for detection in dets:
        world_from_lidar = detection.world_from_base @ extrinsic

        # Where the ground sits in the world. Known from the RTK height when
        # base_height is given; otherwise a free parameter - see solve().
        if base_height is None:
            ground_z = params[12]
        else:
            ground_z = detection.world_from_base[2, 3] - base_height

        if detection.has_corner:
            # the corner must land on the same world point in every scan
            point = (world_from_lidar @ np.append(detection.corner_point, 1.0))[:3]
            out.append(point - target_translation)
            # ... and with the same orientation, which is what makes the lidar
            # rotation observable from a single scan rather than from motion
            out.append(tf.rotation_log(
                target_rotation_t @ world_from_lidar[:3, :3] @ detection.corner_axes))

        if detection.has_ground and ground_weight > 0.0:
            normal = world_from_lidar[:3, :3] @ detection.ground_normal
            # the ground is horizontal in the world: its normal is world +Z
            out.append(ground_weight * np.cross(normal, WORLD_UP))
            # The sensor's measured height above the ground must agree with its
            # world height. With the normal pointed up, the plane's offset is
            # exactly that measured height.
            measured_height = detection.ground_offset
            out.append(np.array([
                ground_weight * (world_from_lidar[2, 3] - ground_z - measured_height)
            ]))

    return np.concatenate(out) if out else np.zeros(1)


def _initial_guess(dets, extrinsic_guess: np.ndarray,
                   base_height: float | None) -> np.ndarray:
    """Seed the optimizer: extrinsic as given, target and ground from the data."""
    target = np.eye(4)
    for detection in dets:
        if detection.has_corner:
            world_from_lidar = detection.world_from_base @ extrinsic_guess
            target[:3, 3] = (world_from_lidar @ np.append(detection.corner_point, 1.0))[:3]
            target[:3, :3] = world_from_lidar[:3, :3] @ detection.corner_axes
            break

    params = [tf.to_params(extrinsic_guess), tf.to_params(target)]
    if base_height is None:
        heights = [
            (detection.world_from_base @ extrinsic_guess)[2, 3] - detection.ground_offset
            for detection in dets if detection.has_ground
        ]
        params.append([float(np.mean(heights)) if heights else 0.0])
    return np.concatenate(params)


def solve(dets, extrinsic_guess: np.ndarray, ground_weight: float = 1.0,
          loss: str = "soft_l1", base_height: float | None = None) -> Result:
    """Least-squares fit of the extrinsic and the target's world pose.

    `base_height` is the height of base_link above the ground, in metres - a
    robot constant, usually straight out of the URDF. Supplying it is what makes
    the lidar's own height observable.

    Without it the ground's world height is a free parameter, and raising the
    lidar, the ground and the target together changes nothing that is measured -
    a genuine gauge freedom, since a ground-plane normal is unchanged by
    translation and the robot only ever rotates about the world z axis. The fit
    then still succeeds, but `z` comes back flagged in `unobservable`.
    """
    used_corner = sum(1 for d in dets if d.has_corner)
    used_ground = sum(1 for d in dets if d.has_ground)
    if used_corner < 2:
        raise ValueError(
            f"need at least 2 corner detections to calibrate, got {used_corner}"
        )
    if base_height is not None and used_ground == 0:
        raise ValueError(
            "base_height was given but no detection carries a ground plane; "
            "enable the ground stage or drop base_height"
        )

    spread_before, _ = corner_spread(dets, extrinsic_guess)

    x0 = _initial_guess(dets, extrinsic_guess, base_height)
    fit = least_squares(
        _residuals, x0, args=(dets, ground_weight, base_height), loss=loss,
        f_scale=0.05, method="trf", max_nfev=500,
    )

    extrinsic = tf.from_params(fit.x[0:6])
    target = tf.from_params(fit.x[6:12])
    spread_after, _ = corner_spread(dets, extrinsic)

    stddev, condition, unobservable = _uncertainty(fit)

    return Result(
        extrinsic=extrinsic,
        target=target,
        spread_before=spread_before,
        spread_after=spread_after,
        residual_rms=float(np.sqrt(np.mean(fit.fun ** 2))),
        condition_number=condition,
        stddev=stddev,
        unobservable=unobservable,
        used_ground=used_ground,
        used_corner=used_corner,
        fixed_ground=base_height is not None,
    )


#: A singular value this far below the largest means the direction is not
#: constrained by the data at all.
RANK_TOLERANCE = 1e-7
#: How much of a null direction must fall on a parameter before it is named.
NULL_SHARE = 0.1

PARAM_NAMES = ["x", "y", "z", "rot_x", "rot_y", "rot_z"]


def _uncertainty(fit) -> tuple[np.ndarray, float, list[str]]:
    """Which extrinsic parameters this recording actually pins down.

    Observability is read off the Jacobian's null space rather than from
    residual-scaled standard deviations. A degenerate recording still fits
    perfectly - it is ambiguous, not noisy - so residual size says nothing about
    it, while a vanishing singular value says everything. Reporting this is the
    point: a confident-looking number from an unconstrained direction is worse
    than no answer.
    """
    jacobian = fit.jac
    _, singular, vt = np.linalg.svd(jacobian, full_matrices=False)

    largest = singular[0] if singular.size else 0.0
    smallest = singular[-1] if singular.size else 0.0
    condition = float(largest / smallest) if smallest > 0 else np.inf

    unobservable: list[str] = []
    if largest > 0:
        for direction in vt[singular < RANK_TOLERANCE * largest]:
            extrinsic_part = np.abs(direction[:6])
            peak = extrinsic_part.max(initial=0.0)
            if peak <= NULL_SHARE:
                continue  # this ambiguity lives in the target pose, not the extrinsic
            unobservable += [
                name for name, weight in zip(PARAM_NAMES, extrinsic_part)
                if weight > NULL_SHARE * peak and name not in unobservable
            ]

    # Standard deviations are still reported, but only as a secondary hint -
    # they are meaningful once the fit carries real measurement noise.
    residual_count, param_count = jacobian.shape
    degrees_of_freedom = max(residual_count - param_count, 1)
    variance = float(np.sum(fit.fun ** 2) / degrees_of_freedom)
    try:
        covariance = np.linalg.pinv(jacobian.T @ jacobian) * variance
        stddev = np.sqrt(np.abs(np.diag(covariance)))[:6]
    except np.linalg.LinAlgError:
        stddev = np.full(6, np.inf)

    return stddev, condition, unobservable


def format_report(result: Result, world_frame: str, base_frame: str,
                  lidar_frame: str) -> str:
    xyz, rpy = tf.to_xyz_rpy(result.extrinsic)
    degrees = np.degrees(rpy)

    lines = [
        f"detections used: {result.used_corner} corner, {result.used_ground} ground",
        "",
        "calibrated base_link -> lidar",
        f"  xyz : [{xyz[0]:.5f}, {xyz[1]:.5f}, {xyz[2]:.5f}]  m",
        f"  rpy : [{rpy[0]:.6f}, {rpy[1]:.6f}, {rpy[2]:.6f}]  rad"
        f"   ({degrees[0]:.3f}, {degrees[1]:.3f}, {degrees[2]:.3f} deg)",
        "",
        f"corner spread in {world_frame}: "
        f"{result.spread_before:.4f} m -> {result.spread_after:.4f} m",
        f"residual rms: {result.residual_rms:.5f}",
        f"jacobian condition number: {result.condition_number:.3g}",
        "  1-sigma: " + ", ".join(
            f"{name}={value:.4f}"
            for name, value in zip(["x", "y", "z", "rx", "ry", "rz"], result.stddev)
        ),
    ]

    if result.unobservable:
        lines += [
            "",
            "WARNING: these parameters are not constrained by this recording: "
            + ", ".join(result.unobservable),
            "  Drive the robot around the target with more varied heading and",
            "  distance, or keep the ground plane enabled, and record again.",
        ]
        if "z" in result.unobservable and result.used_ground and not result.fixed_ground:
            lines += [
                "  For z specifically: pass --base-height (base_link above the",
                "  ground, from the URDF). Without it, raising the lidar, the",
                "  ground and the target together changes nothing measurable,",
                "  so the height cannot be recovered from this data alone.",
            ]

    quaternion = tf.quaternion_from_matrix(result.extrinsic)
    lines += [
        "",
        "paste into a bagfix config to patch recorded bags:",
        "",
        "  - transform: tf_static",
        "    edit:",
        f"      - parent: {base_frame}",
        f"        child: {lidar_frame}",
        f"        xyz: [{xyz[0]:.6f}, {xyz[1]:.6f}, {xyz[2]:.6f}]",
        f"        rpy: [{rpy[0]:.6f}, {rpy[1]:.6f}, {rpy[2]:.6f}]",
        "",
        "    ('edit' expects the bag to already carry this transform, which is"
        " where the",
        "     initial guess came from. Use 'add' instead if it does not.)",
        "",
        "or as a static_transform_publisher:",
        f"  ros2 run tf2_ros static_transform_publisher "
        f"{xyz[0]:.6f} {xyz[1]:.6f} {xyz[2]:.6f} "
        f"{quaternion[0]:.6f} {quaternion[1]:.6f} {quaternion[2]:.6f} "
        f"{quaternion[3]:.6f} {base_frame} {lidar_frame}",
    ]
    return "\n".join(lines)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("detections", help="JSONL file from detect_from_bag or calib_node")
    parser.add_argument("--xyz", type=float, nargs=3, default=[0.0, 0.0, 0.0],
                        help="initial base_link->lidar translation guess")
    parser.add_argument("--rpy", type=float, nargs=3, default=[0.0, 0.0, 0.0],
                        help="initial base_link->lidar rotation guess, radians")
    parser.add_argument("--ground-weight", type=float, default=1.0,
                        help="weight of the ground-plane residuals (0 disables them)")
    parser.add_argument("--base-height", type=float, default=None,
                        help="height of base_link above the ground, in metres "
                             "(from the URDF). Without it the lidar's own height "
                             "is not observable and comes back flagged.")
    parser.add_argument("--loss", default="soft_l1",
                        choices=["linear", "soft_l1", "huber", "cauchy"],
                        help="robust loss; the default survives bad detections")
    parser.add_argument("--world-frame", default="map")
    parser.add_argument("--base-frame", default="robot_base_link")
    parser.add_argument("--lidar-frame", default="lidar")
    args = parser.parse_args(argv)

    try:
        dets = detections_mod.load(args.detections)
        result = solve(dets, tf.from_xyz_rpy(args.xyz, args.rpy),
                       ground_weight=args.ground_weight, loss=args.loss,
                       base_height=args.base_height)
    except (detections_mod.DetectionError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    print(format_report(result, args.world_frame, args.base_frame, args.lidar_frame))
    return 2 if result.unobservable else 0


if __name__ == "__main__":
    raise SystemExit(main())
