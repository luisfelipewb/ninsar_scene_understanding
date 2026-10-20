#!/usr/bin/env python3
"""Stitch waypoint files and loose x,y points into one smooth tour.

Takes a sequence of stops in driving order, each either a waypoint YAML
(replayed as it is) or a bare `x,y` point, and writes a single waypoint file
that visits them in turn. The stretches between stops are cubic curves whose
direction at each end matches the direction the rig already has there --
the yaw the file ends or begins with, or the bend at a loose point -- so the
heading never jumps and the rig never moves sideways. Only numpy and PyYAML,
so it runs under the system interpreter:

    python3 tools/tour_from_points.py config/waypoints/lemon_v4.yaml -40,11 \\
        config/waypoints/cabbage_v4.yaml 8,-100 -7,-76 1.5,-11 35,3.5 99,27 --out tour_01.yaml

Each stretch is a cubic Hermite curve between two knots, with the tangent at a
knot pointing along the rig's heading there and scaled by the chord length,
which is the classic Catmull-Rom compromise between a tight corner and a wide
loop. At a loose point the heading is the bisector of the chords either side,
so the curve flows through it; at the first or last point of the tour it is
simply the chord direction. The curves are sampled every --spacing metres
with the yaw taken from the curve's own tangent; the replay interpolates
positions linearly between waypoints, so the finer the spacing, the smaller
the angle between the straight segment and the yaw across it. The minimum turn
radius of the sampled tour is reported, and turns tighter than --min-radius
raise a warning: move or add an intermediate point to open them up.

z is not planned here: waypoints from a file keep their z and loose points
take --z (default 0). Set `trajectory.terrain` in the run config to drive on
the ground.
"""

import argparse
import os
import re
import sys

import numpy as np

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PROJECT_DIR)

from rigsim import waypoint_config  # noqa: E402  (needs PROJECT_DIR on the path)

WAYPOINTS_DIR = os.path.join(PROJECT_DIR, "config", "waypoints")

POINT = re.compile(r"^\s*-?\d+(\.\d*)?,-?\d+(\.\d*)?$")

# Metres between the samples of a linking curve. The straight segment between
# two samples sits within a degree or two of the yaw across it at this spacing
# for turns of a few metres radius.
SPACING = 0.5

# A turn tighter than this is worth a warning: the rig's own turns (the
# semicircles between crop rows) have a radius of about 1 to 2.5 m.
MIN_RADIUS = 1.5


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("stops", nargs="+", help="waypoint YAML files and `x,y` points, in driving order")
    parser.add_argument("--out", required=True, help="output YAML; a bare name lands in config/waypoints/")
    parser.add_argument("--spacing", type=float, default=SPACING, help="metres between samples of a linking curve")
    parser.add_argument("--z", type=float, default=0.0, help="z of the loose points (default 0)")
    parser.add_argument("--min-radius", type=float, default=MIN_RADIUS, help="warn about turns tighter than this")
    # argparse reads `-40,11` as an option; a leading space hides the dash
    # and is stripped again when the stop is parsed.
    args = parser.parse_args([" " + arg if POINT.match(arg) else arg for arg in sys.argv[1:]])
    args.stops = [stop.strip() for stop in args.stops]
    if len(args.stops) < 2:
        parser.error("need at least two stops")
    if args.spacing <= 0.0:
        parser.error("--spacing must be positive")
    if os.sep not in args.out:
        args.out = os.path.join(WAYPOINTS_DIR, args.out)
    return args


def log(message):
    print(message, file=sys.stderr)


def parse_stops(stops, z):
    """Each stop as an (N, 6) array of waypoints: a file's, or one row for an `x,y` point."""
    parsed = []
    for stop in stops:
        if os.path.isfile(stop):
            waypoints, _ = waypoint_config.load_waypoints(stop)
            parsed.append(np.asarray(waypoints, dtype=np.float64))
            continue
        try:
            x, y = (float(value) for value in stop.split(","))
        except ValueError:
            raise SystemExit(f"{stop}: neither a waypoint file nor an x,y point")
        parsed.append(np.array([[x, y, z, 0.0, 0.0, np.nan]]))  # yaw decided once the neighbours are known
    return parsed


def heading_vector(yaw_deg):
    return np.array([np.cos(np.radians(yaw_deg)), np.sin(np.radians(yaw_deg))])


def knot_directions(stops):
    """Unit (entry, exit) direction at every stop, in the plane.

    A file's directions are its first and last yaw. A loose point gets one
    direction for both: the bisector of the chords to its neighbours, or the
    chord alone at either end of the tour.
    """
    entries, exits = [], []
    for index, stop in enumerate(stops):
        if not np.isnan(stop[0, 5]):
            entries.append(heading_vector(stop[0, 5]))
            exits.append(heading_vector(stop[-1, 5]))
            continue
        chords = []
        if index > 0:
            chords.append(stop[0, :2] - stops[index - 1][-1, :2])
        if index + 1 < len(stops):
            chords.append(stops[index + 1][0, :2] - stop[0, :2])
        direction = sum(chord / np.linalg.norm(chord) for chord in chords)
        if np.linalg.norm(direction) < 1e-6:  # a hairpin: the chords cancel, so keep going straight through
            direction = chords[0]
        direction = direction / np.linalg.norm(direction)
        entries.append(direction)
        exits.append(direction)
    return entries, exits


def hermite(p0, d0, p1, d1, spacing):
    """Samples along the cubic from p0 heading d0 to p1 heading d1, as (points, headings).

    The first sample is p0 itself and the last is p1; the tangent at each is
    the direction given, scaled by the chord length. Sampling is uniform in
    the curve parameter, refined until no step exceeds `spacing`.
    """
    chord = np.linalg.norm(p1 - p0)
    m0, m1 = d0 * chord, d1 * chord
    count = max(2, int(np.ceil(chord / spacing)) + 1)
    while True:
        t = np.linspace(0.0, 1.0, count)[:, None]
        h00, h10 = 2 * t**3 - 3 * t**2 + 1, t**3 - 2 * t**2 + t
        h01, h11 = -2 * t**3 + 3 * t**2, t**3 - t**2
        points = h00 * p0 + h10 * m0 + h01 * p1 + h11 * m1
        steps = np.linalg.norm(np.diff(points, axis=0), axis=1)
        if steps.max() <= spacing:
            break
        count = int(np.ceil(count * steps.max() / spacing)) + 1
    d00, d10 = 6 * t**2 - 6 * t, 3 * t**2 - 4 * t + 1
    d01, d11 = -6 * t**2 + 6 * t, 3 * t**2 - 2 * t
    tangents = d00 * p0 + d10 * m0 + d01 * p1 + d11 * m1
    headings = np.degrees(np.arctan2(tangents[:, 1], tangents[:, 0]))
    return points, headings


def stitch(stops, spacing):
    """The whole tour as (N, 6) waypoints, plus a parallel list of the stop index or 'link'."""
    entries, exits = knot_directions(stops)
    rows, kinds = [], []
    for index, stop in enumerate(stops):
        if index > 0:
            previous = stops[index - 1]
            points, headings = hermite(previous[-1, :2], exits[index - 1], stop[0, :2], entries[index], spacing)
            z = np.linspace(previous[-1, 2], stop[0, 2], len(points))
            for point, height, heading in list(zip(points, z, headings))[1:-1]:  # ends are the stops themselves
                rows.append([point[0], point[1], height, 0.0, 0.0, heading])
                kinds.append("link")
        block = stop.copy()
        if np.isnan(block[0, 5]):
            block[0, 5] = np.degrees(np.arctan2(entries[index][1], entries[index][0]))
        rows.extend(block.tolist())
        kinds.extend([index] * len(block))
    waypoints = np.asarray(rows)
    waypoints[:, 5] = np.degrees(np.unwrap(np.radians(waypoints[:, 5])))
    return waypoints, kinds


def check(waypoints, kinds, min_radius):
    """Largest angle between a segment and the yaw across it, and the tightest turn, on the links.

    The files are replayed as they are, so only the segments this tool built
    -- those with a link sample at either end -- are judged. Both figures are
    also reported for the whole tour, for information.
    """
    xy = waypoints[:, :2]
    chords = np.diff(xy, axis=0)
    chord_yaw = np.degrees(np.arctan2(chords[:, 1], chords[:, 0]))
    mean_yaw = 0.5 * (waypoints[:-1, 5] + waypoints[1:, 5])
    slip = np.abs((chord_yaw - mean_yaw + 180.0) % 360.0 - 180.0)
    slip[np.linalg.norm(chords, axis=1) < 1e-9] = 0.0

    # Turn radius at each interior sample from its two neighbours: the circumradius.
    a, b, c = xy[:-2], xy[1:-1], xy[2:]
    ab, bc, ca = np.linalg.norm(b - a, axis=1), np.linalg.norm(c - b, axis=1), np.linalg.norm(a - c, axis=1)
    cross = np.abs((b - a)[:, 0] * (c - b)[:, 1] - (b - a)[:, 1] * (c - b)[:, 0])
    with np.errstate(divide="ignore", invalid="ignore"):
        radius = np.where(cross > 1e-9, ab * bc * ca / (2.0 * cross), np.inf)

    is_link = np.array([kind == "link" for kind in kinds])
    link_segment = is_link[:-1] | is_link[1:]
    link_corner = is_link[1:-1]
    link_slip = slip[link_segment].max() if link_segment.any() else 0.0
    link_radius = radius[link_corner].min() if link_corner.any() else np.inf
    tightest = int(np.argmin(radius))
    log(
        f"links: largest slip between a segment and its yaw {link_slip:.1f} deg, tightest turn radius {link_radius:.2f} m"
        f" (whole tour: {slip.max():.1f} deg, {radius[tightest]:.2f} m at ({xy[tightest + 1, 0]:.1f}, {xy[tightest + 1, 1]:.1f}))"
    )
    if link_radius < min_radius:
        log(f"WARNING a link turns tighter than {min_radius:.1f} m; move or add an intermediate point to open it up")
    return float(link_slip), float(link_radius)


def main():
    args = parse_args()
    stops = parse_stops(args.stops, args.z)
    waypoints, kinds = stitch(stops, args.spacing)
    slip, radius = check(waypoints, kinds, args.min_radius)

    length = np.linalg.norm(np.diff(waypoints[:, :3], axis=0), axis=1).sum()
    labels = [
        os.path.relpath(stop, PROJECT_DIR) if os.path.isfile(stop) else f"({stop})" for stop in args.stops
    ]
    comments = [
        "Generated by tools/tour_from_points.py",
        "stops in order: " + " -> ".join(labels),
        f"{len(waypoints)} waypoints, {kinds.count('link')} of them on the linking curves, {length:.1f} m in total",
        f"links: cubic Hermite curves sampled every {args.spacing:g} m, yaw along the tangent; "
        f"tightest link turn radius {radius:.2f} m, largest link segment-to-yaw slip {slip:.1f} deg",
        f"z: files keep theirs, loose points and links use {args.z:g}; set trajectory.terrain to follow the ground",
    ]
    source = {
        "stops": labels,
        "spacing_m": args.spacing,
        "z": args.z,
        "tightest_link_turn_radius_m": round(radius, 3),
    }
    written = waypoint_config.save_waypoints(
        args.out, [tuple(row) for row in waypoints], comments=comments, source=source
    )
    log(f"wrote {len(waypoints)} waypoints ({length:.1f} m) to {written}")


if __name__ == "__main__":
    main()
