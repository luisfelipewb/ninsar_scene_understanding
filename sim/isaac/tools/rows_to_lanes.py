#!/usr/bin/env python3
"""Drive the lanes beside and between the rows of trees in a scene.

Finds every prim whose name contains --match (default "lemon"), groups the
trees into rows, and writes one continuous path that drives every lane the
rows define: outside the first row, then between each pair of rows, then
outside the last row, with a curved turn between lanes. Two rows give the
three sections: down the side of row 1, turn, up the middle, turn, down the
side of row 2. run_scene.py replays the file like any other:

    ~/isaacsim/python.sh tools/rows_to_lanes.py --out lemon_v4.yaml
    ./python.sh run_scene.py config/montoldre_v4.yaml --waypoints config/waypoints/lemon_v4.yaml

Needs pxr to read the scene, so it runs under Isaac Sim's interpreter like
tools/cabbages_to_waypoints.py, whose plant finding, row finding and turn
geometry this reuses. The difference is where the waypoints go: a lane runs
parallel to its row(s), --offset metres outside an outer row (default half the
row spacing, so every lane is the same distance from the trees it passes) and
midway between two inner rows. Along a lane there is one waypoint abreast of
each tree in the row(s) it borders -- trees standing abreast in both rows
give one waypoint, not two -- with yaw along the direction of travel. z is
the terrain from --heightmap at the lane point, falling back to the height of
the nearest tree when there is no heightmap or the point lies outside it.
"""

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cabbages_to_waypoints as lawnmower  # noqa: E402  (tools/ on the path)
from cabbages_to_waypoints import PROJECT_DIR, log  # noqa: E402

from rigsim import waypoint_config  # noqa: E402  (PROJECT_DIR is on the path once lawnmower is imported)

# Trees closer than this along the row are abreast and share one lane
# waypoint. Well under the metres between trees in a row.
MERGE_GAP = 0.25


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "usd", nargs="?", default=lawnmower.DEFAULT_USD, help=f"scene to scan (default: {lawnmower.DEFAULT_USD})"
    )
    parser.add_argument("--match", default="lemon", help="case-insensitive substring of the tree prims' names")
    parser.add_argument("--under", default=None, help="prim path to search below (default: whole stage)")
    parser.add_argument("--out", required=True, help="output YAML; a bare name lands in config/waypoints/")
    parser.add_argument(
        "--offset", type=float, default=None, help="metres from an outer row to its lane (default: half the row spacing)"
    )
    parser.add_argument("--clearance", type=float, default=0.0, help="metres to add to every z (default 0)")
    parser.add_argument(
        "--overshoot", type=float, default=lawnmower.OVERSHOOT, help="straight run past the lane end before turning"
    )
    parser.add_argument("--arc-step", type=float, default=lawnmower.ARC_STEP_DEG, help="degrees between arc waypoints")
    parser.add_argument(
        "--row-gap", type=float, default=lawnmower.ROW_GAP, help="perpendicular gap that separates two rows"
    )
    parser.add_argument(
        "--heightmap",
        default=lawnmower.DEFAULT_HEIGHTMAP,
        help="baked terrain (tools/bake_heightmap.py) for the z of the lane points; '' to use the trees' heights",
    )
    parser.add_argument(
        "--reverse", action="store_true", help="start from the opposite corner: last lane first, other direction"
    )
    args = parser.parse_args()
    if not os.path.isfile(args.usd):
        parser.error(f"{args.usd}: no such file")
    if not args.match:
        parser.error("--match must not be empty")
    if args.under is not None and not args.under.startswith("/"):
        parser.error(f"--under {args.under}: prim paths are absolute, starting with /")
    if args.heightmap and not os.path.isfile(args.heightmap):
        parser.error(f"--heightmap {args.heightmap}: no such file")
    if args.offset is not None and args.offset <= 0.0:
        parser.error("--offset must be positive")
    if args.arc_step <= 0.0 or args.arc_step > 90.0:
        parser.error("--arc-step must be in (0, 90] degrees")
    if os.sep not in args.out:
        args.out = os.path.join(lawnmower.WAYPOINTS_DIR, args.out)
    return args


def merge_abreast(values, gap):
    """Sorted values with runs closer than `gap` replaced by their mean."""
    values = np.sort(values)
    merged, run = [], [values[0]]
    for value in values[1:]:
        if value - run[-1] > gap:
            merged.append(np.mean(run))
            run = []
        run.append(value)
    merged.append(np.mean(run))
    return np.asarray(merged)


def lane_points(trees, along, across, rows, offset, ground):
    """The lane waypoints as (N, 3) points plus the index arrays of each lane.

    `rows` are index arrays into `trees`, ordered by their across offset.
    Lanes are ordered the same way, so the result feeds build_path() exactly
    as the rows of a cabbage parcel do.
    """
    origin = trees[:, :2].mean(axis=0)
    local = trees[:, :2] - origin
    a_coord, c_coord = local @ along, local @ across
    row_c = np.array([c_coord[row].mean() for row in rows])

    spacing = np.diff(row_c)
    if offset is None:
        offset = 0.5 * spacing.mean() if len(spacing) else 1.0
    lane_c = np.concatenate([[row_c[0] - offset], 0.5 * (row_c[:-1] + row_c[1:]), [row_c[-1] + offset]])
    # The rows each lane runs beside: one for the outer lanes, two in between.
    lane_rows = [[rows[0]]] + [[rows[i], rows[i + 1]] for i in range(len(rows) - 1)] + [[rows[-1]]]

    points, lanes = [], []
    for c, bordering in zip(lane_c, lane_rows):
        neighbours = np.concatenate(bordering)
        indices = []
        for a in merge_abreast(a_coord[neighbours], MERGE_GAP):
            x, y = origin + a * along + c * across
            z = ground(x, y)
            if z is None:
                z = trees[neighbours[np.argmin(np.abs(a_coord[neighbours] - a))], 2]
            indices.append(len(points))
            points.append([x, y, z])
        lanes.append(np.asarray(indices))
    return np.asarray(points), lanes, offset


def main(args):
    trees = lawnmower.find_plants(args.usd, args.match, args.under)
    along, across = lawnmower.field_frame(trees[:, :2])
    rows = lawnmower.group_rows((trees[:, :2] - trees[:, :2].mean(axis=0)) @ across, args.row_gap)
    if len(rows) < 2:
        raise ValueError(f"found {len(trees)} trees but only one row; lanes need at least two rows")
    if args.reverse:
        rows = rows[::-1]
        along = -along
        across = -across
    ground = lawnmower.terrain_lookup(args.heightmap)
    points, lanes, offset = lane_points(trees, along, across, rows, args.offset, ground)
    log(
        f"{len(trees)} trees in {len(rows)} rows of {[len(row) for row in rows]}, "
        f"rows run along ({along[0]:+.3f}, {along[1]:+.3f}); "
        f"{len(lanes)} lanes of {[len(lane) for lane in lanes]} waypoints"
    )

    waypoints, kinds = lawnmower.build_path(points, along, across, lanes, args.overshoot, args.arc_step, ground)
    waypoints[:, 2] += args.clearance
    lane_count, turn_count = len(points), kinds.count("turn")

    length = np.linalg.norm(np.diff(waypoints[:, :3], axis=0), axis=1).sum()
    centre = trees[:, :2].mean(axis=0)
    row_spacing = np.diff([((trees[row, :2] - centre) @ across).mean() for row in rows])
    comments = [
        f"Generated by tools/rows_to_lanes.py from the *{args.match}* prims of "
        f"{os.path.relpath(args.usd, PROJECT_DIR)}" + (f" under {args.under}" if args.under else ""),
        f"{len(trees)} trees in {len(rows)} rows, row spacing "
        + ", ".join(f"{abs(s):.2f}" for s in row_spacing)
        + " m, rows run along ({:+.3f}, {:+.3f})".format(*along),
        f"{len(lanes)} lanes: outside the first row, between each pair, outside the last; "
        f"outer lanes {offset:.2f} m from their row",
        f"one waypoint abreast of each tree, {lane_count} lane points, {turn_count} turn points, "
        f"{length:.1f} m in total",
        f"turns: {args.overshoot:.2f} m overshoot then a semicircle sampled every {args.arc_step:g} deg",
        "z is the ground at each lane point" + (f" + {args.clearance:.2f} m" if args.clearance else ""),
    ]
    source = {
        "scene": os.path.relpath(args.usd, PROJECT_DIR),
        "match": args.match,
        "under": args.under,
        "rows": len(rows),
        "trees_per_row": [int(len(row)) for row in rows],
        "lanes": len(lanes),
        "offset_m": float(offset),
        "overshoot_m": args.overshoot,
        "arc_step_deg": args.arc_step,
        "clearance_m": args.clearance,
        "heightmap": os.path.relpath(args.heightmap, PROJECT_DIR) if args.heightmap else None,
        "reverse": args.reverse,
    }
    written = waypoint_config.save_waypoints(
        args.out, [tuple(row) for row in waypoints], comments=comments, source=source
    )
    log(f"wrote {len(waypoints)} waypoints ({lane_count} lane points, {turn_count} turn points) to {written}")


if __name__ == "__main__":
    args = parse_args()  # before the simulator boots, so --help and a bad path cost nothing

    from isaacsim import SimulationApp  # noqa: E402  (pxr is importable only once this is instantiated)

    simulation_app = SimulationApp({"headless": True})
    try:
        main(args)
    except Exception:
        import traceback

        # SimulationApp.close() hard-exits, which would swallow the traceback.
        log("FAILED\n" + traceback.format_exc())
    finally:
        simulation_app.close()
