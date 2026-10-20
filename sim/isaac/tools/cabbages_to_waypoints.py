#!/usr/bin/env python3
"""Turn the cabbages of a scene into a lawnmower waypoint YAML.

Finds every prim in the scene USD whose name contains --match (default
"cabbage"; "lemon" for a lemon grove), takes its world position, groups the
plants into rows, and writes a single continuous path that passes over every
one: down one row, a curved turn at the end, back up the next. --under limits
the search to one subtree when the scene has more plants than the ones wanted.
run_scene.py replays the file like any other:

    ~/isaacsim/python.sh tools/cabbages_to_waypoints.py --out cabbage_v4.yaml
    ./python.sh run_scene.py config/montoldre_v4.yaml --waypoints config/waypoints/cabbage_v4.yaml

Needs pxr to read the scene, which needs a headless SimulationApp up first, so
it runs under Isaac Sim's interpreter and takes a minute to start. Kit's
startup log goes to stdout; this script reports on stderr.

A plant is the innermost matching Xformable prim: a group named after its
contents (`CabbageLayout`, say) has matching descendants, so it is a container
rather than a plant, while a material or subset inside a tree model that
carries its name is not Xformable and does not count. Invisible prims are
skipped too, so a scene that keeps its source models under an invisible
`Prototypes` group does not get them as plants.

The rows are found rather than assumed: the direction of the rows is the one
each cabbage's nearest neighbour lies in, and a row is a cluster of cabbages
at the same perpendicular offset. Rows are visited in order of that offset, alternating
direction, and within a row the waypoints are the cabbages themselves, in the
order they are passed, each with a yaw pointing along the direction of travel.

The turn between two rows is a straight overshoot past the last cabbage, a
semicircle whose diameter is the row spacing, and a straight run back to the
first cabbage of the next row -- so the heading swings smoothly through 180
degrees instead of pivoting on the spot. Rows whose ends do not line up get a
longer straight leg on the shorter side; the arc itself always joins the two
rows' centrelines. Yaw is written unwrapped (it keeps counting past 180) so it
reads as continuous through the turns; the interpolator slerps it either way.

A cabbage's position is the origin of its prim, which sits on the ground, so
z in the file is the ground height at each cabbage plus --clearance. The arc
points sample the same terrain from --heightmap, falling back to interpolating
between the two cabbages they join when there is no heightmap or the point
falls outside it. Set `trajectory.terrain` in the run config instead if the
path should follow the ground between waypoints too; then --clearance here is
the height it will hold.
"""

import argparse
import os
import sys

import numpy as np

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PROJECT_DIR)

from rigsim import waypoint_config  # noqa: E402  (needs PROJECT_DIR on the path)

WAYPOINTS_DIR = os.path.join(PROJECT_DIR, "config", "waypoints")

DEFAULT_USD = os.path.join(PROJECT_DIR, "assets", "Montoldre_V4.usdc")

DEFAULT_HEIGHTMAP = os.path.join(PROJECT_DIR, "assets", "montoldre_heightmap.npz")

# Two cabbages further apart than this across the field direction are in
# different rows. Well under the 2.5 m row spacing of the Montoldre parcels and
# well over the few centimetres a cabbage strays from its row's centreline.
ROW_GAP = 0.5

# How far past the last cabbage the straight leg runs before the arc starts,
# so the rig is fully clear of the row before the heading begins to swing.
OVERSHOOT = 1.0

# Angular spacing of the arc samples. 15 degrees on a 1.25 m radius is a chord
# of 33 cm, within the row's own 1 m cabbage spacing.
ARC_STEP_DEG = 15.0


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("usd", nargs="?", default=DEFAULT_USD, help=f"scene to scan (default: {DEFAULT_USD})")
    parser.add_argument("--match", default="cabbage", help="case-insensitive substring of the plant prims' names")
    parser.add_argument("--under", default=None, help="prim path to search below, e.g. one parcel (default: whole stage)")
    parser.add_argument("--out", required=True, help="output YAML; a bare name lands in config/waypoints/")
    parser.add_argument("--clearance", type=float, default=0.0, help="metres to add to every z (default 0)")
    parser.add_argument("--overshoot", type=float, default=OVERSHOOT, help="straight run past the row end before turning")
    parser.add_argument("--arc-step", type=float, default=ARC_STEP_DEG, help="degrees between arc waypoints")
    parser.add_argument("--row-gap", type=float, default=ROW_GAP, help="perpendicular gap that separates two rows")
    parser.add_argument(
        "--heightmap",
        default=DEFAULT_HEIGHTMAP,
        help="baked terrain (tools/bake_heightmap.py) for the z of the turn points; '' to interpolate instead",
    )
    parser.add_argument(
        "--reverse", action="store_true", help="start from the opposite corner: last row first, other direction"
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
    if args.arc_step <= 0.0 or args.arc_step > 90.0:
        parser.error("--arc-step must be in (0, 90] degrees")
    if os.sep not in args.out:
        args.out = os.path.join(WAYPOINTS_DIR, args.out)
    return args


def log(message):
    print(message, file=sys.stderr, flush=True)  # SimulationApp.close() hard-exits, so flush every line


def find_plants(usd_path, match, under=None):
    """World positions, (N, 3), of the visible innermost Xformable prims whose name contains `match`.

    "Innermost" among the Xformables: a plant's own materials and subsets may
    carry the name too, and they are not plants.

    Needs pxr, so a SimulationApp must be up. `under` is a prim path that
    limits the search to its subtree; None searches the whole stage.
    """
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.Open(usd_path)
    if stage is None:
        raise ValueError(f"could not open {usd_path}")
    root = stage.GetPseudoRoot() if under is None else stage.GetPrimAtPath(under)
    if not root:
        raise ValueError(f"{usd_path}: no prim at {under}")

    needle = match.lower()
    # Only Xformables count as matches at all: a tree model carries a
    # `Lemon_001` material and GeomSubset inside it, and those must not make
    # the tree itself look like a group.
    matches = [
        prim for prim in Usd.PrimRange(root) if needle in prim.GetName().lower() and prim.IsA(UsdGeom.Xformable)
    ]
    # A matching prim with a matching descendant is a group, not a plant.
    # (GetAncestorsRange() starts at the path itself, hence the parent.)
    containers = {ancestor for prim in matches for ancestor in prim.GetPath().GetParentPath().GetAncestorsRange()}
    plants = [
        prim
        for prim in matches
        if prim.GetPath() not in containers
        and UsdGeom.Imageable(prim).ComputeVisibility() != UsdGeom.Tokens.invisible
    ]
    if len(plants) < 2:
        raise ValueError(
            f"{usd_path}: found {len(plants)} visible prim(s) named *{match}*"
            + (f" under {under}" if under else "")
            + "; need at least two"
        )

    cache = UsdGeom.XformCache()
    points = [cache.GetLocalToWorldTransform(prim).ExtractTranslation() for prim in plants]
    return np.asarray(points, dtype=np.float64)


def field_frame(xy):
    """Unit vectors (along, across) of the field: along is the rows' direction.

    `along` is the direction plants are closest to each other in: the vector
    from each plant to its nearest neighbour points along its row, since the
    spacing within a row is what is smaller than the spacing between rows.
    The dominant axis of those vectors (sign-blind, so a second-moment
    eigenvector rather than a mean) is the row direction whatever the outline
    of the field -- a short block of two long-spaced rows, or several parcels
    side by side, would fool the principal axis of the cloud itself.

    `along` is flipped to point towards +x so the first row is walked in the
    direction x increases. `across` is `along` rotated +90 degrees, so
    (along, across, z) is right-handed and a turn from a row towards the next
    one at larger `across` is a left turn.
    """
    deltas = xy[:, None, :] - xy[None, :, :]
    distances = np.linalg.norm(deltas, axis=2)
    np.fill_diagonal(distances, np.inf)
    steps = deltas[np.arange(len(xy)), np.argmin(distances, axis=1)]
    steps /= np.linalg.norm(steps, axis=1, keepdims=True)
    eigenvalues, eigenvectors = np.linalg.eigh(steps.T @ steps)
    if eigenvalues[0] > 0.5 * eigenvalues[1]:
        log("WARNING the plants' nearest neighbours do not agree on a row direction; it is a guess")
    along = eigenvectors[:, 1]
    if along[0] < 0.0 or (along[0] == 0.0 and along[1] < 0.0):
        along = -along
    across = np.array([-along[1], along[0]])
    return along, across


def group_rows(across_coord, gap):
    """Indices of the cabbages in each row, rows ordered by their across offset."""
    order = np.argsort(across_coord)
    rows, current = [], [order[0]]
    for previous, index in zip(order[:-1], order[1:]):
        if across_coord[index] - across_coord[previous] > gap:
            rows.append(current)
            current = []
        current.append(index)
    rows.append(current)
    return [np.asarray(row) for row in rows]


def unwrap_yaw_deg(headings):
    """Headings in degrees, each within 180 of the one before it."""
    return np.degrees(np.unwrap(np.radians(headings)))


def build_path(points, along, across, rows, overshoot, arc_step_deg, ground):
    """Waypoints as (N, 6) x,y,z,roll,pitch,yaw plus a matching list of kinds.

    `ground(x, y)` returns the z for a point that is not a cabbage, or None to
    interpolate between the cabbages the turn joins. `kinds` is a parallel list
    of 'cabbage' or 'turn' for the summary.
    """
    origin = points[:, :2].mean(axis=0)
    local = points[:, :2] - origin
    a_coord, c_coord = local @ along, local @ across

    def to_world(a, c):
        return origin + a * along + c * across

    xyz, headings, kinds = [], [], []

    for row_index, row in enumerate(rows):
        forward = row_index % 2 == 0
        sign = 1.0 if forward else -1.0
        ordered = row[np.argsort(sign * a_coord[row])]
        heading = np.degrees(np.arctan2(sign * along[1], sign * along[0]))
        for index in ordered:
            xyz.append(points[index])
            headings.append(heading)
            kinds.append("cabbage")

        if row_index + 1 == len(rows):
            break

        # The turn, worked out in field coordinates: the arc stands past the
        # further of the two row ends, so neither row is clipped short.
        next_row = rows[row_index + 1]
        exit_index = ordered[-1]
        entry_index = next_row[np.argmin(-sign * a_coord[next_row])]
        a_exit, c_exit = a_coord[exit_index], c_coord[row].mean()
        a_entry, c_entry = a_coord[entry_index], c_coord[next_row].mean()
        a_turn = max(sign * a_exit, sign * a_entry) * sign + sign * overshoot
        centre_c = 0.5 * (c_exit + c_entry)
        radius = 0.5 * (c_entry - c_exit)
        z_exit, z_entry = points[exit_index, 2], points[entry_index, 2]

        steps = max(1, int(round(180.0 / arc_step_deg)))
        arc = []
        for step in range(steps + 1):
            phi = np.pi * step / steps
            a = a_turn + sign * radius * np.sin(phi)
            c = centre_c - radius * np.cos(phi)
            tangent = sign * np.cos(phi) * along + np.sin(phi) * across
            arc.append((a, c, np.degrees(np.arctan2(tangent[1], tangent[0]))))

        for fraction, (a, c, heading) in zip(np.linspace(0.0, 1.0, len(arc)), arc):
            x, y = to_world(a, c)
            z = ground(x, y)
            if z is None:
                z = z_exit + fraction * (z_entry - z_exit)
            xyz.append([x, y, z])
            headings.append(heading)
            kinds.append("turn")

    xyz = np.asarray(xyz)
    yaw = unwrap_yaw_deg(np.asarray(headings))
    waypoints = np.column_stack([xyz, np.zeros((len(xyz), 2)), yaw])
    return waypoints, kinds


def terrain_lookup(heightmap):
    """A ground(x, y) sampler over the baked terrain, None beyond its edge."""
    if not heightmap:
        return lambda x, y: None
    from rigsim import terrain_height  # noqa: E402  (numpy-only, but keep the import local to the option)

    terrain = terrain_height.TerrainHeight(heightmap)

    def ground(x, y):
        if terrain.outside(np.array([x]), np.array([y]))[0]:
            return None
        return float(terrain(np.array([x]), np.array([y]))[0])

    return ground


def main(args):
    points = find_plants(args.usd, args.match, args.under)
    along, across = field_frame(points[:, :2])
    rows = group_rows((points[:, :2] - points[:, :2].mean(axis=0)) @ across, args.row_gap)
    if args.reverse:
        rows = rows[::-1]
        along = -along
        across = -across
    log(
        f"{len(points)} cabbages in {len(rows)} rows of {[len(row) for row in rows]}, "
        f"rows run along ({along[0]:+.3f}, {along[1]:+.3f})"
    )

    waypoints, kinds = build_path(
        points, along, across, rows, args.overshoot, args.arc_step, terrain_lookup(args.heightmap)
    )
    waypoints[:, 2] += args.clearance

    length = np.linalg.norm(np.diff(waypoints[:, :3], axis=0), axis=1).sum()
    row_spacing = np.diff([((points[row, :2] - points[:, :2].mean(axis=0)) @ across).mean() for row in rows])
    comments = [
        f"Generated by tools/cabbages_to_waypoints.py from the *{args.match}* prims of "
        f"{os.path.relpath(args.usd, PROJECT_DIR)}" + (f" under {args.under}" if args.under else ""),
        f"{len(points)} cabbages in {len(rows)} rows, row spacing "
        + ", ".join(f"{abs(s):.2f}" for s in row_spacing)
        + " m, rows run along ({:+.3f}, {:+.3f})".format(*along),
        f"lawnmower path: every cabbage is a waypoint, {kinds.count('turn')} turn points, "
        f"{length:.1f} m in total",
        f"turns: {args.overshoot:.2f} m overshoot then a semicircle sampled every {args.arc_step:g} deg",
        f"z is the ground at each cabbage" + (f" + {args.clearance:.2f} m" if args.clearance else ""),
    ]
    source = {
        "scene": os.path.relpath(args.usd, PROJECT_DIR),
        "match": args.match,
        "under": args.under,
        "rows": len(rows),
        "cabbages_per_row": [int(len(row)) for row in rows],
        "overshoot_m": args.overshoot,
        "arc_step_deg": args.arc_step,
        "clearance_m": args.clearance,
        "heightmap": os.path.relpath(args.heightmap, PROJECT_DIR) if args.heightmap else None,
        "reverse": args.reverse,
    }
    written = waypoint_config.save_waypoints(
        args.out, [tuple(row) for row in waypoints], comments=comments, source=source
    )
    log(f"wrote {len(waypoints)} waypoints ({kinds.count('cabbage')} cabbages, {kinds.count('turn')} turn points) to {written}")


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
