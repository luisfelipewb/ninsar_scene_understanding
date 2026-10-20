#!/usr/bin/env python3
"""Circle a prim of a scene with the rig always facing it, as a waypoint YAML.

Finds the prim named by --prim in the scene USD, takes its world position as
the centre, and writes --steps waypoints evenly spaced on a circle of --radius
around it at the prim's height plus --z-offset, each with a yaw pointing at the
centre. The file lands in config/waypoints/, named after the prim unless --name
says otherwise, and run_scene.py replays it like any other:

    ~/isaacsim/python.sh tools/circle_facing_center.py assets/Montoldre_V4.usdc --prim Tractor --radius 5
    ./python.sh run_scene.py config/montoldre_v4.yaml --waypoints config/waypoints/Tractor.yaml

--prim is either an absolute prim path (/root/Props/Tractor) or a bare name,
which must match exactly one visible Xformable prim anywhere in the stage; when
it matches several, they are listed so one can be given by path. Needs pxr to
read the scene, which needs a headless SimulationApp up first, so it runs under
Isaac Sim's interpreter and takes a minute to start. Kit's startup log goes to
stdout; this script reports on stderr.

The circle starts at +x from the centre and runs counter-clockwise (yaw
increases), --start-deg rotating the starting point. Yaw is written unwrapped
so it reads as continuous around the circle; roll and pitch stay 0, so a
downward view onto the prim comes from the rig's own mounting pitch. --close
appends the starting waypoint again so the rig completes the full circle
rather than stopping one step short; `trajectory.loop` in the run config keeps
it going round.
"""

import argparse
import math
import os
import sys

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PROJECT_DIR)

from rigsim import waypoint_config  # noqa: E402  (needs PROJECT_DIR on the path)

WAYPOINTS_DIR = os.path.join(PROJECT_DIR, "config", "waypoints")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("usd", help="scene to read the prim from")
    parser.add_argument("--prim", required=True, help="prim to circle: an absolute path, or a name unique in the stage")
    parser.add_argument("--radius", type=float, default=3.0, help="circle radius in metres (default 3)")
    parser.add_argument("--steps", type=int, default=8, help="waypoints around the circle (default 8)")
    parser.add_argument("--z-offset", type=float, default=0.5, help="metres above the prim's origin (default 0.5)")
    parser.add_argument("--start-deg", type=float, default=0.0, help="angle of the first waypoint from +x (default 0)")
    parser.add_argument("--close", action="store_true", help="repeat the first waypoint at the end to complete the circle")
    parser.add_argument("--name", default=None, help="output file name (default: the prim's name); a bare name lands in config/waypoints/")
    args = parser.parse_args()
    if not os.path.isfile(args.usd):
        parser.error(f"{args.usd}: no such file")
    if not args.prim:
        parser.error("--prim must not be empty")
    if args.radius <= 0.0:
        parser.error("--radius must be positive")
    if args.steps < 2:
        parser.error("--steps must be at least 2")
    return args


def log(message):
    print(message, file=sys.stderr, flush=True)  # SimulationApp.close() hard-exits, so flush every line


def find_prim(usd_path, spec):
    """The prim at path `spec`, or the one visible Xformable prim named `spec`.

    Needs pxr, so a SimulationApp must be up. Returns (prim path, world
    position); the prim itself expires with the stage when this returns.
    """
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.Open(usd_path)
    if stage is None:
        raise ValueError(f"could not open {usd_path}")

    if spec.startswith("/"):
        prim = stage.GetPrimAtPath(spec)
        if not prim:
            raise ValueError(f"{usd_path}: no prim at {spec}")
    else:
        matches = [
            prim
            for prim in stage.Traverse()
            if prim.GetName() == spec
            and prim.IsA(UsdGeom.Xformable)
            and UsdGeom.Imageable(prim).ComputeVisibility() != UsdGeom.Tokens.invisible
        ]
        if not matches:
            raise ValueError(f"{usd_path}: no visible Xformable prim named {spec!r}")
        if len(matches) > 1:
            paths = "\n  ".join(str(prim.GetPath()) for prim in matches)
            raise ValueError(f"{usd_path}: {len(matches)} prims named {spec!r}; give one by path:\n  {paths}")
        prim = matches[0]

    if not prim.IsA(UsdGeom.Xformable):
        raise ValueError(f"{usd_path}: {prim.GetPath()} is not Xformable, so it has no position")
    position = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
    return prim.GetPath(), (float(position[0]), float(position[1]), float(position[2]))


def circle_waypoints(centre, radius, steps, z_offset, start_deg=0.0, close=False):
    """(x, y, z, roll, pitch, yaw) around `centre`, every yaw pointing at it.

    Counter-clockwise from `start_deg`; the yaw is the angle + 180, left
    unwrapped so it climbs steadily through the circle.
    """
    cx, cy, cz = centre
    count = steps + 1 if close else steps
    waypoints = []
    for index in range(count):
        angle_deg = start_deg + 360.0 * index / steps
        angle = math.radians(angle_deg)
        x = cx + radius * math.cos(angle)
        y = cy + radius * math.sin(angle)
        waypoints.append((x, y, cz + z_offset, 0.0, 0.0, angle_deg + 180.0))
    return waypoints


def main(args):
    prim_path, centre = find_prim(args.usd, args.prim)
    log(f"{prim_path} at ({centre[0]:.3f}, {centre[1]:.3f}, {centre[2]:.3f})")

    waypoints = circle_waypoints(centre, args.radius, args.steps, args.z_offset, args.start_deg, args.close)

    name = args.name or prim_path.name + ".yaml"
    if not name.endswith((".yaml", ".yml")):
        name += ".yaml"
    out = name if os.sep in name else os.path.join(WAYPOINTS_DIR, name)

    comments = [
        f"Generated by tools/circle_facing_center.py around {prim_path} of {os.path.relpath(args.usd, PROJECT_DIR)}",
        f"centre ({centre[0]:.3f}, {centre[1]:.3f}, {centre[2]:.3f}), radius {args.radius:g} m, "
        f"{args.steps} steps counter-clockwise from {args.start_deg:g} deg" + (", closed" if args.close else ""),
        f"z is the prim's origin + {args.z_offset:g} m; every yaw faces the centre",
    ]
    source = {
        "scene": os.path.relpath(args.usd, PROJECT_DIR),
        "prim": str(prim_path),
        "centre": [round(value, 3) for value in centre],
        "radius_m": args.radius,
        "steps": args.steps,
        "z_offset_m": args.z_offset,
        "start_deg": args.start_deg,
        "close": args.close,
    }
    written = waypoint_config.save_waypoints(out, waypoints, comments=comments, source=source)
    log(f"wrote {len(waypoints)} waypoints to {written}")


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
