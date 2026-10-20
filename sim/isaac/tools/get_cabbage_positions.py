#!/usr/bin/env python3
"""List the world position of every cabbage prim in a scene USD.

Prints one `cbg-pos: <prim>,x,y,z` line per Xformable prim whose name contains
--match -- groups and invisible prototypes included, so it shows everything a
name match could pick up; tools/cabbages_to_waypoints.py reads the scene itself
and keeps only the visible innermost matches. Needs pxr, which needs a headless
SimulationApp up first, so it runs under Isaac Sim's interpreter and takes a
minute to start. Kit's own startup log shares stdout, so grep the records out:

    ~/isaacsim/python.sh tools/get_cabbage_positions.py | grep cbg-pos > assets/cabbage_list.txt
"""

import argparse
import os
import sys

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_USD = os.path.join(
    PROJECT_DIR, "assets", "Collected_Montoldre_V3_grouped_with_cabbages", "Montoldre_V3_grouped.usdc"
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("usd", nargs="?", default=DEFAULT_USD, help=f"scene to scan (default: {DEFAULT_USD})")
    parser.add_argument("--match", default="cabbage", help="case-insensitive substring of the prim name")
    args = parser.parse_args()
    if not os.path.isfile(args.usd):
        parser.error(f"{args.usd}: no such file")
    return args


def main(args):
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.Open(args.usd)
    if stage is None:
        raise SystemExit(f"could not open {args.usd}")

    needle = args.match.lower()
    for prim in stage.Traverse():
        if needle in prim.GetName().lower() and prim.IsA(UsdGeom.Xformable):
            p = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
            print(f"cbg-pos: {prim.GetPath()},{p[0]:.4f},{p[1]:.4f},{p[2]:.4f}", flush=True)


if __name__ == "__main__":
    args = parse_args()  # before the simulator boots, so --help and a bad path cost nothing

    from isaacsim import SimulationApp  # noqa: E402  (pxr is importable only once this is instantiated)

    simulation_app = SimulationApp({"headless": True})
    try:
        main(args)
    except Exception:
        import traceback

        print("FAILED\n" + traceback.format_exc(), file=sys.stderr, flush=True)
    finally:
        simulation_app.close()
