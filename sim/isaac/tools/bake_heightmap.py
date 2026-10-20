#!/usr/bin/env python3
"""Bake a scene's terrain mesh into the heightmap .npz that `trajectory.terrain` reads.

Done once per environment. The bake reads the mesh through pxr, which needs a
(headless, window-less) SimulationApp up first, so this runs under Isaac Sim's
interpreter and takes a minute to start; the result is read back with numpy
alone, by run_scene.py and the waypoint tools under either interpreter:

    ~/isaacsim/python.sh tools/bake_heightmap.py \
        --usd assets/Montoldre_V3_grouped_clean.usdc --out assets/montoldre_heightmap.npz

See rigsim/terrain_height.py for what the mesh has to look like and how the
lookup works.
"""

import argparse
import os
import sys

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PROJECT_DIR)

from rigsim.terrain_height import MESH_PATH, TerrainHeight, build_lut  # noqa: E402


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--usd", required=True, help="environment USD to bake the terrain from")
    parser.add_argument("--out", required=True, help="heightmap .npz to write (overwrites)")
    parser.add_argument("--mesh", default=MESH_PATH, help=f"terrain mesh prim (default {MESH_PATH})")
    args = parser.parse_args()
    if not os.path.isfile(args.usd):
        parser.error(f"--usd {args.usd}: no such file")
    return args


def main(args):
    written = build_lut(args.usd, args.out, args.mesh)
    terrain = TerrainHeight(written)
    x0, x1, y0, y1 = terrain.bounds
    print(
        f"[terrain] wrote {written}: {terrain.heights.shape[1]} x {terrain.heights.shape[0]} grid, "
        f"x {x0:.1f}..{x1:.1f}m, y {y0:.1f}..{y1:.1f}m, "
        f"z {terrain.heights.min():.2f}..{terrain.heights.max():.2f}m",
        flush=True,
    )


if __name__ == "__main__":
    args = parse_args()  # before the simulator boots, so --help and a bad path cost nothing

    from isaacsim import SimulationApp  # noqa: E402  (pxr is importable only once this is instantiated)

    simulation_app = SimulationApp({"headless": True})
    try:
        main(args)
    except Exception:
        import traceback

        print("[terrain] FAILED\n" + traceback.format_exc(), flush=True)
    finally:
        simulation_app.close()
