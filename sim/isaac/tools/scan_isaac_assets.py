#!/usr/bin/env python3
"""List the labelled assets of an Isaac Sim scene, with their world bounding boxes, as JSON.

An asset is the outermost prim carrying a `semantics:labels:class` label (the
SemanticsLabelsAPI the segmentation camera reads); nothing below it is listed.
Direct children of /root/Props and /root/Buildings with no label anywhere in
their subtree are listed too, with "class": null. The ground is skipped.

Each asset: path, name, class, and the axis-aligned world box (center, size)
from UsdGeom.BBoxCache. `ros2 run ninsar_farm make_isaac_farm_dsg` turns the file
into a scene graph; its default input is ros/src/ninsar_farm/config/isaac_assets.json,
which is also this tool's default output.

Needs pxr, which needs a headless SimulationApp up first, so it runs under
Isaac Sim's interpreter (ROS 2 sourced) and takes a minute to start:

    source /opt/ros/jazzy/setup.bash
    ~/isaacsim/python.sh ~/workspaces/ninsar-scene-understanding/sim/isaac/tools/scan_isaac_assets.py
"""
import argparse
import json
import os
import sys

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # sim/isaac
REPO_DIR = os.path.dirname(os.path.dirname(PROJECT_DIR))
DEFAULT_USD = os.path.join(PROJECT_DIR, "assets", "Montoldre_V4_small.usd")
DEFAULT_OUT = os.path.join(REPO_DIR, "ros", "src", "ninsar_farm", "config", "isaac_assets.json")
LABEL_ATTR = "semantics:labels:class"
UNLABELLED_GROUPS = ("/root/Props", "/root/Buildings")
SKIP_CLASSES = {"ground"}


def parse_args():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("usd", nargs="?", default=DEFAULT_USD, help=f"scene to scan (default: {DEFAULT_USD})")
    ap.add_argument("--out", default=DEFAULT_OUT, help=f"output JSON (default: {DEFAULT_OUT}; relative paths are from the cwd of python.sh)")
    args = ap.parse_args()
    if not os.path.isfile(args.usd):
        ap.error(f"{args.usd}: no such file")
    args.usd, args.out = os.path.abspath(args.usd), os.path.abspath(args.out)
    return args


def main(args):
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.Open(args.usd)
    if stage is None:
        raise SystemExit(f"could not open {args.usd}")
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])

    def label(prim):
        attr = prim.GetAttribute(LABEL_ATTR)
        return list(attr.Get())[0] if attr and attr.HasAuthoredValue() and attr.Get() else None

    def asset(prim, cls):
        box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        lo, hi = box.GetMin(), box.GetMax()
        return {"path": str(prim.GetPath()), "name": prim.GetName(), "class": cls,
                "center": [round((lo[i] + hi[i]) / 2, 4) for i in range(3)],
                "size": [round(hi[i] - lo[i], 4) for i in range(3)]}

    assets, labelled_roots = [], set()
    it = iter(Usd.PrimRange(stage.GetPseudoRoot()))
    for prim in it:
        cls = label(prim)
        if cls is None:
            continue
        it.PruneChildren()
        labelled_roots.add(prim.GetPath())
        if cls not in SKIP_CLASSES:
            assets.append(asset(prim, cls))

    for group in UNLABELLED_GROUPS:
        for prim in stage.GetPrimAtPath(group).GetChildren():
            if not any(p.GetPath() in labelled_roots for p in Usd.PrimRange(prim)):
                assets.append(asset(prim, None))

    # repo-relative when the scene is in the repo (through sim/isaac/assets), so the file is the same on every machine
    inside = os.path.commonpath([args.usd, REPO_DIR]) == REPO_DIR
    source = os.path.relpath(args.usd, REPO_DIR) if inside else args.usd
    doc = {"source": source, "up_axis": UsdGeom.GetStageUpAxis(stage),
           "meters_per_unit": UsdGeom.GetStageMetersPerUnit(stage), "assets": assets}
    with open(args.out, "w") as f:
        json.dump(doc, f, indent=1)
    counts = {}
    for a in assets:
        counts[a["class"]] = counts.get(a["class"], 0) + 1
    print(f"{len(assets)} assets {counts} -> {args.out}", flush=True)


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
