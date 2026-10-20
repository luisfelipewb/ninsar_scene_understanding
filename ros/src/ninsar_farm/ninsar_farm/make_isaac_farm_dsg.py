#!/usr/bin/env python3
"""Build a farm scene graph from the assets of an Isaac Sim scene.

Input is the JSON written by ninsar-isaac-sim's tools/scan_isaac_assets.py (labelled prims and their
world boxes). Same layers and edge conventions as make_farm_dsg.py, minus
the navigation partition:
  1  DETECTIONS  crops, buildings, obstacles (tree, fence, obstacle, unlabelled prop), landmarks
  2  ROWS        found from the plant positions, one crop per row
  3  PARCELS     one per crop class: the rows' extent plus a headland
  4  FARM        root: parent of parcels, buildings, and obstacles outside any parcel

Node labels are the Isaac segmentation class names; each detection keeps its
prim path in metadata. Rows and parcels follow the rows' heading (oriented boxes);
detections keep the scan's axis-aligned boxes.

Usage:
  ~/isaacsim/python.sh ~/workspaces/ninsar-isaac-sim/tools/scan_isaac_assets.py \
    --out ros/src/ninsar_farm/config/isaac_assets.json   # from the repo root
  ros2 run ninsar_farm make_isaac_farm_dsg   # -> ~/data/ninsar/isaac_farm_dsg.json
  ros2 launch hydra_visualizer static_visualizer.launch.yaml \
    scene_graph:=$HOME/data/ninsar/isaac_farm_dsg.json \
    visualizer_config_path:=$(ros2 pkg prefix --share ninsar_bringup)/config/isaac_farm_viz.yaml
"""
import argparse
import json
import os
import sys
from collections import Counter, defaultdict

import numpy as np
import spark_dsg as dsg
from ament_index_python.packages import get_package_share_directory

from ninsar_farm.make_farm_dsg import (DETECTIONS, FARM, PARCELS, ROWS, FarmGraph, box_gap, find,
                           lineage, name, neighbours, relation)

CROPS = ("cabbage", "grapevine", "lemon")
OBSTACLES = ("tree", "fence", "obstacle", "prop")  # "prop": a prop with no label in the scene
LANDMARKS = tuple(f"landmark_d{n}" for n in (4, 6, 8, 10, 12, 20))  # the dice markers under /root/Props/landmarks
LABELS = dict(enumerate(CROPS + ("building",) + OBSTACLES + LANDMARKS))
LABEL_IDS = {label_name: label for label, label_name in LABELS.items()}
COLORS = {
    "cabbage": (60, 180, 75), "grapevine": (145, 30, 180), "lemon": (255, 225, 25),
    "building": (170, 60, 40), "tree": (0, 100, 0), "fence": (245, 130, 48),
    "obstacle": (230, 25, 75), "prop": (0, 130, 200), "parcel": (70, 150, 220), "farm": (0, 0, 0),
    **{landmark: (240, 50, 230) for landmark in LANDMARKS},
}

ROW_GAP = 1.0      # plants further apart than this across the row heading are in different rows [m]
HEADLAND = 2.0     # parcel margin beyond the row ends [m]
NEAR_GAP = 2.0     # obstacle "near" a crop/building if their boxes are closer than this [m]


def row_heading(xy):
    """Direction rows run in [rad, 0..pi): the mean nearest-neighbour direction, axial (mod pi)."""
    d = np.linalg.norm(xy[:, None] - xy[None], axis=2)
    np.fill_diagonal(d, np.inf)
    v = xy[d.argmin(axis=1)] - xy
    theta = np.arctan2(v[:, 1], v[:, 0])
    return float(np.angle(np.exp(2j * theta).mean()) / 2) % np.pi


class Frame:
    """Planar frame along heading h: a = along the rows, b = across them."""

    def __init__(self, h):
        self.h, self.u, self.v = h, np.array([np.cos(h), np.sin(h)]), np.array([-np.sin(h), np.cos(h)])

    def to(self, xy):
        return np.stack([xy @ self.u, xy @ self.v], axis=-1)

    def back(self, ab):
        return ab[0] * self.u + ab[1] * self.v

    def half_extent(self, size):
        """Half-size, along a and b, of an axis-aligned box of `size` seen in this frame."""
        c, s = abs(np.cos(self.h)), abs(np.sin(self.h))
        return np.array([size[0] * c + size[1] * s, size[0] * s + size[1] * c]) / 2

    def inside(self, xy, lo, hi):
        ab = self.to(np.asarray(xy, dtype=float)[:2])
        return bool(np.all(ab >= lo) and np.all(ab <= hi))


def find_rows(plants, frame):
    """Split plants into rows across the heading; each row sorted along it."""
    ab = frame.to(np.array([p["center"][:2] for p in plants]))
    order = np.argsort(ab[:, 1])
    rows, current = [], [order[0]]
    for i, j in zip(order, order[1:]):
        if ab[j, 1] - ab[i, 1] > ROW_GAP:
            rows.append(current)
            current = []
        current.append(j)
    rows.append(current)
    return [[plants[k] for k in sorted(row, key=lambda k: ab[k, 0])] for row in rows]


def extent(plants, frame):
    """(lo, hi) corners, in the frame, of the plants' boxes."""
    ab = frame.to(np.array([p["center"][:2] for p in plants]))
    half = np.array([frame.half_extent(p["size"]) for p in plants])
    return (ab - half).min(axis=0), (ab + half).max(axis=0)


def oriented(f, layer, prefix, node_name, frame, lo, hi, z, height, color, label=None, **meta):
    """Node with a box from frame corners lo/hi, at height z (box centre)."""
    centre = (*frame.back((lo + hi) / 2), z)
    sym = f.add(layer, prefix, node_name, centre, color, label=label, **meta)
    dims = np.array([*(hi - lo), height], dtype=np.float32)
    f.G.get_node(sym).attributes.bounding_box = dsg.BoundingBox(dims, np.array(centre, dtype=np.float32), frame.h)
    return sym


def build(doc):
    f = FarmGraph(labels=LABELS, row_labels={LABEL_IDS[c]: c for c in CROPS}, navigation=False)
    by_class = defaultdict(list)
    for a in doc["assets"]:
        by_class[a["class"] or "prop"].append(a)
    unknown = sorted(set(by_class) - set(LABEL_IDS))
    if unknown:
        print(f"warning: skipping classes with no label: {unknown}", file=sys.stderr)

    def detection(prefix, a, cls, **meta):
        return f.add(DETECTIONS, prefix, a["name"], a["center"], COLORS[cls], a["size"], LABEL_IDS[cls],
                     dsg.ObjectNodeAttributes, prim_path=a["path"], isaac_class=a["class"], **meta)

    # --- parcels -> rows -> crops; the farm node comes last, once its extent is known
    parcels, crops, farm_children = [], [], []
    for crop in CROPS:
        plants = by_class[crop]
        if not plants:
            continue
        frame = Frame(row_heading(np.array([p["center"][:2] for p in plants])))
        rows = find_rows(plants, frame)
        row_bs = [float(np.mean(frame.to(np.array([p["center"][:2] for p in r]))[:, 1])) for r in rows]
        row_spacing = float(np.median(np.diff(row_bs))) if len(rows) > 1 else 2.0
        ground = min(p["center"][2] - p["size"][2] / 2 for p in plants)
        top = max(p["center"][2] + p["size"][2] / 2 for p in plants)

        lo, hi = extent(plants, frame)
        lo = np.array([lo[0] - HEADLAND, row_bs[0] - row_spacing / 2])
        hi = np.array([hi[0] + HEADLAND, row_bs[-1] + row_spacing / 2])
        parcel = oriented(f, PARCELS, "P", f"parcel_{crop}", frame, lo, hi, ground, 0.1, COLORS["parcel"],
                          crop=crop, n_rows=len(rows), n_plants=len(plants),
                          area_m2=round(float(np.prod(hi - lo)), 1), heading_deg=round(np.degrees(frame.h), 1),
                          row_spacing_m=round(row_spacing, 2))
        parcels.append((parcel, frame, lo, hi))
        farm_children.append(parcel)

        prev_row = None
        for i, row in enumerate(rows):
            along = frame.to(np.array([p["center"][:2] for p in row]))[:, 0]
            spacing = float(np.median(np.diff(along))) if len(row) > 1 else 0.0
            rlo, rhi = extent(row, frame)
            r = oriented(f, ROWS, "r", f"{crop}_{i}", frame, rlo, rhi, (ground + top) / 2, top - ground,
                         COLORS[crop], LABEL_IDS[crop], crop=crop, n_plants=len(row),
                         length_m=round(float(along[-1] - along[0]), 2), plant_spacing_m=round(spacing, 2),
                         heading_deg=round(np.degrees(frame.h), 1))
            f.link(parcel, r, "contains")
            if prev_row is not None:
                f.link(prev_row, r, "adjacent_row")
            prev_row = r

            prev_crop = None
            for j, p in enumerate(row):
                c = detection("c", p, crop, row_index=j)
                f.link(r, c, "contains")
                if prev_crop is not None:
                    f.link(prev_crop, c, "next_in_row")
                prev_crop = c
                crops.append(c)

    # --- buildings belong to the farm
    buildings = [detection("B", a, "building") for a in by_class["building"]]
    farm_children += buildings

    # --- obstacles and landmarks belong to the parcel they stand in, otherwise to the farm
    obstacles, in_parcel = [], []
    for cls in OBSTACLES + LANDMARKS:
        for a in by_class[cls]:
            o = detection("o" if cls in OBSTACLES else "l", a, cls)
            obstacles.append(o)
            inside = [p for p, frame, lo, hi in parcels if frame.inside(a["center"], lo, hi)]
            if inside:
                in_parcel.append((inside[0], o))
            else:
                farm_children.append(o)

    # --- farm: spans the parcels and the buildings (not the whole terrain, nor every tree)
    corners = [np.asarray(c)[:2] for p in parcels + [(b,) for b in buildings]
               for c in f.G.get_node(p[0]).attributes.bounding_box.corners()]
    lo, hi = np.min(corners, axis=0), np.max(corners, axis=0)
    ground = min(f.pos(p[0])[2] for p in parcels)
    farm = f.add(FARM, "F", "farm", (*(lo + hi) / 2, ground), COLORS["farm"], (*(hi - lo), 0.1))
    for child in farm_children:
        f.link(farm, child, "contains")
    for parcel, o in in_parcel:
        f.link(parcel, o, "contains")

    # --- "near": obstacles and landmarks next to a crop or a building
    for o in obstacles:
        oa = f.G.get_node(o).attributes
        for d in crops + buildings:
            da = f.G.get_node(d).attributes
            if box_gap(oa.position, oa.bounding_box.dimensions, da.position, da.bounding_box.dimensions) < NEAR_GAP:
                f.link(o, d, "near")

    f.G.metadata.add({"site": os.path.splitext(os.path.basename(doc["source"]))[0], "source": doc["source"],
                      "frame": "world", "units": "m"})
    return f.G


# ----------------------------------------------------------------------------- report
def report(G):
    print("\n== layers")
    for layer in G.layers:
        print(f"  {str(layer.key):6s} {layer.num_nodes():4d} nodes {layer.num_edges():4d} intra-layer edges")
    print("\n== detections by label")
    labels = Counter(LABELS[n.attributes.semantic_label] for n in G.get_layer(DETECTIONS).nodes)
    print("  " + ", ".join(f"{k} {v}" for k, v in labels.items()))
    print("\n== edges by relation")
    for rel, count in sorted(Counter(relation(e) for e in G.edges).items()):
        print(f"  {rel:16s} {count}")

    print("\n== parcels")
    for parcel in G.get_layer(PARCELS).nodes:
        a = parcel.attributes
        meta = a.metadata.get()
        kids = [G.get_node(c) for c in parcel.children()]
        rows = [k for k in kids if k.layer.layer == ROWS]
        other = [name(G, k.id) for k in kids if k.layer.layer == DETECTIONS]
        print(f"  {a.name}: centre ({a.position[0]:.1f}, {a.position[1]:.1f}) "
              f"box {a.bounding_box.dimensions[0]:.1f} x {a.bounding_box.dimensions[1]:.1f} m "
              f"heading {meta['heading_deg']} deg, rows {[len(r.children()) for r in rows]}, obstacles inside {other}")

    print("\n== buildings")
    for b in G.get_layer(DETECTIONS).nodes:
        if b.id.category == "B":
            a = b.attributes
            print(f"  {a.name:22s} ({a.position[0]:7.1f}, {a.position[1]:7.1f})  "
                  f"{a.bounding_box.dimensions[0]:.0f} x {a.bounding_box.dimensions[1]:.0f} x "
                  f"{a.bounding_box.dimensions[2]:.0f} m")

    print("\n== hierarchy (parents)")
    for node_name in ("lemon", "grapevine1", "BuddhaStatue", "barriere140"):
        print("  " + lineage(G, find(G, DETECTIONS, node_name)))

    print("\n== obstacles and landmarks near crops / buildings")
    near = defaultdict(set)
    for o in G.get_layer(DETECTIONS).nodes:
        if o.id.category in ("o", "l"):
            for n in neighbours(G, o.id, "near"):
                near[G.get_node(n).attributes.name].add(o.attributes.name)
    for target, names in sorted(near.items()):
        shown = sorted(names)
        print(f"  {target:22s} <- {len(names):3d}: {', '.join(shown[:5])}{' ...' if len(shown) > 5 else ''}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("assets", nargs="?",
                    default=os.path.join(get_package_share_directory("ninsar_farm"), "config", "isaac_assets.json"),
                    help="scan_isaac_assets.py output (default: ninsar_farm's config/isaac_assets.json)")
    ap.add_argument("-o", "--output", default=os.path.expanduser("~/data/ninsar/isaac_farm_dsg.json"), help=".json (readable) or any other extension (binary)")
    ap.add_argument("-q", "--quiet", action="store_true", help="skip the report")
    args = ap.parse_args()

    with open(args.assets) as fh:
        G = build(json.load(fh))
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    G.save(args.output, include_mesh=False)
    print(f"{G.num_nodes()} nodes, {G.num_edges()} edges -> {args.output}")
    if not args.quiet:
        report(dsg.DynamicSceneGraph.load(args.output))  # query the saved file, proving it round-trips


if __name__ == "__main__":
    main()
