#!/usr/bin/env python3
"""Build a handcrafted farm scene graph that exercises spark_dsg's connection model.

Layers (bottom -> top); in an edge between layers the higher layer is the parent:
  1      DETECTIONS  what a robot can detect: crops (cabbage/vine/lemon), buildings, obstacles
  1 'n'  NAVIGATION  partition of layer 1: lane / headland / road waypoints
  2      ROWS        one crop type per row
  3      PARCELS     groups of rows (parcel D mixes crop types)
  4      FARM        root: parent of parcels, and directly of buildings (skips 2 layers)

Two kinds of edges, both tagged with EdgeAttributes metadata {"relation": ...}:
  hierarchy  between layers    "contains"   crop -> row -> parcel -> farm, building -> farm,
                                            obstacle -> parcel it lies in (or farm)
  spatial    inside a layer    "next_in_row", "near", "adjacent_row", "adjacent_parcel"
             layer 1 <-> 'n'   "access" (crop/building <-> waypoint)
             inside 'n'        "lane", "headland", "road", "entrance"
Spatial edges carry weight = planar distance [m]. A lane segment passing over an
obstacle is kept but marked {"blocked": true}, so route queries can avoid it.

Usage:
  ros2 run ninsar_farm make_farm_dsg     # -> ~/data/ninsar/farm_dsg.json
  ros2 launch hydra_visualizer static_visualizer.launch.yaml \
    scene_graph:=$HOME/data/ninsar/farm_dsg.json \
    visualizer_config_path:=$(ros2 pkg prefix --share ninsar_bringup)/config/farm_viz.yaml
"""
import argparse
import heapq
import itertools
import os
from collections import Counter, defaultdict

import numpy as np
import spark_dsg as dsg

DETECTIONS, ROWS, PARCELS, FARM = 1, 2, 3, 4
NAV = "n"  # partition of DETECTIONS
NAV_RELATIONS = {"lane", "headland", "road", "entrance"}

LABELS = {0: "cabbage", 1: "vine", 2: "lemon", 3: "barn", 4: "house", 5: "silo",
          6: "rock", 7: "pole", 8: "water_tank"}
LABEL_IDS = {name: label for label, name in LABELS.items()}
COLORS = {
    "cabbage": (60, 180, 75), "vine": (145, 30, 180), "lemon": (255, 225, 25),
    "barn": (170, 60, 40), "house": (220, 190, 150), "silo": (128, 128, 128),
    "rock": (230, 25, 75), "pole": (245, 130, 48), "water_tank": (0, 130, 200),
    "lane": (190, 190, 190), "headland": (120, 120, 120), "road": (40, 40, 40),
    "parcel": (70, 150, 220), "farm": (0, 0, 0),
}
# crop -> (box dims x/y/z [m], spacing along the row [m])
CROPS = {"cabbage": ((0.5, 0.5, 0.4), 2.0), "vine": ((0.6, 0.6, 1.8), 2.0), "lemon": ((3.0, 3.0, 3.5), 4.0)}

# parcel name, first-row origin (x, y), row length, row spacing, crop of each row
PARCEL_SPECS = [
    ("A", (0.0, 0.0), 18.0, 3.0, ["cabbage"] * 4),
    ("B", (24.0, 0.0), 18.0, 3.0, ["vine"] * 4),
    ("C", (0.0, 18.0), 16.0, 5.0, ["lemon"] * 3),
    ("D", (24.0, 18.0), 18.0, 3.0, ["cabbage", "cabbage", "vine", "vine"]),
]
# name, center (x, y), box dims
BUILDINGS = [("barn", (54.0, 6.0), (12.0, 8.0, 6.0)),
             ("house", (54.0, 26.0), (10.0, 8.0, 5.0)),
             ("silo", (44.0, 36.0), (4.0, 4.0, 10.0))]
OBSTACLES = [("rock", (9.0, 4.5), (0.8, 0.8, 0.5)),         # in a parcel A lane
             ("pole", (33.0, 10.5), (0.3, 0.3, 4.0)),        # on parcel B's outer lane
             ("rock", (8.0, 25.5), (1.0, 1.0, 0.6)),         # in a parcel C lane
             ("water_tank", (46.0, 2.0), (2.0, 2.0, 2.5))]   # outside any parcel, by the barn
# road polylines (x, y); waypoints are shared where they coincide
ROADS = [[(x, 13.5) for x in (-4, 2, 8, 14, 20, 26, 32, 38, 44, 48)],
         [(48, y) for y in (-2, 4, 10, 13.5, 19, 25, 31, 36)]]

HEADLAND = 2.0        # headland width beyond the row ends [m]
LANE_STEP = 4.0       # target waypoint spacing along a lane [m]
NEAR_GAP = 2.0        # "near" if axis-aligned boxes are closer than this [m]
PARCEL_GAP = 6.0      # "adjacent_parcel" if parcel boundaries are closer than this [m]
ENTRANCE_DIST = 8.0   # max headland -> road connection length [m]


class FarmGraph:
    def __init__(self, labels=LABELS, row_labels=None, navigation=True):
        self.G = dsg.DynamicSceneGraph(empty=True)
        self.G.add_layer(DETECTIONS, name="DETECTIONS")
        if navigation:
            self.G.add_layer(DETECTIONS, NAV, name="NAVIGATION")
        self.G.add_layer(ROWS, name="ROWS")
        self.G.add_layer(PARCELS, name="PARCELS")
        self.G.add_layer(FARM, name="FARM")
        self.G.set_labelspace(dsg.Labelspace(labels), DETECTIONS)
        if row_labels is None:
            row_labels = {LABEL_IDS[c]: c for c in CROPS}
        self.G.set_labelspace(dsg.Labelspace(row_labels), ROWS)
        self._ids = defaultdict(itertools.count)

    def add(self, layer, prefix, name, pos, color, dims=None, label=None, cls=dsg.SemanticNodeAttributes,
            partition=0, **meta):
        sym = dsg.NodeSymbol(prefix, next(self._ids[prefix]))
        attrs = cls()
        attrs.position = np.array(pos, dtype=np.float64)
        attrs.name = name
        attrs.color = np.array(color, dtype=np.uint8)
        if label is not None:
            attrs.semantic_label = label
        if dims is not None:
            attrs.bounding_box = dsg.BoundingBox(np.array(dims, dtype=np.float32), np.array(pos, dtype=np.float32))
        if meta:
            attrs.metadata.set(meta)
        assert self.G.add_node(layer, sym, attrs, partition), sym.str()
        return sym

    def link(self, a, b, relation, **meta):
        info = dsg.EdgeAttributes()
        info.weight = float(np.linalg.norm(self.pos(a)[:2] - self.pos(b)[:2]))
        info.weighted = True
        info.metadata.set({"relation": relation, **meta})
        assert self.G.insert_edge(a, b, info), (a.str(), b.str(), relation)

    def pos(self, sym):
        return self.G.get_node(sym).attributes.position

    def waypoint(self, xy, kind):
        return self.add(DETECTIONS, "w", "", (*xy, 0.0), COLORS[kind], partition=NAV, kind=kind)


def box_gap(c1, d1, c2, d2):
    """Planar distance between two axis-aligned boxes (0 if they overlap)."""
    c1, d1, c2, d2 = (np.asarray(v, dtype=float)[:2] for v in (c1, d1, c2, d2))
    return float(np.linalg.norm(np.maximum(0.0, np.abs(c1 - c2) - (d1 + d2) / 2)))


def point_segment_dist(p, a, b):
    p, a, b = (np.asarray(v, dtype=float)[:2] for v in (p, a, b))
    t = np.clip(np.dot(p - a, b - a) / np.dot(b - a, b - a), 0.0, 1.0)
    return float(np.linalg.norm(p - (a + t * (b - a))))


def build():
    f = FarmGraph()
    farm = f.add(FARM, "F", "farm", (25.0, 17.0, 0.0), COLORS["farm"], dims=(68.0, 48.0, 0.1))

    # --- roads: a navigation graph shared by the whole farm
    road = {}
    for polyline in ROADS:
        prev = None
        for xy in polyline:
            if xy not in road:
                road[xy] = f.waypoint(xy, "road")
            if prev is not None:
                f.link(road[prev], road[xy], "road")
            prev = xy

    def nearest_road(xy):
        return min(road.values(), key=lambda w: np.linalg.norm(f.pos(w)[:2] - np.asarray(xy)))

    # --- buildings belong only to the farm (FARM -> DETECTIONS skips ROWS and PARCELS)
    for name, (x, y), dims in BUILDINGS:
        b = f.add(DETECTIONS, "B", name, (x, y, dims[2] / 2), COLORS[name], dims, LABEL_IDS[name],
                  dsg.ObjectNodeAttributes)
        f.link(farm, b, "contains")
        f.link(b, nearest_road((x, y)), "access")

    # --- parcels -> rows -> crops, plus each parcel's lanes and headlands
    parcel_bounds, lane_segments = {}, []
    for pname, (x0, y0), length, spacing, row_crops in PARCEL_SPECS:
        row_ys = [y0 + i * spacing for i in range(len(row_crops))]
        xmin, xmax = x0 - HEADLAND, x0 + length + HEADLAND
        ymin, ymax = row_ys[0] - spacing / 2, row_ys[-1] + spacing / 2
        center = ((xmin + xmax) / 2, (ymin + ymax) / 2, 0.0)
        parcel = f.add(PARCELS, "P", f"parcel_{pname}", center, COLORS["parcel"], (xmax - xmin, ymax - ymin, 0.1),
                       area_m2=round((xmax - xmin) * (ymax - ymin), 1), crops=sorted(set(row_crops)))
        f.link(farm, parcel, "contains")
        parcel_bounds[parcel] = (center, (xmax - xmin, ymax - ymin, 0.1))

        # lanes run between rows and outside the outer rows; their ends are headland waypoints
        lane_ys = [ymin] + [(a + b) / 2 for a, b in zip(row_ys, row_ys[1:])] + [ymax]
        n_wp = int(np.ceil((xmax - xmin) / LANE_STEP)) + 1
        lanes = []
        for ly in lane_ys:
            xs = np.linspace(xmin, xmax, n_wp)
            wps = [f.waypoint((x, ly), "headland" if x in (xmin, xmax) else "lane") for x in xs]
            for a, b in zip(wps, wps[1:]):
                lane_segments.append((a, b))
            lanes.append(wps)
        for side in (0, -1):  # headland paths join the lane ends on each side
            for a, b in zip(lanes, lanes[1:]):
                f.link(a[side], b[side], "headland")
            end = min((lane[side] for lane in lanes), key=lambda w: np.linalg.norm(f.pos(w)[:2] - f.pos(nearest_road(f.pos(w)[:2]))[:2]))
            gate = nearest_road(f.pos(end)[:2])
            if np.linalg.norm(f.pos(end)[:2] - f.pos(gate)[:2]) <= ENTRANCE_DIST:
                f.link(end, gate, "entrance")

        prev_row = None
        for i, (ry, crop) in enumerate(zip(row_ys, row_crops)):
            dims, step = CROPS[crop]
            xs = np.arange(x0, x0 + length + 1e-6, step)
            row_min = (xs[0] - dims[0] / 2, ry - dims[1] / 2)
            row_max = (xs[-1] + dims[0] / 2, ry + dims[1] / 2)
            row_center = ((row_min[0] + row_max[0]) / 2, ry, dims[2] / 2)
            row = f.add(ROWS, "r", f"{pname}{i}", row_center, COLORS[crop],
                        (row_max[0] - row_min[0], row_max[1] - row_min[1], dims[2]), LABEL_IDS[crop],
                        crop=crop, length_m=float(xs[-1] - xs[0]), plant_spacing_m=step, heading_deg=0.0)
            f.link(parcel, row, "contains")
            if prev_row is not None:
                f.link(prev_row, row, "adjacent_row")
            prev_row = row

            prev_crop = None
            for j, x in enumerate(xs):
                c = f.add(DETECTIONS, "c", crop, (x, ry, dims[2] / 2), COLORS[crop], dims, LABEL_IDS[crop],
                          dsg.ObjectNodeAttributes, row_index=j,
                          health=round(0.6 + 0.4 * ((i * 7 + j * 3) % 10) / 10, 2))
                f.link(row, c, "contains")
                if prev_crop is not None:
                    f.link(prev_crop, c, "next_in_row")
                prev_crop = c
                # a crop is reachable from the closest waypoint of each lane beside its row
                for lane in (lanes[i], lanes[i + 1]):
                    f.link(c, min(lane, key=lambda w: np.linalg.norm(f.pos(w)[:2] - (x, ry))), "access")

    for a, b in itertools.combinations(parcel_bounds, 2):
        gap = box_gap(*parcel_bounds[a], *parcel_bounds[b])
        if gap <= PARCEL_GAP:
            f.link(a, b, "adjacent_parcel", gap_m=round(gap, 2))

    # --- obstacles belong to the parcel they lie in, otherwise to the farm
    obstacles = []
    for name, (x, y), dims in OBSTACLES:
        o = f.add(DETECTIONS, "o", name, (x, y, dims[2] / 2), COLORS[name], dims, LABEL_IDS[name],
                  dsg.ObjectNodeAttributes)
        inside = [p for p, (c, d) in parcel_bounds.items() if box_gap(c, d, (x, y), (0, 0)) == 0.0]
        f.link(inside[0] if inside else farm, o, "contains")
        obstacles.append((o, dims))

    # --- lanes: add segments, marking the ones an obstacle sits on as blocked
    for a, b in lane_segments:
        blockers = [o for o, dims in obstacles
                    if point_segment_dist(f.pos(o), f.pos(a), f.pos(b)) < max(dims[:2]) / 2 + 0.5]
        if blockers:
            f.link(a, b, "lane", blocked=True, blocked_by=[o.str() for o in blockers])
        else:
            f.link(a, b, "lane")

    # --- "near": obstacles next to any other detection (crops, buildings)
    detections = [n.id for n in f.G.get_layer(DETECTIONS).nodes]  # node.id is a NodeSymbol
    for o, dims in obstacles:
        for d in detections:
            if d == o:
                continue
            other = f.G.get_node(d).attributes
            if box_gap(f.pos(o), dims, other.position, other.bounding_box.dimensions) < NEAR_GAP:
                f.link(o, d, "near")

    f.G.metadata.add({"site": "demo_farm", "frame": "map", "units": "m"})
    return f.G


# ----------------------------------------------------------------------------- queries
def symbol(node_id):
    """node.id is a NodeSymbol; parents()/children()/edge endpoints are raw ints."""
    return node_id if isinstance(node_id, dsg.NodeSymbol) else dsg.NodeSymbol(node_id)


def name(G, node_id):
    return f"{symbol(node_id).str()}:{G.get_node(node_id).attributes.name or 'wp'}"


def lineage(G, sym):
    """Walk parents up to the root (every node here has a single parent)."""
    chain, node = [symbol(sym).value], G.get_node(sym)
    while node.parents():
        chain.append(next(iter(node.parents())))
        node = G.get_node(chain[-1])
    return " -> ".join(name(G, s) for s in chain)


def relation(edge):
    return edge.info.metadata.get().get("relation")


def neighbours(G, sym, rel):
    """Ids connected to `sym` by edges tagged with relation `rel`."""
    key = symbol(sym).value
    return [e.target if e.source == key else e.source
            for e in G.edges if relation(e) == rel and key in (e.source, e.target)]


def route(G, start, goal, avoid_blocked=True):
    """Dijkstra over the navigation partition; 'access' edges only at the two ends."""
    start, goal = symbol(start).value, symbol(goal).value
    adj = defaultdict(list)
    for e in G.edges:
        meta = e.info.metadata.get()
        rel = meta.get("relation")
        usable = rel in NAV_RELATIONS or (rel == "access" and {start, goal} & {e.source, e.target})
        if usable and not (avoid_blocked and meta.get("blocked")):
            adj[e.source].append((e.target, e.info.weight))
            adj[e.target].append((e.source, e.info.weight))
    dist, prev, queue = {start: 0.0}, {}, [(0.0, start)]
    while queue:
        d, n = heapq.heappop(queue)
        if n == goal:
            path = [n]
            while path[-1] != start:
                path.append(prev[path[-1]])
            return d, path[::-1]
        if d > dist[n]:
            continue
        for m, w in adj[n]:
            if d + w < dist.get(m, np.inf):
                dist[m], prev[m] = d + w, n
                heapq.heappush(queue, (d + w, m))
    return np.inf, []


def find(G, layer, node_name, index=0, partition=0):
    matches = [n.id for n in G.get_layer(layer, partition).nodes if n.attributes.name == node_name]
    return sorted(matches, key=lambda s: s.value)[index]


def report(G):
    print("\n== layers")
    for layer in list(G.layers) + list(G.layer_partitions):
        print(f"  {str(layer.key):6s} {layer.num_nodes():4d} nodes {layer.num_edges():4d} intra-layer edges")
    print("\n== edges by relation")
    for rel, count in sorted(Counter(relation(e) for e in G.edges).items()):
        print(f"  {rel:16s} {count}")

    print("\n== hierarchy (parents)")
    for node_name in ("lemon", "rock", "water_tank", "barn"):
        print("  " + lineage(G, find(G, DETECTIONS, node_name)))

    print("\n== what each parcel contains")
    for parcel in G.get_layer(PARCELS).nodes:
        kids = [G.get_node(c) for c in parcel.children()]
        crops = Counter(G.get_node(c).attributes.name for k in kids if k.layer.layer == ROWS for c in k.children())
        other = [name(G, k.id) for k in kids if k.layer.layer == DETECTIONS]
        print(f"  {parcel.attributes.name}: {len([k for k in kids if k.layer.layer == ROWS])} rows, "
              f"{dict(crops)}, obstacles {other}, metadata {dict(parcel.attributes.metadata.get())}")

    print("\n== spatial neighbours")
    for p in G.get_layer(PARCELS).nodes:
        adj = [G.get_node(n).attributes.name for n in neighbours(G, p.id, "adjacent_parcel")]
        print(f"  {p.attributes.name} adjacent to {adj}")
    for obstacle in G.get_layer(DETECTIONS).nodes:
        if obstacle.id.category == "o":
            print(f"  {name(G, obstacle.id)} near {[name(G, n) for n in neighbours(G, obstacle.id, 'near')]}")
    blocked = Counter(o for e in G.edges for o in e.info.metadata.get().get("blocked_by", []))
    print(f"  blocked lane segments per obstacle: {dict(blocked)}")

    print("\n== routes from the barn to the crops next to each in-field obstacle")
    barn = find(G, DETECTIONS, "barn")
    goals = [n for o in G.get_layer(DETECTIONS).nodes if o.id.category == "o"
             for n in neighbours(G, o.id, "near") if G.get_node(n).attributes.name in CROPS]
    for goal in goals:
        length, path = route(G, barn, goal)
        direct, _ = route(G, barn, goal, avoid_blocked=False)
        print(f"  barn -> {name(G, goal)} in {lineage(G, goal).split(' -> ')[1]}: {length:.1f} m over "
              f"{len(path) - 2} waypoints ({direct:.1f} m if blocked lanes were passable)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-o", "--output", default=os.path.expanduser("~/data/ninsar/farm_dsg.json"), help=".json (readable) or any other extension (binary)")
    ap.add_argument("-q", "--quiet", action="store_true", help="skip the query report")
    args = ap.parse_args()

    G = build()
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    G.save(args.output, include_mesh=False)
    print(f"{G.num_nodes()} nodes, {G.num_edges()} edges -> {args.output}")
    if not args.quiet:
        report(dsg.DynamicSceneGraph.load(args.output))  # query the saved file, proving it round-trips


if __name__ == "__main__":
    main()
