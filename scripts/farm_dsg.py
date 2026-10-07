#!/usr/bin/env python3
"""Farm scene graph (docs/FARM_DSG_SPEC.md) as code: layers, the graph builder, the query
helpers of spec §8 and a checker for the spec rules.

spark_dsg makes every higher-layer neighbour a parent, so a node in an overlay has more than
one spark parent and get_parent() returns None. The helpers here follow `contains` edges
only (spec §2); use them instead of get_parent()/parents().

Helpers take a node as a uid string ("r13"), a NodeSymbol or a raw id, and return uid strings.

Usage:
  .venv/bin/python src/ninsar_mapping/scripts/farm_dsg.py verify GRAPH.json   # exit 1 on errors
  .venv/bin/python src/ninsar_mapping/scripts/farm_dsg.py summary GRAPH.json
"""
import argparse
import heapq
import math
import sys
from collections import Counter, defaultdict
from dataclasses import dataclass

import numpy as np
import shapely
import spark_dsg as dsg
from shapely.geometry import Point, shape

DSG_SPEC_VERSION = "0.1"

OBJECTS, ROWS, AREAS, FARM = 1, 2, 3, 4
NAV, AGENTS, OVERLAYS = "n", "a", "o"  # partitions of OBJECTS, OBJECTS, FARM
P0 = 0

# (layer, partition) -> name; a partition is its character, partition 0 is 0
LAYERS = {(FARM, P0): "FARM", (FARM, OVERLAYS): "OVERLAYS", (AREAS, P0): "AREAS", (ROWS, P0): "ROWS",
          (OBJECTS, P0): "OBJECTS", (OBJECTS, NAV): "NAVIGATION", (OBJECTS, AGENTS): "AGENTS"}
OBJECT_PREFIXES = set("ckvieBsmo")  # spec §5.1
PREFIXES = {(FARM, P0): {"F"}, (FARM, OVERLAYS): {"Z"}, (AREAS, P0): {"A"}, (ROWS, P0): {"r"},
            (OBJECTS, P0): OBJECT_PREFIXES, (OBJECTS, NAV): {"w"}, (OBJECTS, AGENTS): {"a"}}
ATTRIBUTE_CLASSES = {(OBJECTS, P0): "ObjectNodeAttributes", (OBJECTS, AGENTS): "AgentNodeAttributes"}

TREE = {"contains"}
OVERLAY = {"member_of"}
SPATIAL = {"next_in_row", "adjacent_row", "adjacent_area", "near"}
NAVIGATION = {"lane", "headland", "road", "entrance"}
ACCESS = {"access"}
AGENT = {"at"}
WEIGHTED = SPATIAL | NAVIGATION | ACCESS  # weight = planar distance (spec §8)

_MEMBERS = [(OBJECTS, P0), (ROWS, P0), (AREAS, P0), (OBJECTS, NAV)]
EDGE_RULES = {  # relation -> allowed unordered endpoint pairs; `contains` is checked by T1/T2/T5
    "member_of": [(m, (FARM, OVERLAYS)) for m in _MEMBERS],
    "next_in_row": [((OBJECTS, P0), (OBJECTS, P0))],
    "adjacent_row": [((ROWS, P0), (ROWS, P0))],
    "adjacent_area": [((AREAS, P0), (AREAS, P0))],
    "near": [((OBJECTS, P0), (OBJECTS, P0))],
    "access": [((OBJECTS, P0), (OBJECTS, NAV))],
    **{rel: [((OBJECTS, NAV), (OBJECTS, NAV))] for rel in NAVIGATION},
    "at": [((OBJECTS, AGENTS), (OBJECTS, NAV))],
}
EXCLUSIVE = {"irrigation_sector": None, "management_zone": "scheme"}  # spec §4 O4: kind -> per attribute
SOURCES, STATUSES = {"prior", "perceived"}, {"expected", "confirmed", "missing"}
GRAPH_METADATA = ("spec_version", "site", "origin", "frame", "units", "thresholds", "labelspaces")
NAV_COLORS = {"lane": (190, 190, 190), "headland": (120, 120, 120), "road": (40, 40, 40), "yard": (160, 160, 120)}


# --- ids --------------------------------------------------------------------------------

def symbol(node):
    """uid string, NodeSymbol or raw id -> NodeSymbol."""
    if isinstance(node, dsg.NodeSymbol):
        return node
    if isinstance(node, str):
        return dsg.NodeSymbol(node[0], int(node[1:]))
    return dsg.NodeSymbol(int(node))


def uid(node):
    return symbol(node).str()


def layer_key(node):
    """(layer, partition) of a SceneGraphNode, with the partition as its character or 0."""
    k = node.layer
    return (k.layer, chr(k.partition) if k.partition else P0)


def all_nodes(G):
    for layer in list(G.layers) + list(G.layer_partitions):
        yield from layer.nodes


def meta(obj):
    """Metadata of node attributes or an edge as a plain dict."""
    info = obj.info if hasattr(obj, "info") else obj
    return dict(info.metadata.get())


def relation(edge):
    return edge.info.metadata.get().get("relation")


def planar_distance(G, a, b):
    pa, pb = G.get_node(symbol(a)).attributes.position, G.get_node(symbol(b)).attributes.position
    return float(np.hypot(pa[0] - pb[0], pa[1] - pb[1]))


# --- builder ----------------------------------------------------------------------------

class FarmGraph:
    """An empty graph with the spec layers, plus node/edge helpers that keep the conventions."""

    def __init__(self):
        self.G = dsg.DynamicSceneGraph(empty=True)
        for (layer, partition), name in LAYERS.items():
            if partition:
                self.G.add_layer(layer, partition, name=name)
            else:
                self.G.add_layer(layer, name=name)

    def add(self, sym, layer, partition=P0, cls=dsg.SemanticNodeAttributes, name="", position=(0, 0, 0),
            color=(0, 0, 0), box=None, label=None, metadata=None):
        """Add a node; `box` is (dims xyz, yaw rad[, centre xyz]), centred on `position` by default."""
        attrs = cls()
        attrs.position = np.array(position, dtype=np.float64)
        if hasattr(attrs, "name"):
            attrs.name = name
            attrs.color = np.array(color, dtype=np.uint8)
        if label is not None:
            attrs.semantic_label = label
        if box is not None:
            dims, yaw, centre = (*box, position) if len(box) == 2 else box
            attrs.bounding_box = dsg.BoundingBox(np.array(dims, dtype=np.float32),
                                                 np.array(centre, dtype=np.float32), float(yaw))
        if metadata:
            attrs.metadata.set(metadata)
        assert self.G.add_node(layer, sym, attrs, partition), sym.str()
        return sym

    def link(self, a, b, rel, **metadata):
        """Edge a -> b tagged with `relation`; weighted relations get the planar distance."""
        info = dsg.EdgeAttributes()
        if rel in WEIGHTED:
            info.weight = planar_distance(self.G, a, b)
            info.weighted = True
        info.metadata.set({"relation": rel, **metadata})
        assert self.G.insert_edge(symbol(a), symbol(b), info), (uid(a), uid(b), rel)


# --- queries (spec §8) ------------------------------------------------------------------

def edges_of(G, node, rel=None):
    """(neighbour uid, edge) pairs of `node`, optionally only for relation `rel`."""
    sym = symbol(node)
    for other in G.get_node(sym).connections():
        e = G.get_edge(sym.value, other)
        if rel is None or relation(e) == rel:
            yield uid(other), e


def neighbours(G, node, rel):
    return [n for n, _ in edges_of(G, node, rel)]


def parent(G, node):
    """The `contains` neighbour in a higher layer, or None (spec §8 parent(n))."""
    layer = G.get_node(symbol(node)).layer.layer
    return next((n for n in neighbours(G, node, "contains") if G.get_node(symbol(n)).layer.layer > layer), None)


def children(G, node):
    layer = G.get_node(symbol(node)).layer.layer
    return [n for n in neighbours(G, node, "contains") if G.get_node(symbol(n)).layer.layer < layer]


def lineage(G, node):
    out, n = [], parent(G, node)
    while n is not None:
        out.append(n)
        n = parent(G, n)
    return out


def descendants(G, node):
    out, todo = [], children(G, node)
    while todo:
        n = todo.pop()
        out.append(n)
        todo += children(G, n)
    return out


def _is_overlay(G, n):
    return layer_key(G.get_node(symbol(n))) == (FARM, OVERLAYS)


def overlays(G, node):
    """Overlays linked to `node` or to any of its ancestors (membership is inherited, O3)."""
    found = set()
    for n in [uid(node)] + lineage(G, node):
        found.update(z for z in neighbours(G, n, "member_of") if _is_overlay(G, z))
    return sorted(found, key=lambda z: symbol(z).category_id)


def members(G, overlay):
    """Nodes linked to the overlay plus their tree descendants."""
    out = set()
    for m in neighbours(G, overlay, "member_of"):
        out.add(m)
        out.update(descendants(G, m))
    return out


def no_entry_waypoints(G):
    out = set()
    for z in G.get_layer(FARM, OVERLAYS).nodes:
        if meta(z.attributes).get("restriction") == "no_entry":
            out.update(n for n in neighbours(G, z.id, "member_of")
                       if layer_key(G.get_node(symbol(n))) == (OBJECTS, NAV))
    return out


def route(G, start, goal, avoid_blocked=True, avoid_no_entry=True):
    """Shortest path over navigation edges; `access` edges only at the two ends.

    Returns (length m, [uids]) or (inf, [])."""
    start, goal = symbol(start).value, symbol(goal).value
    banned = {symbol(w).value for w in no_entry_waypoints(G)} - {start, goal} if avoid_no_entry else set()
    adj = defaultdict(list)
    for e in G.edges:
        m = e.info.metadata.get()
        rel = m.get("relation")
        usable = rel in NAVIGATION or (rel == "access" and {start, goal} & {e.source, e.target})
        if usable and not (avoid_blocked and m.get("blocked")) and not banned & {e.source, e.target}:
            adj[e.source].append((e.target, e.info.weight))
            adj[e.target].append((e.source, e.info.weight))
    dist, prev, queue = {start: 0.0}, {}, [(0.0, start)]
    while queue:
        d, n = heapq.heappop(queue)
        if n == goal:
            path = [n]
            while path[-1] != start:
                path.append(prev[path[-1]])
            return d, [uid(p) for p in reversed(path)]
        if d > dist[n]:
            continue
        for m, w in adj[n]:
            if d + w < dist.get(m, math.inf):
                dist[m], prev[m] = d + w, n
                heapq.heappush(queue, (d + w, m))
    return math.inf, []


# --- checker ----------------------------------------------------------------------------

@dataclass
class Violation:
    rule: str
    uid: object  # node uid, "a-b" for an edge, or None for the graph
    message: str
    level: str = "error"


def verify(G):
    """Check the spec rules that can be read from the graph alone. Returns Violations."""
    out = []
    err = lambda rule, who, msg: out.append(Violation(rule, who, msg))  # noqa: E731
    gmeta = dict(G.metadata.get())
    tol = float((gmeta.get("thresholds") or {}).get("contain_tol_m", 0.1))

    # graph metadata and layers
    for k in GRAPH_METADATA:
        if k not in gmeta:
            err("M", None, f"graph metadata has no '{k}'")
    names = G.layer_names
    for (layer, partition), name in LAYERS.items():
        key = names.get(name)
        if key is None or key.layer != layer or (chr(key.partition) if key.partition else P0) != partition:
            err("L", None, f"layer {name} missing or not at {layer}/{partition}")

    # nodes
    nodes = {}
    for node in all_nodes(G):
        u, key, a = node.id.str(), layer_key(node), node.attributes
        nodes[u] = (key, node)
        m = meta(a)
        if node.id.category not in PREFIXES.get(key, set()):
            err("L", u, f"prefix '{node.id.category}' not allowed in layer {LAYERS.get(key, key)}")
        cls = ATTRIBUTE_CLASSES.get(key, "SemanticNodeAttributes")
        if type(a).__name__ != cls:
            err("L", u, f"attributes are {type(a).__name__}, expected {cls}")
        if m.get("uid") != u:
            err("M", u, f"metadata uid {m.get('uid')!r} differs from the node symbol")
        if not m.get("kind"):
            err("M", u, "no metadata kind")
        if key != (OBJECTS, AGENTS):
            if m.get("source") not in SOURCES:
                err("M", u, f"source {m.get('source')!r} not in {sorted(SOURCES)}")
            if m.get("status") not in STATUSES:
                err("M", u, f"status {m.get('status')!r} not in {sorted(STATUSES)}")
            if not isinstance(m.get("modified"), bool):
                err("M", u, "modified must be a bool")
            c = m.get("confidence")
            if c is not None and not 0.0 <= c <= 1.0:
                err("M", u, f"confidence {c} outside [0, 1]")
        if key in ((OBJECTS, P0), (ROWS, P0)):
            names_ = G.get_labelspace(key[0], 0).labels_to_names
            if a.semantic_label not in names_:
                err("M", u, f"semantic_label {a.semantic_label} not in the layer {key[0]} labelspace")
    farms = [u for u, (key, _) in nodes.items() if key == (FARM, P0)]
    if len(farms) != 1:
        err("T1", None, f"expected one farm node, found {len(farms)}")

    # edges
    tree_parents = defaultdict(list)
    direct_overlays = defaultdict(list)
    for e in G.edges:
        a, b = uid(e.source), uid(e.target)
        who, m = f"{a}-{b}", meta(e)
        rel = m.get("relation")
        ka, kb = nodes[a][0], nodes[b][0]
        if rel == "contains":
            if ka[1] != P0 or kb[1] != P0:
                err("T5", who, "contains edge outside partition 0")
            elif ka[0] == kb[0]:
                err("T2", who, "contains edge inside one layer")
            else:
                hi, lo = (a, b) if ka[0] > kb[0] else (b, a)
                tree_parents[lo].append(hi)
        elif rel in EDGE_RULES:
            if not any({ka, kb} == {p, q} and (ka == kb) == (p == q) for p, q in EDGE_RULES[rel]):
                err("E", who, f"{rel} not allowed between {LAYERS.get(ka, ka)} and {LAYERS.get(kb, kb)}")
            elif rel == "member_of":
                z, n = (a, b) if ka == (FARM, OVERLAYS) else (b, a)
                direct_overlays[n].append(z)
        else:
            err("E", who, f"unknown relation {rel!r}")
            continue
        if rel in WEIGHTED:
            d = planar_distance(G, a, b)
            if not e.info.weighted or abs(e.info.weight - d) > 1e-3:
                err("E", who, f"{rel} weight {e.info.weight:.3f} differs from the distance {d:.3f}")
        if rel == "adjacent_area" and "gap_m" not in m:
            err("E", who, "adjacent_area without gap_m")
        for blocker in m.get("blocked_by", []):
            if blocker not in nodes:
                err("E", who, f"blocked_by {blocker} is not a node")

    # tree (T1, T3)
    shapes = {}

    def geom(u):
        if u not in shapes:
            g = meta(nodes[u][1].attributes).get("geometry")
            shapes[u] = shape(g) if g else None
        return shapes[u]

    for u, (key, node) in nodes.items():
        if key[1] != P0 or key == (FARM, P0):
            continue
        ps = tree_parents.get(u, [])
        if len(ps) != 1:
            err("T1", u, f"{len(ps)} contains parents ({', '.join(ps) or 'none'})")
            continue
        pg = geom(ps[0])
        if pg is not None and pg.geom_type == "Polygon":
            cg = geom(u) or Point(node.attributes.position[:2])
            if not pg.buffer(tol).covers(cg):
                err("T3", u, f"not inside its parent {ps[0]}")
    for u in farms:
        if tree_parents.get(u):
            err("T1", u, "the farm has a contains parent")

    # overlays (O4): inherited membership, top-down
    zinfo = {z: meta(n.attributes) for z, (k, n) in nodes.items() if k == (FARM, OVERLAYS)}
    kids = defaultdict(list)
    for child, ps in tree_parents.items():
        kids[ps[0]].append(child)

    def exclusive_keys(zs):
        ks = Counter()
        for z in zs:
            kind = zinfo[z].get("kind")
            if kind in EXCLUSIVE:
                per = EXCLUSIVE[kind]
                ks[(kind, zinfo[z].get(per)) if per else (kind,)] += 1
        return [k for k, c in ks.items() if c > 1]

    def walk(u, above):
        mine = above | set(direct_overlays.get(u, []))
        for k in exclusive_keys(mine):
            err("O4", u, f"in more than one exclusive {'/'.join(map(str, k))} overlay ({', '.join(sorted(mine))})")
        for c in kids.get(u, []):
            walk(c, mine)

    sys.setrecursionlimit(max(sys.getrecursionlimit(), 10000))
    for u in farms:
        walk(u, set())
    for u, (key, _) in nodes.items():
        if key == (OBJECTS, NAV):
            for k in exclusive_keys(set(direct_overlays.get(u, []))):
                err("O4", u, f"in more than one exclusive {'/'.join(map(str, k))} overlay")

    # service points need access (§5.1)
    access = Counter()
    for e in G.edges:
        if relation(e) == "access":
            access[uid(e.source)] += 1
            access[uid(e.target)] += 1
    for u, (key, _) in nodes.items():
        if key == (OBJECTS, P0) and u[0] == "s" and not access[u]:
            err("§5.1", u, "service point without an access edge")

    # waypoints inside a restricted area are linked to it (§4)
    wps = [u for u, (k, _) in nodes.items() if k == (OBJECTS, NAV)]
    if wps:
        tree = shapely.STRtree([Point(nodes[w][1].attributes.position[:2]) for w in wps])
        for z, m in zinfo.items():
            if m.get("kind") != "restricted_area" or geom(z) is None:
                continue
            for i in tree.query(geom(z).buffer(tol), predicate="covers"):
                if z not in direct_overlays.get(wps[i], []):
                    err("§4", wps[i], f"waypoint inside restricted area {z} is not member_of it")

    # navigation connectivity (warning)
    banned = no_entry_waypoints(G)
    free = [w for w in wps if w not in banned]
    if free:
        root = {w: w for w in free}

        def find(x):
            while root[x] != x:
                root[x] = root[root[x]]
                x = root[x]
            return x

        for e in G.edges:
            m = e.info.metadata.get()
            a, b = uid(e.source), uid(e.target)
            if m.get("relation") in NAVIGATION and not m.get("blocked") and a in root and b in root:
                root[find(a)] = find(b)
        parts = Counter(find(w) for w in free)
        if len(parts) > 1:
            out.append(Violation("nav", None, f"navigation graph has {len(parts)} components "
                                 f"(sizes {sorted(parts.values(), reverse=True)})", "warning"))
    return out


# --- report -----------------------------------------------------------------------------

def summary(G, out=sys.stdout):
    out.write("== layers\n")
    for (layer, partition), name in LAYERS.items():
        lyr = G.get_layer(layer, partition) if partition else G.get_layer(layer)
        out.write(f"  {name:10s} {layer}/{partition}  {lyr.num_nodes():6d} nodes\n")
    out.write("== edges by relation\n")
    counts = Counter(relation(e) for e in G.edges)
    for rel, c in sorted(counts.items(), key=lambda kv: str(kv[0])):
        out.write(f"  {rel:14s} {c}\n")
    blocked = sum(1 for e in G.edges if e.info.metadata.get().get("blocked"))
    if blocked:
        out.write(f"  ({blocked} blocked navigation edges)\n")


def print_violations(violations, out=sys.stdout):
    for v in violations:
        out.write(f"{v.level:7s} {v.rule:5s} {v.uid or '-':12s} {v.message}\n")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["verify", "summary"])
    ap.add_argument("graph", help="spark_dsg graph file (.json or binary)")
    args = ap.parse_args(argv)
    G = dsg.DynamicSceneGraph.load(args.graph)
    summary(G)
    if args.cmd == "summary":
        return 0
    violations = verify(G)
    print_violations(violations)
    n_err = sum(v.level == "error" for v in violations)
    print(f"{G.num_nodes()} nodes, {G.num_edges()} edges, {n_err} errors, {len(violations) - n_err} warnings")
    return 1 if n_err else 0


if __name__ == "__main__":
    sys.exit(main())
